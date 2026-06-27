#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SimBench Full Analysis - Two Versions
=====================================

Version 1: BASE MODELS ONLY (35 models) - Fair comparison
Version 2: ALL MODELS (51 models) - Complete picture

Generates separate output folders for each version.
"""

import sys
import io
import os
import pandas as pd
import numpy as np
from scipy import stats
from scipy.cluster import hierarchy
from sklearn.preprocessing import StandardScaler
from sklearn.decomposition import PCA
from sklearn.linear_model import LinearRegression
from sklearn.model_selection import cross_val_score
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

# Setup
plt.style.use('seaborn-v0_8-whitegrid')
plt.rcParams['figure.dpi'] = 150
plt.rcParams['savefig.dpi'] = 200
plt.rcParams['font.size'] = 10

# Paths
SCRIPT_DIR = Path(__file__).resolve().parent
DATA_FILE = SCRIPT_DIR / 'out' / 'all_metrics_merged.csv'

# System categories
MBS = ['pendulum', 'slider_crank', 'gear', 'mass_spring_damper', 'particles']
FEA = ['beam', 'buckling', 'rotor', 'tablecloth', 'cable']
SEN = ['gps_imu', 'lidar', 'veh_app', 'camera']
RBT = ['turtlebot', 'viper', 'curiosity', 'vehros', 'sensros', 'handler']
VEH = ['citybus', 'feda', 'gator', 'hmmwv', 'kraz', 'art', 'rigid_highway', 
       'rigid_multipatches', 'scm', 'scm_hill', 'uazbus', 'm113', 'sedan', 'man']

CATEGORY_MAP = {}
for s in MBS: CATEGORY_MAP[s] = 'MBS'
for s in FEA: CATEGORY_MAP[s] = 'FEA'
for s in SEN: CATEGORY_MAP[s] = 'Sensor'
for s in RBT: CATEGORY_MAP[s] = 'Robot'
for s in VEH: CATEGORY_MAP[s] = 'Vehicle'

# Metrics
CODE_METRICS = ['codebleu', 'ngram_match_score', 'weighted_ngram_match_score',
                'syntax_match_score', 'dataflow_match_score',
                'rouge1', 'rouge2', 'rougeL', 'rougeLsum']
LLM_METRICS = ['score_document_norm', 'score_reference_norm', 'score_reference_document_norm']
ALL_METRICS = CODE_METRICS + LLM_METRICS

# Colors
COLORS = ['#FF6B6B', '#4ECDC4', '#667eea', '#f093fb', '#45B7D1', '#96CEB4']


def get_training_type(model_name):
    """Classify model by training type."""
    name_lower = model_name.lower()
    if name_lower.startswith('pe_'):
        return 'Prompt-Engineered'
    elif any(x in name_lower for x in ['-f1', '-f3', 'lora', 'sft']):
        return 'Fine-tuned'
    return 'Base'


def load_and_prepare_data():
    """Load and prepare data with all necessary columns."""
    df = pd.read_csv(DATA_FILE)
    
    # Normalize LLM scores
    df['score_document_norm'] = df['score_document'] / 100
    df['score_reference_norm'] = df['score_reference'] / 100
    df['score_reference_document_norm'] = df['score_reference_document'] / 100
    
    # Add metadata
    df['category'] = df['system'].map(CATEGORY_MAP).fillna('Other')
    df['round_num'] = df['round'].str.extract(r'(\d+)').astype(int)
    df['training_type'] = df['model'].apply(get_training_type)
    
    return df


def run_full_analysis(df, version_name, out_dir):
    """Run complete analysis on given dataframe."""
    
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    
    n_models = df['model'].nunique()
    n_rows = len(df)
    
    print(f"\n{'='*80}")
    print(f"  {version_name}")
    print(f"  Models: {n_models}, Rows: {n_rows}")
    print(f"  Output: {out_dir}")
    print(f"{'='*80}")
    
    results = {}
    
    # ================================================================
    # 1. Model Rankings
    # ================================================================
    print(f"\n  [1/10] Model Rankings...")
    
    model_scores = df.groupby('model')[ALL_METRICS].mean()
    
    # Rank by each metric
    rankings = pd.DataFrame(index=model_scores.index)
    for metric in ALL_METRICS:
        rankings[f'rank_{metric}'] = model_scores[metric].rank(ascending=False)
    rankings['avg_rank'] = rankings.mean(axis=1)
    rankings = rankings.sort_values('avg_rank')
    
    # Save rankings
    full_ranking = model_scores.copy()
    full_ranking['avg_rank'] = rankings['avg_rank']
    full_ranking = full_ranking.sort_values('avg_rank')
    full_ranking.to_csv(out_dir / 'model_rankings.csv', float_format='%.4f')
    
    results['top_model'] = rankings.index[0]
    results['top_score'] = model_scores.loc[rankings.index[0], 'score_reference_document_norm']
    
    # Print top 10
    print(f"\n  Top 10 Models:")
    for i, (model, row) in enumerate(rankings.head(10).iterrows(), 1):
        score = model_scores.loc[model, 'score_reference_document_norm']
        print(f"    {i:2}. {model:35s} Score={score:.3f}")
    
    # ================================================================
    # 2. Round Analysis
    # ================================================================
    print(f"\n  [2/10] Round-by-Round Analysis...")
    
    round_avg = df.groupby('round_num')[ALL_METRICS].mean()
    round_avg.to_csv(out_dir / 'round_performance.csv', float_format='%.4f')
    
    # Calculate improvement
    r1_to_r3 = round_avg.loc[3] - round_avg.loc[1]
    results['round_improvement'] = r1_to_r3['score_reference_document_norm']
    
    print(f"    R1→R3 Improvement (LLM Judge): +{results['round_improvement']:.3f}")
    
    # ================================================================
    # 3. Category Analysis
    # ================================================================
    print(f"\n  [3/10] Category Analysis...")
    
    cat_avg = df.groupby('category')[ALL_METRICS].mean()
    cat_avg = cat_avg.sort_values('score_reference_document_norm', ascending=False)
    cat_avg.to_csv(out_dir / 'category_performance.csv', float_format='%.4f')
    
    results['hardest_cat'] = cat_avg['score_reference_document_norm'].idxmin()
    results['easiest_cat'] = cat_avg['score_reference_document_norm'].idxmax()
    
    print(f"    Easiest: {results['easiest_cat']}, Hardest: {results['hardest_cat']}")
    
    # ================================================================
    # 4. Correlation Analysis
    # ================================================================
    print(f"\n  [4/10] Correlation Analysis (Pearson, Spearman, Kendall)...")
    
    corr_pearson = model_scores.corr(method='pearson')
    corr_spearman = model_scores.corr(method='spearman')
    corr_kendall = model_scores.corr(method='kendall')
    
    corr_pearson.to_csv(out_dir / 'correlation_pearson.csv', float_format='%.4f')
    corr_spearman.to_csv(out_dir / 'correlation_spearman.csv', float_format='%.4f')
    corr_kendall.to_csv(out_dir / 'correlation_kendall.csv', float_format='%.4f')
    
    # Average code-llm correlation
    code_llm_corrs = []
    for cm in CODE_METRICS:
        for lm in LLM_METRICS:
            code_llm_corrs.append(corr_pearson.loc[cm, lm])
    
    results['avg_correlation'] = np.mean(code_llm_corrs)
    print(f"    Avg Code↔LLM Correlation: {results['avg_correlation']:.3f}")
    
    # ================================================================
    # 5. PCA Analysis
    # ================================================================
    print(f"\n  [5/10] PCA Analysis...")
    
    scaler = StandardScaler()
    scaled_data = scaler.fit_transform(model_scores)
    
    pca = PCA()
    pca_result = pca.fit_transform(scaled_data)
    
    results['pca_pc1_var'] = pca.explained_variance_ratio_[0]
    print(f"    PC1 explains {results['pca_pc1_var']*100:.1f}% variance")
    
    # Save PCA loadings
    pca_loadings = pd.DataFrame(
        pca.components_.T,
        index=ALL_METRICS,
        columns=[f'PC{i+1}' for i in range(len(ALL_METRICS))]
    )
    pca_loadings.to_csv(out_dir / 'pca_loadings.csv', float_format='%.4f')
    
    # ================================================================
    # 6. Metric Redundancy
    # ================================================================
    print(f"\n  [6/10] Metric Redundancy Analysis...")
    
    redundant_pairs = []
    for i, m1 in enumerate(ALL_METRICS):
        for m2 in ALL_METRICS[i+1:]:
            r = corr_pearson.loc[m1, m2]
            if abs(r) > 0.9:
                redundant_pairs.append({'metric1': m1, 'metric2': m2, 'correlation': r})
    
    redundant_df = pd.DataFrame(redundant_pairs)
    if len(redundant_df) > 0:
        redundant_df.to_csv(out_dir / 'redundant_pairs.csv', index=False, float_format='%.4f')
    
    results['n_redundant_pairs'] = len(redundant_pairs)
    print(f"    Redundant pairs (r>0.9): {results['n_redundant_pairs']}")
    
    # ================================================================
    # 7. Ranking Agreement
    # ================================================================
    print(f"\n  [7/10] Ranking Agreement (Kendall's W)...")
    
    ranks = model_scores.rank(ascending=False)
    n_models_rank = len(model_scores)
    n_metrics = len(ALL_METRICS)
    
    mean_ranks = ranks.mean(axis=1)
    S = ((mean_ranks - mean_ranks.mean()) ** 2).sum()
    W = 12 * S / (n_metrics**2 * (n_models_rank**3 - n_models_rank))
    
    results['kendalls_w'] = W
    print(f"    Kendall's W: {W:.4f}")
    
    # ================================================================
    # 8. Discrimination Power (Cohen's d)
    # ================================================================
    print(f"\n  [8/10] Discrimination Power (Cohen's d)...")
    
    llm_score = model_scores['score_reference_document_norm']
    top_models = llm_score[llm_score > llm_score.median()].index
    bottom_models = llm_score[llm_score <= llm_score.median()].index
    
    cohens_d = {}
    for metric in ALL_METRICS:
        top_vals = model_scores.loc[top_models, metric]
        bottom_vals = model_scores.loc[bottom_models, metric]
        pooled_std = np.sqrt((top_vals.std()**2 + bottom_vals.std()**2) / 2)
        if pooled_std > 0:
            cohens_d[metric] = (top_vals.mean() - bottom_vals.mean()) / pooled_std
        else:
            cohens_d[metric] = 0
    
    cohens_df = pd.DataFrame.from_dict(cohens_d, orient='index', columns=['cohens_d'])
    cohens_df.to_csv(out_dir / 'cohens_d.csv', float_format='%.4f')
    
    results['max_cohens_d'] = max(cohens_d.values())
    print(f"    Max Cohen's d: {results['max_cohens_d']:.2f}")
    
    # ================================================================
    # 9. Regression Analysis
    # ================================================================
    print(f"\n  [9/10] Regression Analysis...")
    
    X = model_scores[CODE_METRICS]
    y = model_scores['score_reference_document_norm']
    
    model_reg = LinearRegression()
    cv_scores = cross_val_score(model_reg, X, y, cv=min(5, len(model_scores)//2), scoring='r2')
    
    results['regression_r2'] = cv_scores.mean()
    print(f"    R² (Code→LLM, 5-fold CV): {results['regression_r2']:.3f}")
    
    # ================================================================
    # 10. Generate Plots
    # ================================================================
    print(f"\n  [10/10] Generating Plots...")
    
    # Plot 1: Round Improvement
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    fig.suptitle(f'{version_name}: Round-by-Round Performance', fontsize=14, fontweight='bold')
    
    for i, (metrics, title, colors) in enumerate([
        (['codebleu', 'rouge1', 'syntax_match_score'], 'Code Metrics', COLORS[:3]),
        (LLM_METRICS, 'LLM Metrics', COLORS[3:6])
    ]):
        ax = axes[i]
        for j, m in enumerate(metrics):
            label = m.replace('_norm', '').replace('_', ' ')
            ax.plot([1, 2, 3], round_avg[m].values, 'o-', label=label, 
                   linewidth=2.5, markersize=10, color=colors[j])
        ax.set_xlabel('Round')
        ax.set_ylabel('Score')
        ax.set_title(title)
        ax.legend()
        ax.set_xticks([1, 2, 3])
        ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(out_dir / 'round_improvement.png', bbox_inches='tight')
    plt.close()
    
    # Plot 2: Correlation Heatmap
    fig, ax = plt.subplots(figsize=(12, 10))
    mask = np.zeros_like(corr_pearson, dtype=bool)
    mask[np.triu_indices_from(mask, k=1)] = True
    
    sns.heatmap(corr_pearson, mask=mask, annot=True, fmt='.2f', cmap='RdYlGn',
                center=0.5, square=True, linewidths=1, ax=ax, vmin=0, vmax=1,
                annot_kws={'size': 9})
    ax.set_title(f'{version_name}: Metric Correlation Matrix', fontsize=14, fontweight='bold')
    
    plt.tight_layout()
    plt.savefig(out_dir / 'correlation_heatmap.png', bbox_inches='tight')
    plt.close()
    
    # Plot 3: Category Comparison
    fig, ax = plt.subplots(figsize=(10, 6))
    cats = cat_avg.index.tolist()
    x = np.arange(len(cats))
    width = 0.25
    
    ax.bar(x - width, cat_avg['codebleu'], width, label='CodeBLEU', color=COLORS[0])
    ax.bar(x, cat_avg['rouge1'], width, label='ROUGE-1', color=COLORS[1])
    ax.bar(x + width, cat_avg['score_reference_document_norm'], width, label='LLM Judge', color=COLORS[2])
    
    ax.set_xlabel('Category')
    ax.set_ylabel('Score')
    ax.set_title(f'{version_name}: Performance by Category')
    ax.set_xticks(x)
    ax.set_xticklabels(cats)
    ax.legend()
    ax.set_ylim(0, 1)
    
    plt.tight_layout()
    plt.savefig(out_dir / 'category_comparison.png', bbox_inches='tight')
    plt.close()
    
    # Plot 4: PCA
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    
    ax1 = axes[0]
    ax1.scatter(pca_result[:, 0], pca_result[:, 1], alpha=0.7, s=80, c=COLORS[2])
    ax1.set_xlabel(f'PC1 ({pca.explained_variance_ratio_[0]*100:.1f}%)')
    ax1.set_ylabel(f'PC2 ({pca.explained_variance_ratio_[1]*100:.1f}%)')
    ax1.set_title('Model Positions')
    ax1.axhline(0, color='gray', linestyle='--', alpha=0.3)
    ax1.axvline(0, color='gray', linestyle='--', alpha=0.3)
    
    ax2 = axes[1]
    for i, metric in enumerate(ALL_METRICS):
        ax2.arrow(0, 0, pca.components_[0, i]*0.9, pca.components_[1, i]*0.9,
                 head_width=0.03, fc=COLORS[0], ec=COLORS[0])
        ax2.text(pca.components_[0, i]*1.05, pca.components_[1, i]*1.05, 
                metric[:15], fontsize=8)
    ax2.set_xlabel('PC1 Loading')
    ax2.set_ylabel('PC2 Loading')
    ax2.set_title('Metric Loadings')
    ax2.set_xlim(-1, 1)
    ax2.set_ylim(-1, 1)
    
    fig.suptitle(f'{version_name}: PCA Analysis', fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig(out_dir / 'pca_analysis.png', bbox_inches='tight')
    plt.close()
    
    # Plot 5: Top 15 Models Bar Chart
    fig, ax = plt.subplots(figsize=(12, 8))
    top15 = rankings.head(15)
    scores = [model_scores.loc[m, 'score_reference_document_norm'] for m in top15.index]
    
    colors_bar = plt.cm.RdYlGn(np.linspace(0.8, 0.3, 15))
    bars = ax.barh(range(15), scores, color=colors_bar)
    ax.set_yticks(range(15))
    ax.set_yticklabels(top15.index)
    ax.invert_yaxis()
    ax.set_xlabel('LLM Judge Score')
    ax.set_title(f'{version_name}: Top 15 Models')
    
    for bar, score in zip(bars, scores):
        ax.text(score + 0.01, bar.get_y() + bar.get_height()/2, f'{score:.3f}',
               va='center', fontsize=9)
    
    plt.tight_layout()
    plt.savefig(out_dir / 'top15_models.png', bbox_inches='tight')
    plt.close()
    
    # ================================================================
    # Save Summary
    # ================================================================
    summary = pd.DataFrame([results])
    summary.to_csv(out_dir / 'summary.csv', index=False, float_format='%.4f')
    
    print(f"\n  ✓ Analysis complete! Files saved to: {out_dir}")
    
    return results


def main():
    print("=" * 80)
    print("  SIMBENCH FULL ANALYSIS")
    print("  Generating TWO versions: Base-Only & All-Models")
    print("=" * 80)
    
    # Load data
    df = load_and_prepare_data()
    
    print(f"\n  Loaded {len(df)} rows, {df['model'].nunique()} models")
    print(f"  - Base: {df[df['training_type']=='Base']['model'].nunique()}")
    print(f"  - Fine-tuned: {df[df['training_type']=='Fine-tuned']['model'].nunique()}")
    print(f"  - Prompt-Engineered: {df[df['training_type']=='Prompt-Engineered']['model'].nunique()}")
    
    # Version 1: Base models only
    df_base = df[df['training_type'] == 'Base'].copy()
    results_base = run_full_analysis(
        df_base, 
        "VERSION 1: BASE MODELS ONLY (35 models)",
        SCRIPT_DIR / 'out' / 'analysis_base'
    )
    
    # Version 2: All models
    results_all = run_full_analysis(
        df, 
        "VERSION 2: ALL MODELS (51 models)",
        SCRIPT_DIR / 'out' / 'analysis_all'
    )
    
    # Comparison summary
    print("\n" + "=" * 80)
    print("  COMPARISON: BASE vs ALL")
    print("=" * 80)
    
    comparison = pd.DataFrame({
        'Metric': ['Top Model', 'Top Score', 'Avg Correlation', 'PC1 Variance', 
                   'Kendall W', 'Max Cohen d', 'Regression R²'],
        'Base Only': [
            results_base['top_model'], f"{results_base['top_score']:.3f}",
            f"{results_base['avg_correlation']:.3f}", f"{results_base['pca_pc1_var']*100:.1f}%",
            f"{results_base['kendalls_w']:.4f}", f"{results_base['max_cohens_d']:.2f}",
            f"{results_base['regression_r2']:.3f}"
        ],
        'All Models': [
            results_all['top_model'], f"{results_all['top_score']:.3f}",
            f"{results_all['avg_correlation']:.3f}", f"{results_all['pca_pc1_var']*100:.1f}%",
            f"{results_all['kendalls_w']:.4f}", f"{results_all['max_cohens_d']:.2f}",
            f"{results_all['regression_r2']:.3f}"
        ]
    })
    
    print("\n" + comparison.to_string(index=False))
    comparison.to_csv(SCRIPT_DIR / 'out' / 'comparison_base_vs_all.csv', index=False)
    
    print("\n" + "=" * 80)
    print("  ALL DONE!")
    print("=" * 80)
    print(f"\n  Output folders:")
    print(f"    - {SCRIPT_DIR / 'out' / 'analysis_base'}")
    print(f"    - {SCRIPT_DIR / 'out' / 'analysis_all'}")
    print(f"    - {SCRIPT_DIR / 'out' / 'comparison_base_vs_all.csv'}")


if __name__ == "__main__":
    main()
