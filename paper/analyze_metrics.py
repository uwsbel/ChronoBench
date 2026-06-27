#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SimBench Comprehensive Data Analysis
=====================================

Based on all_metrics_merged.csv (15 columns, 5202 rows)

Analysis includes:
1. Overall Statistics & Summary
2. Model Performance Ranking (multiple metrics)
3. Round-by-Round Improvement Analysis
4. System/Task Category Analysis
5. Correlation Analysis (Pearson & Spearman)
6. Statistical Significance Tests
7. Model Analysis by Company, Release Time, Size

Author: SimBench Team
"""

import sys
import io
import warnings
from pathlib import Path

import numpy as np
import pandas as pd
from scipy import stats

# Handle encoding for Windows
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
warnings.filterwarnings('ignore')

# Plotting
import matplotlib.pyplot as plt
import seaborn as sns
from matplotlib import font_manager

# =============================================================================
# Beautiful Plot Styling
# =============================================================================
# Color palettes
PALETTE_MAIN = ['#FF6B6B', '#4ECDC4', '#45B7D1', '#96CEB4', '#FFEAA7', '#DDA0DD', '#98D8C8', '#F7DC6F']
PALETTE_GRADIENT = ['#667eea', '#764ba2', '#f093fb', '#f5576c', '#4facfe', '#00f2fe']
PALETTE_WARM = ['#ff9a9e', '#fecfef', '#fecfef', '#a18cd1', '#fbc2eb', '#a6c1ee']
PALETTE_COOL = ['#a8edea', '#fed6e3', '#d299c2', '#fef9d7', '#d4fc79', '#96e6a1']

# Dark theme colors
BG_DARK = '#1a1a2e'
BG_CARD = '#16213e'
TEXT_LIGHT = '#eaeaea'
ACCENT_1 = '#e94560'
ACCENT_2 = '#0f3460'

def setup_plot_style():
    """Setup beautiful plot styling."""
    plt.style.use('seaborn-v0_8-darkgrid')
    
    plt.rcParams.update({
        # Figure
        'figure.facecolor': '#fafafa',
        'figure.edgecolor': 'none',
        'figure.dpi': 150,
        'savefig.dpi': 200,
        'savefig.facecolor': '#fafafa',
        'savefig.edgecolor': 'none',
        'savefig.bbox': 'tight',
        'savefig.pad_inches': 0.2,
        
        # Axes
        'axes.facecolor': 'white',
        'axes.edgecolor': '#cccccc',
        'axes.linewidth': 1.2,
        'axes.grid': True,
        'axes.titlesize': 14,
        'axes.titleweight': 'bold',
        'axes.titlepad': 15,
        'axes.labelsize': 11,
        'axes.labelweight': 'medium',
        'axes.labelpad': 8,
        'axes.spines.top': False,
        'axes.spines.right': False,
        
        # Grid
        'grid.color': '#e0e0e0',
        'grid.linestyle': '--',
        'grid.linewidth': 0.6,
        'grid.alpha': 0.7,
        
        # Font
        'font.family': 'sans-serif',
        'font.sans-serif': ['Segoe UI', 'Arial', 'Helvetica', 'DejaVu Sans'],
        'font.size': 10,
        
        # Legend
        'legend.frameon': True,
        'legend.framealpha': 0.9,
        'legend.edgecolor': '#cccccc',
        'legend.fontsize': 10,
        'legend.title_fontsize': 11,
        
        # Ticks
        'xtick.labelsize': 10,
        'ytick.labelsize': 10,
        'xtick.color': '#333333',
        'ytick.color': '#333333',
    })

setup_plot_style()

# =============================================================================
# Paths
# =============================================================================
SCRIPT_DIR = Path(__file__).resolve().parent
OUT_DIR = SCRIPT_DIR / "out"
DATA_FILE = OUT_DIR / "all_metrics_merged.csv"
ANALYSIS_DIR = OUT_DIR / "analysis"

# =============================================================================
# System Categories (from evaluatePy.py)
# =============================================================================
MBS_list = ["pendulum", "slider_crank", "gear", "mass_spring_damper", "particles"]
FEA_list = ["beam", "buckling", "rotor", "tablecloth", "cable"]
SEN_list = ["gps_imu", "lidar", "veh_app", "camera"]
RBT_list = ["turtlebot", "viper", "curiosity", "vehros", "sensros", "handler"]
VEH_list = ["citybus", "feda", "gator", "hmmwv", "kraz", "art", "rigid_highway", 
            "rigid_multipatches", "scm", "scm_hill", "uazbus", "m113", "sedan", "man"]

SYSTEM_CATEGORY = {}
for s in MBS_list: SYSTEM_CATEGORY[s] = "MBS"
for s in FEA_list: SYSTEM_CATEGORY[s] = "FEA"
for s in SEN_list: SYSTEM_CATEGORY[s] = "Sensor"
for s in RBT_list: SYSTEM_CATEGORY[s] = "Robot"
for s in VEH_list: SYSTEM_CATEGORY[s] = "Vehicle"

# Metric groups
CODE_SIM_METRICS = ['codebleu', 'ngram_match_score', 'weighted_ngram_match_score',
                    'syntax_match_score', 'dataflow_match_score',
                    'rouge1', 'rouge2', 'rougeL', 'rougeLsum']
LLM_JUDGE_METRICS = ['score_document', 'score_reference', 'score_reference_document']
# Normalized LLM metrics (0-1 range)
LLM_JUDGE_NORM = ['score_document_norm', 'score_reference_norm', 'score_reference_document_norm']
ALL_METRICS = CODE_SIM_METRICS + LLM_JUDGE_NORM

# =============================================================================
# Model Metadata: Company, Release Time, Size
# =============================================================================
MODEL_COMPANY = {
    # OpenAI
    'gpt-4o': 'OpenAI', 'gpt-4o-mini': 'OpenAI', 'gpt-4o-mini-f1': 'OpenAI', 'gpt-4o-mini-f3': 'OpenAI',
    'gpt-4.1': 'OpenAI', 'gpt-4.1-mini': 'OpenAI', 'gpt-4.1-nano': 'OpenAI',
    'o3': 'OpenAI', 'o4-mini': 'OpenAI', 'pe_gpt-4o-mini': 'OpenAI',
    # Anthropic
    'claude-3-5-sonnet': 'Anthropic', 'claude-3-7-sonnet-20250219': 'Anthropic', 
    'claude-4-sonnet-20250514': 'Anthropic',
    # Google
    'Gemini-1.5-pro': 'Google', 'Gemini-2.5-pro': 'Google',
    'gemma-2-27b-it': 'Google', 'gemma-2-9b-it': 'Google', 'gemma-2-2b-it': 'Google', 'gemma-3-1b-it': 'Google',
    # Meta
    'llama-3.1-405b-instruct': 'Meta', 'llama-3.1-70b-instruct': 'Meta', 'llama-3.1-8b-instruct': 'Meta',
    'llama-3.3-70b-instruct': 'Meta', 'llama3.1-8b-f1': 'Meta', 'llama3.1-8b-lora1': 'Meta',
    'llama3.3-70b-lora1': 'Meta', 'llama3.3-70b-sft1': 'Meta', 'llama4-109b-lora1': 'Meta',
    'llama4_maverick': 'Meta', 'llama4_scout': 'Meta',
    'pe_llama-3.1-405b-instruct': 'Meta', 'pe_llama-3.1-70b-instruct': 'Meta', 
    'pe_llama-3.1-8b-instruct': 'Meta', 'pe_llama-3.3-70b-instruct': 'Meta',
    'pe_llama4_maverick': 'Meta', 'pe_llama4_scout': 'Meta',
    # DeepSeek
    'deepseek-r1': 'DeepSeek', 'deepseek-r1-32b': 'DeepSeek', 'deepseek-r1-8b': 'DeepSeek',
    'pe_deepseek-r1-32b': 'DeepSeek', 'pe_deepseek-r1-8b': 'DeepSeek',
    # Mistral
    'codestral-22b-instruct-v0.1': 'Mistral', 'mamba-codestral-7b-v0.1': 'Mistral',
    'mistral-large-latest': 'Mistral', 'mistral-nemo-12b-instruct': 'Mistral',
    'mixtral-8x22b-instruct-v0.1': 'Mistral', 'mixtral-8x7b-instruct-v0.1': 'Mistral',
    # Microsoft
    'phi-3-medium-128k-instruct': 'Microsoft', 'phi-3-mini-128k-instruct': 'Microsoft',
    # NVIDIA
    'nemotron-4-340b-instruct': 'NVIDIA',
    # Alibaba
    'qwen3-235b-a22b': 'Alibaba',
}

# Release time categories (approximate)
MODEL_RELEASE = {
    # 2023 Q4
    'gemma-2-2b-it': '2023-Q4', 'gemma-2-9b-it': '2023-Q4', 'gemma-2-27b-it': '2023-Q4',
    'mixtral-8x7b-instruct-v0.1': '2023-Q4',
    # 2024 Q1-Q2
    'claude-3-5-sonnet': '2024-Q1', 'llama-3.1-8b-instruct': '2024-Q2', 
    'llama-3.1-70b-instruct': '2024-Q2', 'llama-3.1-405b-instruct': '2024-Q2',
    'gpt-4o': '2024-Q2', 'gpt-4o-mini': '2024-Q2',
    'Gemini-1.5-pro': '2024-Q1', 'mistral-large-latest': '2024-Q1',
    'phi-3-medium-128k-instruct': '2024-Q2', 'phi-3-mini-128k-instruct': '2024-Q2',
    'mixtral-8x22b-instruct-v0.1': '2024-Q2', 'nemotron-4-340b-instruct': '2024-Q2',
    # 2024 Q3-Q4
    'llama-3.3-70b-instruct': '2024-Q4', 'deepseek-r1': '2024-Q4', 
    'deepseek-r1-8b': '2024-Q4', 'deepseek-r1-32b': '2024-Q4',
    'codestral-22b-instruct-v0.1': '2024-Q3', 'mamba-codestral-7b-v0.1': '2024-Q3',
    'mistral-nemo-12b-instruct': '2024-Q3', 'qwen3-235b-a22b': '2024-Q4',
    # 2025
    'claude-3-7-sonnet-20250219': '2025-Q1', 'claude-4-sonnet-20250514': '2025-Q2',
    'gpt-4.1': '2025-Q1', 'gpt-4.1-mini': '2025-Q1', 'gpt-4.1-nano': '2025-Q1',
    'o3': '2025-Q1', 'o4-mini': '2025-Q2',
    'Gemini-2.5-pro': '2025-Q1', 'gemma-3-1b-it': '2025-Q1',
    'llama4_maverick': '2025-Q1', 'llama4_scout': '2025-Q1', 'llama4-109b-lora1': '2025-Q1',
    # Fine-tuned/Custom (same as base)
    'gpt-4o-mini-f1': '2024-Q2', 'gpt-4o-mini-f3': '2024-Q2', 'pe_gpt-4o-mini': '2024-Q2',
    'llama3.1-8b-f1': '2024-Q2', 'llama3.1-8b-lora1': '2024-Q2',
    'llama3.3-70b-lora1': '2024-Q4', 'llama3.3-70b-sft1': '2024-Q4',
    'pe_llama-3.1-8b-instruct': '2024-Q2', 'pe_llama-3.1-70b-instruct': '2024-Q2',
    'pe_llama-3.1-405b-instruct': '2024-Q2', 'pe_llama-3.3-70b-instruct': '2024-Q4',
    'pe_llama4_maverick': '2025-Q1', 'pe_llama4_scout': '2025-Q1',
    'pe_deepseek-r1-8b': '2024-Q4', 'pe_deepseek-r1-32b': '2024-Q4',
}

# Model size categories (parameter count)
MODEL_SIZE = {
    # Tiny (<3B)
    'gemma-2-2b-it': 'Tiny (<3B)', 'gemma-3-1b-it': 'Tiny (<3B)',
    # Small (3B-10B)
    'llama-3.1-8b-instruct': 'Small (3-10B)', 'llama3.1-8b-f1': 'Small (3-10B)', 
    'llama3.1-8b-lora1': 'Small (3-10B)', 'pe_llama-3.1-8b-instruct': 'Small (3-10B)',
    'deepseek-r1-8b': 'Small (3-10B)', 'pe_deepseek-r1-8b': 'Small (3-10B)',
    'gemma-2-9b-it': 'Small (3-10B)', 'mamba-codestral-7b-v0.1': 'Small (3-10B)',
    'phi-3-mini-128k-instruct': 'Small (3-10B)',
    # Medium (10B-50B)
    'gemma-2-27b-it': 'Medium (10-50B)', 'mistral-nemo-12b-instruct': 'Medium (10-50B)',
    'codestral-22b-instruct-v0.1': 'Medium (10-50B)', 'deepseek-r1-32b': 'Medium (10-50B)',
    'pe_deepseek-r1-32b': 'Medium (10-50B)', 'phi-3-medium-128k-instruct': 'Medium (10-50B)',
    # Large (50B-100B)
    'llama-3.1-70b-instruct': 'Large (50-100B)', 'llama-3.3-70b-instruct': 'Large (50-100B)',
    'llama3.3-70b-lora1': 'Large (50-100B)', 'llama3.3-70b-sft1': 'Large (50-100B)',
    'pe_llama-3.1-70b-instruct': 'Large (50-100B)', 'pe_llama-3.3-70b-instruct': 'Large (50-100B)',
    'mixtral-8x7b-instruct-v0.1': 'Large (50-100B)',
    # XLarge (100B+)
    'llama-3.1-405b-instruct': 'XLarge (100B+)', 'pe_llama-3.1-405b-instruct': 'XLarge (100B+)',
    'llama4-109b-lora1': 'XLarge (100B+)', 'llama4_maverick': 'XLarge (100B+)', 
    'llama4_scout': 'XLarge (100B+)', 'pe_llama4_maverick': 'XLarge (100B+)', 'pe_llama4_scout': 'XLarge (100B+)',
    'deepseek-r1': 'XLarge (100B+)', 'mixtral-8x22b-instruct-v0.1': 'XLarge (100B+)',
    'nemotron-4-340b-instruct': 'XLarge (100B+)', 'qwen3-235b-a22b': 'XLarge (100B+)',
    'mistral-large-latest': 'XLarge (100B+)',
    # Proprietary (unknown size but likely large)
    'gpt-4o': 'Proprietary', 'gpt-4o-mini': 'Proprietary', 'gpt-4o-mini-f1': 'Proprietary',
    'gpt-4o-mini-f3': 'Proprietary', 'pe_gpt-4o-mini': 'Proprietary',
    'gpt-4.1': 'Proprietary', 'gpt-4.1-mini': 'Proprietary', 'gpt-4.1-nano': 'Proprietary',
    'o3': 'Proprietary', 'o4-mini': 'Proprietary',
    'claude-3-5-sonnet': 'Proprietary', 'claude-3-7-sonnet-20250219': 'Proprietary',
    'claude-4-sonnet-20250514': 'Proprietary',
    'Gemini-1.5-pro': 'Proprietary', 'Gemini-2.5-pro': 'Proprietary',
}


def get_training_type(model_name):
    """Classify model by training type."""
    name_lower = model_name.lower()
    if name_lower.startswith('pe_'):
        return 'Prompt-Engineered'
    elif any(x in name_lower for x in ['-f1', '-f3', 'lora', 'sft', '_f1', '_f3']):
        return 'Fine-tuned'
    else:
        return 'Base'


def load_data():
    """Load and preprocess data."""
    df = pd.read_csv(DATA_FILE)
    df['category'] = df['system'].map(SYSTEM_CATEGORY).fillna('Other')
    df['round_num'] = df['round'].str.extract(r'(\d+)').astype(int)
    
    # Normalize LLM judge scores to 0-1 range
    for col in LLM_JUDGE_METRICS:
        df[col + '_norm'] = df[col] / 100.0
    
    # Add model metadata
    df['company'] = df['model'].map(MODEL_COMPANY).fillna('Other')
    df['release'] = df['model'].map(MODEL_RELEASE).fillna('Unknown')
    df['size'] = df['model'].map(MODEL_SIZE).fillna('Unknown')
    
    # Add training type
    df['training_type'] = df['model'].apply(get_training_type)
    
    return df


def filter_base_models(df):
    """Filter to only base models (no fine-tuning, no prompt engineering)."""
    return df[df['training_type'] == 'Base'].copy()


# =============================================================================
# 1. Overall Statistics
# =============================================================================
def analyze_overall_stats(df):
    """Generate overall statistics summary."""
    print("\n" + "=" * 70)
    print("  1. OVERALL STATISTICS")
    print("=" * 70)
    
    print(f"\n  Dataset: {len(df):,} rows × {len(df.columns)} columns")
    print(f"  Models: {df['model'].nunique()}")
    print(f"  Systems: {df['system'].nunique()}")
    print(f"  Rounds: {df['round'].nunique()}")
    
    # Category distribution
    cat_counts = df.groupby('category')['system'].nunique()
    print(f"\n  Systems per category:")
    for cat, cnt in cat_counts.items():
        print(f"    {cat}: {cnt} systems")
    
    # Metric statistics (all in 0-1 range)
    print(f"\n  Metric Statistics (0-1 range):")
    print("-" * 70)
    stats_df = df[ALL_METRICS].describe().T
    stats_df = stats_df[['mean', 'std', 'min', '25%', '50%', '75%', 'max']]
    stats_df.columns = ['Mean', 'Std', 'Min', '25%', 'Median', '75%', 'Max']
    print(stats_df.round(3).to_string())
    
    # Save stats
    stats_df.to_csv(ANALYSIS_DIR / "metric_statistics.csv", float_format="%.4f")
    
    return stats_df


# =============================================================================
# 2. Model Performance Ranking (BASE MODELS ONLY)
# =============================================================================
def analyze_model_rankings(df, df_base):
    """Rank BASE models by different metrics."""
    print("\n" + "=" * 70)
    print("  2. MODEL PERFORMANCE RANKING (BASE MODELS ONLY)")
    print("=" * 70)
    
    # Aggregate by model (base only)
    model_agg = df_base.groupby('model')[ALL_METRICS].mean()
    
    print(f"\n  Analyzing {len(model_agg)} base models (excluding fine-tuned and PE)")
    
    # Rank by each metric
    rankings = pd.DataFrame(index=model_agg.index)
    for metric in ALL_METRICS:
        rankings[f'rank_{metric}'] = model_agg[metric].rank(ascending=False)
    
    # Average rank
    rankings['avg_rank'] = rankings.mean(axis=1)
    rankings = rankings.sort_values('avg_rank')
    
    # Print top 15
    print(f"\n  Top 15 Base Models (by Average Rank):")
    print("-" * 70)
    
    top15 = rankings.head(15)[['avg_rank']].copy()
    top15['score_ref_doc'] = model_agg.loc[top15.index, 'score_reference_document_norm'].round(3)
    top15['codebleu'] = model_agg.loc[top15.index, 'codebleu'].round(3)
    top15['rouge1'] = model_agg.loc[top15.index, 'rouge1'].round(3)
    top15['avg_rank'] = top15['avg_rank'].round(1)
    top15.insert(0, 'Rank', range(1, 16))
    print(top15.to_string())
    
    # Save full rankings
    full_ranking = model_agg.copy()
    full_ranking['avg_rank'] = rankings['avg_rank']
    full_ranking = full_ranking.sort_values('avg_rank')
    full_ranking.to_csv(ANALYSIS_DIR / "base_model_rankings.csv", float_format="%.4f")
    
    return model_agg, rankings


# =============================================================================
# 3. Round-by-Round Improvement
# =============================================================================
def analyze_round_improvement(df):
    """Analyze performance improvement across rounds."""
    print("\n" + "=" * 70)
    print("  3. ROUND-BY-ROUND IMPROVEMENT ANALYSIS")
    print("=" * 70)
    
    # Average by round
    round_avg = df.groupby('round_num')[ALL_METRICS].mean()
    
    print(f"\n  Average Metrics by Round (0-1 range):")
    print("-" * 70)
    print(round_avg.T.round(3).to_string())
    
    # Improvement from round 1 to 3
    print(f"\n  Improvement (Round 3 - Round 1):")
    print("-" * 70)
    improvement = round_avg.loc[3] - round_avg.loc[1]
    improvement_pct = (improvement / round_avg.loc[1] * 100).round(1)
    
    imp_df = pd.DataFrame({
        'Round1': round_avg.loc[1].round(3),
        'Round3': round_avg.loc[3].round(3),
        'Absolute': improvement.round(3),
        'Relative%': improvement_pct
    })
    print(imp_df.to_string())
    
    # Plot - Beautiful version
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.patch.set_facecolor('#fafafa')
    
    colors_code = ['#FF6B6B', '#4ECDC4', '#45B7D1']
    colors_llm = ['#667eea', '#f093fb', '#f5576c']
    
    # Code similarity metrics
    ax1 = axes[0]
    ax1.set_facecolor('white')
    metrics_code = ['codebleu', 'rouge1', 'syntax_match_score']
    for i, metric in enumerate(metrics_code):
        ax1.plot([1, 2, 3], round_avg[metric].values, 'o-', 
                 label=metric.replace('_', ' ').title(), 
                 linewidth=3, markersize=12, color=colors_code[i],
                 markeredgecolor='white', markeredgewidth=2)
        # Add value labels
        for x, y in zip([1, 2, 3], round_avg[metric].values):
            ax1.annotate(f'{y:.2f}', (x, y), textcoords="offset points", 
                        xytext=(0, 10), ha='center', fontsize=8, fontweight='bold',
                        color=colors_code[i])
    
    ax1.set_xlabel('Round', fontsize=12, fontweight='bold')
    ax1.set_ylabel('Score', fontsize=12, fontweight='bold')
    ax1.set_title('📊 Code Similarity Metrics', fontsize=14, fontweight='bold', pad=15)
    ax1.legend(loc='lower right', framealpha=0.95)
    ax1.set_xticks([1, 2, 3])
    ax1.set_xticklabels(['Round 1\n(Zero-shot)', 'Round 2\n(+ Reference)', 'Round 3\n(+ Feedback)'])
    ax1.set_ylim(0, 1.1)
    ax1.axhspan(0.8, 1.1, alpha=0.1, color='green', label='Excellent')
    
    # LLM judge metrics
    ax2 = axes[1]
    ax2.set_facecolor('white')
    for i, metric in enumerate(LLM_JUDGE_NORM):
        label = metric.replace('_norm', '').replace('score_', '').replace('_', ' ').title()
        ax2.plot([1, 2, 3], round_avg[metric].values, 'o-', 
                 label=label, linewidth=3, markersize=12, color=colors_llm[i],
                 markeredgecolor='white', markeredgewidth=2)
        for x, y in zip([1, 2, 3], round_avg[metric].values):
            ax2.annotate(f'{y:.2f}', (x, y), textcoords="offset points", 
                        xytext=(0, 10), ha='center', fontsize=8, fontweight='bold',
                        color=colors_llm[i])
    
    ax2.set_xlabel('Round', fontsize=12, fontweight='bold')
    ax2.set_ylabel('Score', fontsize=12, fontweight='bold')
    ax2.set_title('🤖 LLM Judge Metrics', fontsize=14, fontweight='bold', pad=15)
    ax2.legend(loc='lower right', framealpha=0.95)
    ax2.set_xticks([1, 2, 3])
    ax2.set_xticklabels(['Round 1\n(Zero-shot)', 'Round 2\n(+ Reference)', 'Round 3\n(+ Feedback)'])
    ax2.set_ylim(0, 0.7)
    
    plt.suptitle('SimBench: Round-by-Round Performance Improvement', 
                 fontsize=16, fontweight='bold', y=1.02)
    plt.tight_layout()
    plt.savefig(ANALYSIS_DIR / "round_improvement.png", bbox_inches='tight')
    plt.close()
    
    return round_avg


# =============================================================================
# 4. System/Category Analysis
# =============================================================================
def analyze_categories(df):
    """Analyze performance by system category."""
    print("\n" + "=" * 70)
    print("  4. SYSTEM CATEGORY ANALYSIS")
    print("=" * 70)
    
    # Average by category
    cat_avg = df.groupby('category')[ALL_METRICS].mean()
    cat_avg = cat_avg.sort_values('score_reference_document_norm', ascending=False)
    
    print(f"\n  Average Metrics by Category (0-1 range):")
    print("-" * 70)
    display_cols = ['codebleu', 'rouge1', 'score_document_norm', 'score_reference_norm', 'score_reference_document_norm']
    print(cat_avg[display_cols].round(3).to_string())
    
    # Difficulty ranking
    print(f"\n  Category Difficulty (by score_reference_document):")
    cat_rank = cat_avg['score_reference_document_norm'].sort_values()
    for i, (cat, score) in enumerate(cat_rank.items(), 1):
        difficulty = 'Harder' if score < cat_avg['score_reference_document_norm'].median() else 'Easier'
        print(f"    {i}. {cat}: {score:.3f} ({difficulty})")
    
    # Plot - Beautiful radar + bar hybrid
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.patch.set_facecolor('#fafafa')
    
    categories = cat_avg.index.tolist()
    cat_icons = {'MBS': '⚙️', 'FEA': '🔧', 'Sensor': '📡', 'Robot': '🤖', 'Vehicle': '🚗'}
    cat_labels = [f"{cat_icons.get(c, '')} {c}" for c in categories]
    
    # Left: Grouped bar chart
    ax1 = axes[0]
    ax1.set_facecolor('white')
    x = np.arange(len(categories))
    width = 0.25
    
    colors = ['#FF6B6B', '#4ECDC4', '#45B7D1']
    bars1 = ax1.bar(x - width, cat_avg['codebleu'], width, label='CodeBLEU', 
                    color=colors[0], edgecolor='white', linewidth=1.5)
    bars2 = ax1.bar(x, cat_avg['rouge1'], width, label='ROUGE-1', 
                    color=colors[1], edgecolor='white', linewidth=1.5)
    bars3 = ax1.bar(x + width, cat_avg['score_reference_document_norm'], width, 
                    label='LLM Judge', color=colors[2], edgecolor='white', linewidth=1.5)
    
    # Add value labels on bars
    for bars in [bars1, bars2, bars3]:
        for bar in bars:
            height = bar.get_height()
            ax1.annotate(f'{height:.2f}', xy=(bar.get_x() + bar.get_width()/2, height),
                        xytext=(0, 3), textcoords="offset points", ha='center', 
                        fontsize=8, fontweight='bold')
    
    ax1.set_xlabel('System Category', fontsize=12, fontweight='bold')
    ax1.set_ylabel('Score (0-1)', fontsize=12, fontweight='bold')
    ax1.set_title('📊 Performance by Category', fontsize=14, fontweight='bold', pad=15)
    ax1.set_xticks(x)
    ax1.set_xticklabels(cat_labels, fontsize=11)
    ax1.set_ylim(0, 0.85)
    ax1.legend(loc='upper right', framealpha=0.95)
    
    # Right: Difficulty ranking (horizontal bar)
    ax2 = axes[1]
    ax2.set_facecolor('white')
    
    difficulty_order = cat_avg['score_reference_document_norm'].sort_values()
    y_pos = np.arange(len(difficulty_order))
    
    # Color gradient from hard (red) to easy (green)
    colors_diff = plt.cm.RdYlGn(np.linspace(0.2, 0.8, len(difficulty_order)))
    
    bars = ax2.barh(y_pos, difficulty_order.values, color=colors_diff, 
                    edgecolor='white', linewidth=1.5, height=0.6)
    
    # Add value labels
    for i, (bar, val) in enumerate(zip(bars, difficulty_order.values)):
        ax2.text(val + 0.01, bar.get_y() + bar.get_height()/2, f'{val:.3f}',
                va='center', fontsize=11, fontweight='bold')
    
    ax2.set_yticks(y_pos)
    ax2.set_yticklabels([f"{cat_icons.get(c, '')} {c}" for c in difficulty_order.index], fontsize=11)
    ax2.set_xlabel('LLM Judge Score', fontsize=12, fontweight='bold')
    ax2.set_title('🎯 Category Difficulty Ranking', fontsize=14, fontweight='bold', pad=15)
    ax2.set_xlim(0, 0.55)
    
    # Add difficulty labels
    ax2.axvline(x=0.35, color='#666', linestyle='--', alpha=0.5)
    ax2.text(0.25, len(y_pos)-0.5, '← Harder', fontsize=10, color='#e74c3c', fontweight='bold')
    ax2.text(0.42, len(y_pos)-0.5, 'Easier →', fontsize=10, color='#27ae60', fontweight='bold')
    
    plt.suptitle('SimBench: System Category Analysis', fontsize=16, fontweight='bold', y=1.02)
    plt.tight_layout()
    plt.savefig(ANALYSIS_DIR / "category_comparison.png", bbox_inches='tight')
    plt.close()
    
    cat_avg.to_csv(ANALYSIS_DIR / "category_performance.csv", float_format="%.4f")
    
    return cat_avg


# =============================================================================
# 5. Correlation Analysis (Model-level Rank Correlation, BASE ONLY)
# =============================================================================
def analyze_correlations(df, df_base):
    """
    Analyze correlations between metrics at MODEL level (BASE MODELS ONLY).
    
    Method:
    1. Aggregate scores per base model (mean across all systems/rounds)
    2. Rank all base models by each metric
    3. Calculate Pearson & Spearman correlation between rankings
    """
    print("\n" + "=" * 70)
    print("  5. CORRELATION ANALYSIS (Base Models, Rank Correlation)")
    print("=" * 70)
    
    # Step 1: Aggregate scores per model (base only)
    model_scores = df_base.groupby('model')[ALL_METRICS].mean()
    print(f"\n  Aggregated {len(model_scores)} base models")
    
    # Step 2: Rank models by each metric (1=best, higher score = better = lower rank)
    model_ranks = pd.DataFrame(index=model_scores.index)
    for metric in ALL_METRICS:
        # ascending=False: higher score gets rank 1
        model_ranks[metric] = model_scores[metric].rank(ascending=False)
    
    print(f"  Ranked models by {len(ALL_METRICS)} metrics")
    
    # Step 3: Calculate correlations on RANKS
    # Pearson on ranks (linear relationship between rankings)
    corr_pearson = model_ranks.corr(method='pearson')
    
    # Spearman on ranks (equivalent to Spearman on original scores)
    corr_spearman = model_ranks.corr(method='spearman')
    
    # Print results
    print(f"\n  A. PEARSON Correlation (on Model Rankings):")
    print("-" * 70)
    
    for llm_metric in LLM_JUDGE_NORM:
        llm_short = llm_metric.replace('_norm', '')
        print(f"\n  {llm_short}:")
        for code_metric in CODE_SIM_METRICS:
            r = corr_pearson.loc[code_metric, llm_metric]
            stars = '***' if r > 0.7 else '**' if r > 0.5 else '*' if r > 0.3 else ''
            print(f"    vs {code_metric:30s}: {r:.3f} {stars}")
    
    print(f"\n  B. SPEARMAN Correlation (on Model Rankings):")
    print("-" * 70)
    
    for llm_metric in LLM_JUDGE_NORM:
        llm_short = llm_metric.replace('_norm', '')
        print(f"\n  {llm_short}:")
        for code_metric in CODE_SIM_METRICS:
            rho = corr_spearman.loc[code_metric, llm_metric]
            stars = '***' if rho > 0.7 else '**' if rho > 0.5 else '*' if rho > 0.3 else ''
            print(f"    vs {code_metric:30s}: {rho:.3f} {stars}")
    
    # Create beautiful correlation heatmap
    fig = plt.figure(figsize=(14, 12))
    fig.patch.set_facecolor('#fafafa')
    
    # Single comprehensive heatmap (since Pearson ≈ Spearman on ranks)
    ax = fig.add_subplot(111)
    ax.set_facecolor('white')
    
    # Custom colormap - purple to teal
    from matplotlib.colors import LinearSegmentedColormap
    colors_cmap = ['#f8f9fa', '#e9ecef', '#adb5bd', '#4ECDC4', '#1a936f', '#114b5f']
    custom_cmap = LinearSegmentedColormap.from_list('custom', colors_cmap, N=256)
    
    # Mask upper triangle
    mask = np.zeros_like(corr_pearson, dtype=bool)
    mask[np.triu_indices_from(mask, k=1)] = True
    
    # Create heatmap
    sns.heatmap(corr_pearson, mask=mask, annot=True, fmt='.2f', cmap=custom_cmap,
                square=True, linewidths=2, linecolor='white',
                ax=ax, vmin=0, vmax=1, cbar_kws={'shrink': 0.8, 'label': 'Correlation'},
                annot_kws={'size': 11, 'weight': 'bold'})
    
    # Style improvements
    ax.set_title('🔗 Metric Correlation Matrix\n(Model-level Rank Correlation)', 
                 fontsize=16, fontweight='bold', pad=20)
    
    # Rotate labels
    ax.set_xticklabels(ax.get_xticklabels(), rotation=45, ha='right', fontsize=10)
    ax.set_yticklabels(ax.get_yticklabels(), rotation=0, fontsize=10)
    
    # Add explanatory text
    fig.text(0.5, 0.02, 
             'Higher values indicate stronger agreement between metric rankings across 51 LLMs',
             ha='center', fontsize=11, style='italic', color='#666')
    
    plt.tight_layout(rect=[0, 0.05, 1, 1])
    plt.savefig(ANALYSIS_DIR / "correlation_heatmap.png", bbox_inches='tight')
    plt.close()
    
    # Save rankings and correlations
    model_ranks.to_csv(ANALYSIS_DIR / "model_ranks_by_metric.csv", float_format="%.0f")
    corr_pearson.to_csv(ANALYSIS_DIR / "correlation_pearson.csv", float_format="%.4f")
    corr_spearman.to_csv(ANALYSIS_DIR / "correlation_spearman.csv", float_format="%.4f")
    
    # Summary comparison
    print(f"\n  C. Pearson vs Spearman Comparison (Code Sim ↔ score_reference_document):")
    print("-" * 70)
    print(f"  {'Metric':<35s} {'Pearson':>10s} {'Spearman':>10s} {'Diff':>10s}")
    for code_metric in CODE_SIM_METRICS:
        p = corr_pearson.loc[code_metric, 'score_reference_document_norm']
        s = corr_spearman.loc[code_metric, 'score_reference_document_norm']
        print(f"  {code_metric:<35s} {p:>10.3f} {s:>10.3f} {s-p:>+10.3f}")
    
    # Show top 10 model rankings example
    print(f"\n  D. Example: Top 10 Models by score_reference_document vs codebleu:")
    print("-" * 70)
    example = model_ranks[['score_reference_document_norm', 'codebleu']].copy()
    example.columns = ['Rank_ScoreRefDoc', 'Rank_CodeBLEU']
    example = example.sort_values('Rank_ScoreRefDoc').head(10)
    example['Rank_ScoreRefDoc'] = example['Rank_ScoreRefDoc'].astype(int)
    example['Rank_CodeBLEU'] = example['Rank_CodeBLEU'].astype(int)
    print(example.to_string())
    
    return corr_pearson, corr_spearman


# =============================================================================
# 6. Statistical Significance Tests
# =============================================================================
def analyze_statistical_tests(df):
    """Perform statistical significance tests."""
    print("\n" + "=" * 70)
    print("  6. STATISTICAL SIGNIFICANCE TESTS")
    print("=" * 70)
    
    # Test 1: Round improvement significance
    print(f"\n  A. Round Improvement Significance (Paired t-test):")
    print("-" * 70)
    
    for metric in ['codebleu', 'rouge1', 'score_reference_document_norm']:
        r1 = df[df['round_num'] == 1].set_index(['model', 'system'])[metric]
        r3 = df[df['round_num'] == 3].set_index(['model', 'system'])[metric]
        common = r1.index.intersection(r3.index)
        
        t_stat, p_val = stats.ttest_rel(r3.loc[common], r1.loc[common])
        sig = '***' if p_val < 0.001 else '**' if p_val < 0.01 else '*' if p_val < 0.05 else 'ns'
        
        mean_diff = (r3.loc[common] - r1.loc[common]).mean()
        print(f"  {metric:35s}: Δ={mean_diff:+.3f}, t={t_stat:.2f}, p={p_val:.4f} {sig}")
    
    # Test 2: Category differences
    print(f"\n  B. Category Differences (One-way ANOVA):")
    print("-" * 70)
    
    for metric in ['codebleu', 'rouge1', 'score_reference_document_norm']:
        groups = [df[df['category'] == cat][metric].dropna() for cat in df['category'].unique()]
        f_stat, p_val = stats.f_oneway(*groups)
        sig = '***' if p_val < 0.001 else '**' if p_val < 0.01 else '*' if p_val < 0.05 else 'ns'
        print(f"  {metric:35s}: F={f_stat:.2f}, p={p_val:.4f} {sig}")


# =============================================================================
# 7. Model Analysis by Company (BASE ONLY)
# =============================================================================
def analyze_by_company(df, df_base):
    """Analyze performance by company (BASE MODELS ONLY)."""
    print("\n" + "=" * 70)
    print("  7. MODEL ANALYSIS BY COMPANY (BASE MODELS ONLY)")
    print("=" * 70)
    
    # Company performance (base only)
    company_avg = df_base.groupby('company')[ALL_METRICS].mean()
    company_avg = company_avg.sort_values('score_reference_document_norm', ascending=False)
    company_count = df_base.groupby('company')['model'].nunique()
    
    print(f"\n  Performance by Company (0-1 range):")
    print("-" * 70)
    
    summary = company_avg[['codebleu', 'rouge1', 'score_reference_document_norm']].round(3)
    summary['n_models'] = company_count
    print(summary.to_string())
    
    # Plot - Beautiful company comparison
    fig, ax = plt.subplots(figsize=(14, 7))
    fig.patch.set_facecolor('#fafafa')
    ax.set_facecolor('white')
    
    companies = company_avg.index.tolist()
    x = np.arange(len(companies))
    width = 0.25
    
    # Company colors (brand-inspired)
    company_colors = {
        'OpenAI': '#10a37f', 'Anthropic': '#d4a574', 'Google': '#4285f4',
        'Meta': '#0668e1', 'DeepSeek': '#1e88e5', 'Mistral': '#ff6f00',
        'Microsoft': '#00a4ef', 'NVIDIA': '#76b900', 'Alibaba': '#ff6a00', 'Other': '#888888'
    }
    
    colors = ['#FF6B6B', '#4ECDC4', '#667eea']
    
    bars1 = ax.bar(x - width, company_avg['codebleu'], width, label='CodeBLEU', 
                   color=colors[0], edgecolor='white', linewidth=1.5, zorder=3)
    bars2 = ax.bar(x, company_avg['rouge1'], width, label='ROUGE-1', 
                   color=colors[1], edgecolor='white', linewidth=1.5, zorder=3)
    bars3 = ax.bar(x + width, company_avg['score_reference_document_norm'], width, 
                   label='LLM Judge', color=colors[2], edgecolor='white', linewidth=1.5, zorder=3)
    
    # Add value labels
    for bars, color in zip([bars1, bars2, bars3], colors):
        for bar in bars:
            height = bar.get_height()
            ax.annotate(f'{height:.2f}', xy=(bar.get_x() + bar.get_width()/2, height),
                        xytext=(0, 3), textcoords="offset points", ha='center', 
                        fontsize=8, fontweight='bold', color='#333')
    
    # Add model count annotations
    for i, company in enumerate(companies):
        count = company_count.get(company, 0)
        ax.annotate(f'n={count}', xy=(x[i], 0.02), ha='center', fontsize=9, 
                    color='#666', style='italic')
    
    ax.set_xlabel('Company', fontsize=12, fontweight='bold')
    ax.set_ylabel('Score (0-1)', fontsize=12, fontweight='bold')
    ax.set_title('🏢 Performance by Company', fontsize=16, fontweight='bold', pad=20)
    ax.set_xticks(x)
    ax.set_xticklabels(companies, rotation=30, ha='right', fontsize=11, fontweight='medium')
    ax.set_ylim(0, 0.9)
    ax.legend(loc='upper right', framealpha=0.95, fontsize=10)
    
    # Add horizontal reference lines
    ax.axhline(y=0.5, color='#aaa', linestyle='--', alpha=0.5, zorder=1)
    ax.text(len(x)-0.5, 0.51, 'Median', fontsize=9, color='#666')
    
    plt.tight_layout()
    plt.savefig(ANALYSIS_DIR / "company_comparison.png", bbox_inches='tight')
    plt.close()
    
    company_avg.to_csv(ANALYSIS_DIR / "company_performance.csv", float_format="%.4f")
    
    return company_avg


# =============================================================================
# 8. Model Analysis by Release Time (BASE ONLY)
# =============================================================================
def analyze_by_release(df, df_base):
    """Analyze performance by release time (BASE MODELS ONLY)."""
    print("\n" + "=" * 70)
    print("  8. MODEL ANALYSIS BY RELEASE TIME (BASE MODELS ONLY)")
    print("=" * 70)
    
    # Filter out unknown (base only)
    df_known = df_base[df_base['release'] != 'Unknown']
    
    # Release performance
    release_avg = df_known.groupby('release')[ALL_METRICS].mean()
    
    # Sort by time
    time_order = ['2023-Q4', '2024-Q1', '2024-Q2', '2024-Q3', '2024-Q4', '2025-Q1', '2025-Q2']
    release_avg = release_avg.reindex([t for t in time_order if t in release_avg.index])
    
    release_count = df_known.groupby('release')['model'].nunique()
    
    print(f"\n  Performance by Release Time (0-1 range):")
    print("-" * 70)
    
    summary = release_avg[['codebleu', 'rouge1', 'score_reference_document_norm']].round(3)
    summary['n_models'] = release_count
    print(summary.to_string())
    
    # Plot trend - Beautiful version
    fig, ax = plt.subplots(figsize=(12, 6))
    fig.patch.set_facecolor('#fafafa')
    ax.set_facecolor('white')
    
    colors = ['#FF6B6B', '#4ECDC4', '#667eea']
    markers = ['o', 's', '^']
    metrics = ['codebleu', 'rouge1', 'score_reference_document_norm']
    labels = ['CodeBLEU', 'ROUGE-1', 'LLM Judge']
    
    x_numeric = np.arange(len(release_avg))
    
    for i, (metric, label) in enumerate(zip(metrics, labels)):
        line = ax.plot(x_numeric, release_avg[metric], f'{markers[i]}-', 
                       label=label, linewidth=3, markersize=12, color=colors[i],
                       markeredgecolor='white', markeredgewidth=2, zorder=3)
        
        # Add value labels
        for x, y in zip(x_numeric, release_avg[metric]):
            ax.annotate(f'{y:.2f}', (x, y), textcoords="offset points", 
                        xytext=(0, 12), ha='center', fontsize=9, fontweight='bold',
                        color=colors[i])
    
    # Fill area under curves
    ax.fill_between(x_numeric, release_avg['score_reference_document_norm'], alpha=0.1, color=colors[2])
    
    ax.set_xlabel('Release Period', fontsize=12, fontweight='bold')
    ax.set_ylabel('Score (0-1)', fontsize=12, fontweight='bold')
    ax.set_title('📅 Performance Trend by Model Release Time', fontsize=16, fontweight='bold', pad=20)
    ax.legend(loc='lower right', framealpha=0.95, fontsize=11)
    ax.set_ylim(0.25, 0.8)
    
    ax.set_xticks(x_numeric)
    ax.set_xticklabels(release_avg.index, rotation=30, ha='right', fontsize=11)
    
    # Add trend annotation
    first_score = release_avg['score_reference_document_norm'].iloc[0]
    last_score = release_avg['score_reference_document_norm'].iloc[-1]
    trend = (last_score - first_score) / first_score * 100
    trend_color = '#27ae60' if trend > 0 else '#e74c3c'
    ax.annotate(f'Trend: {trend:+.1f}%', xy=(0.95, 0.95), xycoords='axes fraction',
                fontsize=12, fontweight='bold', color=trend_color,
                ha='right', va='top',
                bbox=dict(boxstyle='round,pad=0.3', facecolor='white', edgecolor=trend_color, alpha=0.9))
    
    plt.tight_layout()
    plt.savefig(ANALYSIS_DIR / "release_time_trend.png", bbox_inches='tight')
    plt.close()
    
    release_avg.to_csv(ANALYSIS_DIR / "release_time_performance.csv", float_format="%.4f")
    
    return release_avg


# =============================================================================
# 9. Model Analysis by Size (BASE ONLY)
# =============================================================================
def analyze_by_size(df, df_base):
    """Analyze performance by model size (BASE MODELS ONLY)."""
    print("\n" + "=" * 70)
    print("  9. MODEL ANALYSIS BY SIZE (BASE MODELS ONLY)")
    print("=" * 70)
    
    # Filter out unknown (base only)
    df_known = df_base[df_base['size'] != 'Unknown']
    
    # Size performance
    size_avg = df_known.groupby('size')[ALL_METRICS].mean()
    
    # Sort by size
    size_order = ['Tiny (<3B)', 'Small (3-10B)', 'Medium (10-50B)', 'Large (50-100B)', 'XLarge (100B+)', 'Proprietary']
    size_avg = size_avg.reindex([s for s in size_order if s in size_avg.index])
    
    size_count = df_known.groupby('size')['model'].nunique()
    
    print(f"\n  Performance by Model Size (0-1 range):")
    print("-" * 70)
    
    summary = size_avg[['codebleu', 'rouge1', 'score_reference_document_norm']].round(3)
    summary['n_models'] = size_count
    print(summary.to_string())
    
    # Plot - Beautiful size comparison with gradient
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.patch.set_facecolor('#fafafa')
    
    sizes = size_avg.index.tolist()
    x = np.arange(len(sizes))
    
    # Left: Grouped bar chart
    ax1 = axes[0]
    ax1.set_facecolor('white')
    width = 0.25
    
    colors = ['#FF6B6B', '#4ECDC4', '#667eea']
    
    bars1 = ax1.bar(x - width, size_avg['codebleu'], width, label='CodeBLEU', 
                    color=colors[0], edgecolor='white', linewidth=1.5)
    bars2 = ax1.bar(x, size_avg['rouge1'], width, label='ROUGE-1', 
                    color=colors[1], edgecolor='white', linewidth=1.5)
    bars3 = ax1.bar(x + width, size_avg['score_reference_document_norm'], width, 
                    label='LLM Judge', color=colors[2], edgecolor='white', linewidth=1.5)
    
    for bars in [bars1, bars2, bars3]:
        for bar in bars:
            height = bar.get_height()
            ax1.annotate(f'{height:.2f}', xy=(bar.get_x() + bar.get_width()/2, height),
                        xytext=(0, 3), textcoords="offset points", ha='center', 
                        fontsize=8, fontweight='bold')
    
    ax1.set_xlabel('Model Size', fontsize=12, fontweight='bold')
    ax1.set_ylabel('Score (0-1)', fontsize=12, fontweight='bold')
    ax1.set_title('📊 Performance by Size', fontsize=14, fontweight='bold', pad=15)
    ax1.set_xticks(x)
    ax1.set_xticklabels([s.replace('(', '\n(') for s in sizes], fontsize=9)
    ax1.set_ylim(0, 0.9)
    ax1.legend(loc='upper left', framealpha=0.95)
    
    # Right: Scaling curve
    ax2 = axes[1]
    ax2.set_facecolor('white')
    
    # Filter out proprietary for clean scaling curve
    open_sizes = [s for s in sizes if s != 'Proprietary']
    if len(open_sizes) >= 3:
        open_idx = [sizes.index(s) for s in open_sizes]
        
        for i, (metric, label, color, marker) in enumerate(zip(
            ['codebleu', 'rouge1', 'score_reference_document_norm'],
            ['CodeBLEU', 'ROUGE-1', 'LLM Judge'],
            colors, ['o', 's', '^']
        )):
            values = [size_avg.loc[s, metric] for s in open_sizes]
            ax2.plot(range(len(open_sizes)), values, f'{marker}-', 
                     label=label, linewidth=3, markersize=12, color=color,
                     markeredgecolor='white', markeredgewidth=2)
        
        ax2.set_xticks(range(len(open_sizes)))
        ax2.set_xticklabels([s.split('(')[0].strip() for s in open_sizes], fontsize=10)
        ax2.set_xlabel('Model Size (Parameters)', fontsize=12, fontweight='bold')
        ax2.set_ylabel('Score (0-1)', fontsize=12, fontweight='bold')
        ax2.set_title('📈 Scaling Curve (Open Models)', fontsize=14, fontweight='bold', pad=15)
        ax2.legend(loc='lower right', framealpha=0.95)
        ax2.set_ylim(0.2, 0.75)
        
        # Add arrow annotation
        ax2.annotate('', xy=(len(open_sizes)-1.2, 0.45), xytext=(0.2, 0.3),
                    arrowprops=dict(arrowstyle='->', color='#888', lw=2))
        ax2.text(len(open_sizes)/2, 0.25, 'Scaling →', fontsize=11, 
                 color='#666', ha='center', style='italic')
    
    plt.suptitle('SimBench: Model Size Analysis', fontsize=16, fontweight='bold', y=1.02)
    plt.tight_layout()
    plt.savefig(ANALYSIS_DIR / "size_comparison.png", bbox_inches='tight')
    plt.close()
    
    size_avg.to_csv(ANALYSIS_DIR / "size_performance.csv", float_format="%.4f")
    
    return size_avg


# =============================================================================
# 10. Training Type Analysis (Base vs Fine-tuned vs Prompt-Engineered)
# =============================================================================
def analyze_training_type(df):
    """Analyze performance by training type: Base, Fine-tuned, Prompt-Engineered."""
    print("\n" + "=" * 70)
    print("  10. TRAINING TYPE ANALYSIS (All Models)")
    print("=" * 70)
    
    # training_type already added in load_data()
    
    # Count models per type
    type_counts = df.groupby('training_type')['model'].nunique()
    print(f"\n  Model counts by training type:")
    for t, c in type_counts.items():
        print(f"    {t}: {c} models")
    
    # Performance by training type
    type_avg = df.groupby('training_type')[ALL_METRICS].mean()
    type_avg = type_avg.reindex(['Base', 'Prompt-Engineered', 'Fine-tuned'])
    
    print(f"\n  Performance by Training Type (0-1 range):")
    print("-" * 70)
    display_cols = ['codebleu', 'rouge1', 'score_reference_document_norm']
    print(type_avg[display_cols].round(3).to_string())
    
    # Calculate gains
    if 'Base' in type_avg.index:
        base_score = type_avg.loc['Base', 'score_reference_document_norm']
        print(f"\n  Gains over Base models (score_reference_document):")
        print("-" * 70)
        for t in ['Prompt-Engineered', 'Fine-tuned']:
            if t in type_avg.index:
                t_score = type_avg.loc[t, 'score_reference_document_norm']
                gain = t_score - base_score
                gain_pct = gain / base_score * 100
                print(f"    {t}: {gain:+.3f} ({gain_pct:+.1f}%)")
    
    # Paired comparison: Same base model with different treatments
    print(f"\n  Paired Comparisons (Same base model):")
    print("-" * 70)
    
    # Find pairs
    pairs = []
    base_models = df[df['training_type'] == 'Base']['model'].unique()
    
    for base in base_models:
        base_lower = base.lower().replace('-instruct', '').replace('-it', '')
        
        # Find PE version
        pe_candidates = df[df['training_type'] == 'Prompt-Engineered']['model'].unique()
        for pe in pe_candidates:
            pe_base = pe.lower().replace('pe_', '').replace('-instruct', '').replace('-it', '')
            if base_lower == pe_base or base_lower in pe_base or pe_base in base_lower:
                base_score = df[df['model'] == base]['score_reference_document_norm'].mean()
                pe_score = df[df['model'] == pe]['score_reference_document_norm'].mean()
                pairs.append({
                    'base': base, 'variant': pe, 'type': 'PE',
                    'base_score': base_score, 'variant_score': pe_score,
                    'gain': pe_score - base_score
                })
    
    # Sort by gain
    pairs = sorted(pairs, key=lambda x: x['gain'], reverse=True)
    
    if pairs:
        print(f"\n  Top PE gains:")
        for p in pairs[:5]:
            print(f"    {p['base'][:25]:<25s} → PE: {p['gain']:+.3f} ({p['base_score']:.3f} → {p['variant_score']:.3f})")
    
    # Fine-tuned comparisons
    ft_pairs = []
    for model in df['model'].unique():
        if df[df['model'] == model]['training_type'].iloc[0] == 'Fine-tuned':
            # Try to find base
            model_lower = model.lower()
            for suffix in ['-f1', '-f3', 'lora1', 'sft1', '_f1', '_f3']:
                if suffix in model_lower:
                    base_name = model_lower.replace(suffix, '').replace('llama3.1', 'llama-3.1').replace('llama3.3', 'llama-3.3')
                    for base in base_models:
                        if base.lower().startswith(base_name[:10]):
                            ft_score = df[df['model'] == model]['score_reference_document_norm'].mean()
                            base_score = df[df['model'] == base]['score_reference_document_norm'].mean()
                            ft_pairs.append({
                                'base': base, 'variant': model, 'type': 'FT',
                                'base_score': base_score, 'variant_score': ft_score,
                                'gain': ft_score - base_score
                            })
                    break
    
    ft_pairs = sorted(ft_pairs, key=lambda x: x['gain'], reverse=True)
    
    if ft_pairs:
        print(f"\n  Top Fine-tuning gains:")
        for p in ft_pairs[:5]:
            print(f"    {p['base'][:25]:<25s} → FT: {p['gain']:+.3f} ({p['base_score']:.3f} → {p['variant_score']:.3f})")
    
    # Plot
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.patch.set_facecolor('#fafafa')
    
    # Left: Bar chart by training type
    ax1 = axes[0]
    ax1.set_facecolor('white')
    
    types = type_avg.index.tolist()
    x = np.arange(len(types))
    width = 0.25
    colors = ['#FF6B6B', '#4ECDC4', '#667eea']
    
    bars1 = ax1.bar(x - width, type_avg['codebleu'], width, label='CodeBLEU', 
                    color=colors[0], edgecolor='white', linewidth=1.5)
    bars2 = ax1.bar(x, type_avg['rouge1'], width, label='ROUGE-1', 
                    color=colors[1], edgecolor='white', linewidth=1.5)
    bars3 = ax1.bar(x + width, type_avg['score_reference_document_norm'], width, 
                    label='LLM Judge', color=colors[2], edgecolor='white', linewidth=1.5)
    
    for bars in [bars1, bars2, bars3]:
        for bar in bars:
            height = bar.get_height()
            ax1.annotate(f'{height:.2f}', xy=(bar.get_x() + bar.get_width()/2, height),
                        xytext=(0, 3), textcoords="offset points", ha='center', 
                        fontsize=9, fontweight='bold')
    
    ax1.set_xlabel('Training Type', fontsize=12, fontweight='bold')
    ax1.set_ylabel('Score (0-1)', fontsize=12, fontweight='bold')
    ax1.set_title('🎓 Performance by Training Type', fontsize=14, fontweight='bold', pad=15)
    ax1.set_xticks(x)
    ax1.set_xticklabels(types, fontsize=11)
    ax1.set_ylim(0, 0.85)
    ax1.legend(loc='upper left', framealpha=0.95)
    
    # Add model counts
    for i, t in enumerate(types):
        count = type_counts.get(t, 0)
        ax1.annotate(f'n={count}', xy=(x[i], 0.02), ha='center', fontsize=9, color='#666')
    
    # Right: Gain visualization
    ax2 = axes[1]
    ax2.set_facecolor('white')
    
    if 'Base' in type_avg.index:
        base_val = type_avg.loc['Base', 'score_reference_document_norm']
        
        labels = []
        gains = []
        colors_gain = []
        
        for t in ['Prompt-Engineered', 'Fine-tuned']:
            if t in type_avg.index:
                gain = type_avg.loc[t, 'score_reference_document_norm'] - base_val
                labels.append(t.replace('-', '\n'))
                gains.append(gain)
                colors_gain.append('#27ae60' if gain > 0 else '#e74c3c')
        
        y_pos = np.arange(len(labels))
        bars = ax2.barh(y_pos, gains, color=colors_gain, edgecolor='white', linewidth=1.5, height=0.5)
        
        for bar, val in zip(bars, gains):
            x_pos = val + 0.005 if val > 0 else val - 0.005
            ha = 'left' if val > 0 else 'right'
            ax2.text(x_pos, bar.get_y() + bar.get_height()/2, f'{val:+.3f}',
                    va='center', ha=ha, fontsize=12, fontweight='bold')
        
        ax2.set_yticks(y_pos)
        ax2.set_yticklabels(labels, fontsize=11)
        ax2.set_xlabel('Gain over Base Models', fontsize=12, fontweight='bold')
        ax2.set_title('📈 Performance Gain', fontsize=14, fontweight='bold', pad=15)
        ax2.axvline(x=0, color='#333', linewidth=1)
        ax2.set_xlim(-0.05, max(gains) * 1.5 if gains else 0.1)
    
    plt.suptitle('SimBench: Training Type Analysis', fontsize=16, fontweight='bold', y=1.02)
    plt.tight_layout()
    plt.savefig(ANALYSIS_DIR / "training_type_comparison.png", bbox_inches='tight')
    plt.close()
    
    type_avg.to_csv(ANALYSIS_DIR / "training_type_performance.csv", float_format="%.4f")
    
    return type_avg


# =============================================================================
# 11. Generate Summary Report
# =============================================================================
def generate_summary_report(df, model_agg, corr_pearson, corr_spearman):
    """Generate a summary report."""
    print("\n" + "=" * 70)
    print("  11. SUMMARY REPORT")
    print("=" * 70)
    
    # Key findings
    top_model = model_agg['score_reference_document_norm'].idxmax()
    top_score = model_agg.loc[top_model, 'score_reference_document_norm']
    
    # Highest correlation
    max_pearson = corr_pearson.loc[CODE_SIM_METRICS, LLM_JUDGE_NORM].max().max()
    max_spearman = corr_spearman.loc[CODE_SIM_METRICS, LLM_JUDGE_NORM].max().max()
    
    report = f"""
╔══════════════════════════════════════════════════════════════════════╗
║                    SimBench Analysis Summary                         ║
╠══════════════════════════════════════════════════════════════════════╣
║  Dataset:     {len(df):,} evaluations, {df['model'].nunique()} models, {df['system'].nunique()} systems          ║
║                                                                      ║
║  Top Model:   {top_model:<40s}           ║
║              (Score: {top_score:.3f})                                        ║
║                                                                      ║
║  Correlation (Code Sim ↔ LLM Judge):                                 ║
║    - Pearson (max):  {max_pearson:.3f}                                       ║
║    - Spearman (max): {max_spearman:.3f}                                       ║
║                                                                      ║
║  Round Improvement (R1→R3):                                          ║
║    - CodeBLEU:       +{(df[df['round_num']==3]['codebleu'].mean() - df[df['round_num']==1]['codebleu'].mean()):.3f}                                       ║
║    - Score Ref Doc:  +{(df[df['round_num']==3]['score_reference_document_norm'].mean() - df[df['round_num']==1]['score_reference_document_norm'].mean()):.3f}                                       ║
╚══════════════════════════════════════════════════════════════════════╝
"""
    print(report)
    
    with open(ANALYSIS_DIR / "summary_report.txt", 'w', encoding='utf-8') as f:
        f.write(report)


# =============================================================================
# MAIN
# =============================================================================
def main():
    print("\n" + "=" * 70)
    print("  SimBench Comprehensive Data Analysis")
    print("=" * 70)
    
    # Create output directory
    ANALYSIS_DIR.mkdir(parents=True, exist_ok=True)
    
    # Check input
    if not DATA_FILE.exists():
        print(f"[ERROR] Data file not found: {DATA_FILE}")
        print("Please run rank_llm.py first to generate all_metrics_merged.csv")
        return
    
    # Load data
    df = load_data()
    df_base = filter_base_models(df)
    
    print(f"\n  Loaded {len(df):,} rows from {DATA_FILE.name}")
    print(f"  Total models: {df['model'].nunique()}")
    print(f"  Base models: {df_base['model'].nunique()} (used for main analysis)")
    print(f"  Fine-tuned: {df[df['training_type']=='Fine-tuned']['model'].nunique()}")
    print(f"  Prompt-Engineered: {df[df['training_type']=='Prompt-Engineered']['model'].nunique()}")
    
    # Run all analyses (most use BASE models only)
    stats_df = analyze_overall_stats(df)  # All data for stats
    model_agg, rankings = analyze_model_rankings(df, df_base)  # Base only
    round_avg = analyze_round_improvement(df)  # All data
    cat_avg = analyze_categories(df)  # All data
    corr_p, corr_s = analyze_correlations(df, df_base)  # Base only
    analyze_statistical_tests(df)  # All data
    analyze_by_company(df, df_base)  # Base only
    analyze_by_release(df, df_base)  # Base only
    analyze_by_size(df, df_base)  # Base only
    analyze_training_type(df)  # All data (comparison)
    generate_summary_report(df_base, model_agg, corr_p, corr_s)  # Base only
    
    # List outputs
    print("\n" + "=" * 70)
    print("  OUTPUT FILES")
    print("=" * 70)
    for f in sorted(ANALYSIS_DIR.glob("*")):
        print(f"  - {f.name}")
    
    print("\n  Analysis complete! ✓\n")


if __name__ == "__main__":
    main()
