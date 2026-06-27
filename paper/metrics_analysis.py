#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Comprehensive Metrics Analysis for SimBench
Beyond simple correlation - deeper insights into metric relationships
"""

import sys
import io
import pandas as pd
import numpy as np
from scipy import stats
from scipy.cluster import hierarchy
from sklearn.preprocessing import StandardScaler
from sklearn.decomposition import PCA
from sklearn.metrics import mutual_info_score
import matplotlib.pyplot as plt
import seaborn as sns

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

# Setup plotting
plt.style.use('seaborn-v0_8-whitegrid')
plt.rcParams['figure.dpi'] = 150
plt.rcParams['savefig.dpi'] = 200

# Paths
DATA_FILE = 'paper/out/all_metrics_merged.csv'
OUT_DIR = 'paper/out/analysis'

# Load data
df = pd.read_csv(DATA_FILE)

# Normalize LLM scores to 0-1
df['score_document_norm'] = df['score_document'] / 100
df['score_reference_norm'] = df['score_reference'] / 100
df['score_reference_document_norm'] = df['score_reference_document'] / 100

# Classify by training type
def get_training_type(model_name):
    name_lower = model_name.lower()
    if name_lower.startswith('pe_'):
        return 'PE'
    elif any(x in name_lower for x in ['-f1', '-f3', 'lora', 'sft']):
        return 'FT'
    return 'Base'

df['training_type'] = df['model'].apply(get_training_type)

# Filter to BASE MODELS ONLY
df_base = df[df['training_type'] == 'Base'].copy()

# All metrics (0-1 range)
CODE_METRICS = ['codebleu', 'ngram_match_score', 'weighted_ngram_match_score',
                'syntax_match_score', 'dataflow_match_score',
                'rouge1', 'rouge2', 'rougeL', 'rougeLsum']
LLM_METRICS = ['score_document_norm', 'score_reference_norm', 'score_reference_document_norm']
ALL_METRICS = CODE_METRICS + LLM_METRICS

print("=" * 80)
print("  COMPREHENSIVE METRICS ANALYSIS (BASE MODELS ONLY)")
print("=" * 80)
print(f"\n  Total models: {df['model'].nunique()}")
print(f"  Base models: {df_base['model'].nunique()} (used for analysis)")
print(f"  Excluded: {df['model'].nunique() - df_base['model'].nunique()} (FT + PE)")

# ============================================================
# 1. CORRELATION ANALYSIS (Extended)
# ============================================================
print("\n" + "=" * 80)
print("  1. CORRELATION ANALYSIS (Pearson, Spearman, Kendall)")
print("=" * 80)

# Aggregate per model (BASE ONLY)
model_scores = df_base.groupby('model')[ALL_METRICS].mean()

corr_pearson = model_scores.corr(method='pearson')
corr_spearman = model_scores.corr(method='spearman')
corr_kendall = model_scores.corr(method='kendall')

print(f"\n  A. Average Correlation Strength (Code Sim ↔ LLM Judge):")
code_llm_corrs = []
for code_m in CODE_METRICS:
    for llm_m in LLM_METRICS:
        code_llm_corrs.append({
            'code': code_m, 'llm': llm_m,
            'pearson': corr_pearson.loc[code_m, llm_m],
            'spearman': corr_spearman.loc[code_m, llm_m],
            'kendall': corr_kendall.loc[code_m, llm_m]
        })

corr_df = pd.DataFrame(code_llm_corrs)
print(f"\n     Average Pearson:  {corr_df['pearson'].mean():.3f} ± {corr_df['pearson'].std():.3f}")
print(f"     Average Spearman: {corr_df['spearman'].mean():.3f} ± {corr_df['spearman'].std():.3f}")
print(f"     Average Kendall:  {corr_df['kendall'].mean():.3f} ± {corr_df['kendall'].std():.3f}")

# ============================================================
# 2. PRINCIPAL COMPONENT ANALYSIS (PCA)
# ============================================================
print("\n" + "=" * 80)
print("  2. PRINCIPAL COMPONENT ANALYSIS")
print("=" * 80)

scaler = StandardScaler()
scaled_data = scaler.fit_transform(model_scores)

pca = PCA()
pca_result = pca.fit_transform(scaled_data)

print(f"\n  A. Explained Variance Ratio:")
for i, var in enumerate(pca.explained_variance_ratio_[:5]):
    cum_var = pca.explained_variance_ratio_[:i+1].sum()
    print(f"     PC{i+1}: {var*100:.1f}% (cumulative: {cum_var*100:.1f}%)")

print(f"\n  B. Key Finding:")
if pca.explained_variance_ratio_[0] > 0.5:
    print(f"     → PC1 explains {pca.explained_variance_ratio_[0]*100:.1f}% of variance")
    print(f"     → Metrics are highly correlated, ONE underlying factor dominates")
else:
    print(f"     → No single PC dominates, metrics capture diverse aspects")

# PC1 loadings
pc1_loadings = pd.Series(pca.components_[0], index=ALL_METRICS).sort_values(key=abs, ascending=False)
print(f"\n  C. PC1 Loadings (dominant factor):")
for metric, loading in pc1_loadings.head(6).items():
    print(f"     {metric:35s}: {loading:+.3f}")

# ============================================================
# 3. METRIC REDUNDANCY ANALYSIS
# ============================================================
print("\n" + "=" * 80)
print("  3. METRIC REDUNDANCY ANALYSIS")
print("=" * 80)

print("\n  A. Highly Correlated Metric Pairs (r > 0.9, potentially redundant):")
redundant_pairs = []
for i, m1 in enumerate(ALL_METRICS):
    for m2 in ALL_METRICS[i+1:]:
        r = corr_pearson.loc[m1, m2]
        if abs(r) > 0.9:
            redundant_pairs.append((m1, m2, r))
            print(f"     {m1} ↔ {m2}: r={r:.3f}")

print(f"\n  B. Unique Information Metrics (low avg correlation with others):")
avg_corr = corr_pearson.abs().mean()
for metric in avg_corr.nsmallest(5).index:
    print(f"     {metric:35s}: avg|r|={avg_corr[metric]:.3f}")

# ============================================================
# 4. RANKING AGREEMENT ANALYSIS
# ============================================================
print("\n" + "=" * 80)
print("  4. RANKING AGREEMENT ANALYSIS")
print("=" * 80)

# Rank models by each metric
ranks = model_scores.rank(ascending=False)

# Kendall's W (coefficient of concordance)
n_models = len(model_scores)
n_metrics = len(ALL_METRICS)

# Calculate mean rank and S
mean_ranks = ranks.mean(axis=1)
S = ((mean_ranks - mean_ranks.mean()) ** 2).sum()
W = 12 * S / (n_metrics**2 * (n_models**3 - n_models))

print(f"\n  A. Kendall's W (Overall Ranking Agreement): {W:.3f}")
if W > 0.7:
    print("     → Strong agreement: metrics produce similar rankings")
elif W > 0.5:
    print("     → Moderate agreement: some consistency in rankings")
else:
    print("     → Weak agreement: metrics rank models differently")

# Pairwise ranking agreement
print(f"\n  B. Pairwise Rank Correlation (Kendall's Tau):")
print(f"     {'Metric 1':<25s} {'Metric 2':<25s} {'Tau':>8s}")
print("     " + "-" * 60)

# Key comparisons
key_pairs = [
    ('codebleu', 'score_reference_document_norm'),
    ('rouge1', 'score_reference_document_norm'),
    ('codebleu', 'rouge1'),
    ('score_document_norm', 'score_reference_document_norm'),
]

for m1, m2 in key_pairs:
    tau, p = stats.kendalltau(ranks[m1], ranks[m2])
    sig = '***' if p < 0.001 else '**' if p < 0.01 else '*' if p < 0.05 else ''
    print(f"     {m1:<25s} {m2:<25s} {tau:+.3f} {sig}")

# ============================================================
# 5. METRIC DISCRIMINATION POWER
# ============================================================
print("\n" + "=" * 80)
print("  5. METRIC DISCRIMINATION POWER")
print("=" * 80)

print("\n  A. Coefficient of Variation (higher = more discriminative):")
cv = model_scores.std() / model_scores.mean()
cv_sorted = cv.sort_values(ascending=False)
for metric in cv_sorted.index:
    print(f"     {metric:35s}: CV={cv_sorted[metric]:.3f}")

print(f"\n  B. Dynamic Range (max - min):")
dynamic_range = model_scores.max() - model_scores.min()
for metric in dynamic_range.sort_values(ascending=False).index:
    print(f"     {metric:35s}: range={dynamic_range[metric]:.3f}")

# ============================================================
# 6. METRIC CLUSTERING (Hierarchical)
# ============================================================
print("\n" + "=" * 80)
print("  6. METRIC CLUSTERING")
print("=" * 80)

# Distance matrix (1 - correlation)
dist_matrix = 1 - corr_spearman.abs()

# Hierarchical clustering
linkage = hierarchy.linkage(hierarchy.distance.squareform(dist_matrix.values), method='average')

# Get clusters at a threshold
clusters = hierarchy.fcluster(linkage, t=0.3, criterion='distance')
cluster_df = pd.DataFrame({'metric': ALL_METRICS, 'cluster': clusters})

print("\n  A. Metric Clusters (based on correlation distance):")
for cluster_id in sorted(cluster_df['cluster'].unique()):
    metrics_in_cluster = cluster_df[cluster_df['cluster'] == cluster_id]['metric'].tolist()
    print(f"     Cluster {cluster_id}: {metrics_in_cluster}")

# ============================================================
# 7. REGRESSION ANALYSIS: Predict LLM from Code Metrics
# ============================================================
print("\n" + "=" * 80)
print("  7. REGRESSION ANALYSIS")
print("=" * 80)

from sklearn.linear_model import LinearRegression
from sklearn.model_selection import cross_val_score

print("\n  A. Can we predict LLM-Judge from Code Metrics?")

X = model_scores[CODE_METRICS]
for target in LLM_METRICS:
    y = model_scores[target]
    
    model = LinearRegression()
    cv_scores = cross_val_score(model, X, y, cv=5, scoring='r2')
    
    model.fit(X, y)
    
    print(f"\n     Target: {target}")
    print(f"     R² (5-fold CV): {cv_scores.mean():.3f} ± {cv_scores.std():.3f}")
    
    # Top 3 predictors
    coefs = pd.Series(model.coef_, index=CODE_METRICS).abs().sort_values(ascending=False)
    print(f"     Top predictors: {', '.join(coefs.head(3).index)}")

# ============================================================
# 8. PARTIAL CORRELATION
# ============================================================
print("\n" + "=" * 80)
print("  8. PARTIAL CORRELATION ANALYSIS")
print("=" * 80)

print("\n  A. Partial Correlation: CodeBLEU ↔ LLM-Judge (controlling for ROUGE1)")
def partial_corr(x, y, z):
    """Partial correlation between x and y, controlling for z"""
    from scipy.stats import pearsonr
    
    # Residuals of x ~ z
    r_xz = pearsonr(x, z)[0]
    x_resid = x - r_xz * (x.std() / z.std()) * z
    
    # Residuals of y ~ z
    r_yz = pearsonr(y, z)[0]
    y_resid = y - r_yz * (y.std() / z.std()) * z
    
    return pearsonr(x_resid, y_resid)[0]

x = model_scores['codebleu']
y = model_scores['score_reference_document_norm']
z = model_scores['rouge1']

r_xy = stats.pearsonr(x, y)[0]
r_xy_z = partial_corr(x, y, z)

print(f"     CodeBLEU ↔ LLM-Judge (raw):           r = {r_xy:.3f}")
print(f"     CodeBLEU ↔ LLM-Judge (control ROUGE): r = {r_xy_z:.3f}")
print(f"     Difference: {r_xy_z - r_xy:+.3f}")

if abs(r_xy_z) < abs(r_xy) * 0.5:
    print("     → ROUGE explains most of the CodeBLEU-LLM relationship")
else:
    print("     → CodeBLEU has unique predictive value beyond ROUGE")

# ============================================================
# 9. EFFECT SIZE ANALYSIS
# ============================================================
print("\n" + "=" * 80)
print("  9. EFFECT SIZE: Top vs Bottom Models")
print("=" * 80)

# Split models into top/bottom by LLM judge
llm_score = model_scores['score_reference_document_norm']
top_models = llm_score[llm_score > llm_score.median()].index
bottom_models = llm_score[llm_score <= llm_score.median()].index

print("\n  A. Cohen's d (Top vs Bottom models on each metric):")
for metric in ALL_METRICS:
    top_vals = model_scores.loc[top_models, metric]
    bottom_vals = model_scores.loc[bottom_models, metric]
    
    pooled_std = np.sqrt((top_vals.std()**2 + bottom_vals.std()**2) / 2)
    cohens_d = (top_vals.mean() - bottom_vals.mean()) / pooled_std
    
    effect = 'large' if abs(cohens_d) > 0.8 else 'medium' if abs(cohens_d) > 0.5 else 'small'
    print(f"     {metric:35s}: d={cohens_d:+.2f} ({effect})")

# ============================================================
# 10. SUMMARY TABLE
# ============================================================
print("\n" + "=" * 80)
print("  10. METRICS SUMMARY TABLE")
print("=" * 80)

summary = pd.DataFrame({
    'Mean': model_scores.mean(),
    'Std': model_scores.std(),
    'CV': cv,
    'Range': dynamic_range,
    'PC1_Loading': pca.components_[0],
    'Avg_Corr': avg_corr
})

print("\n" + summary.round(3).to_string())

# Save summary
summary.to_csv(f'{OUT_DIR}/metrics_summary.csv', float_format='%.4f')

# ============================================================
# PLOTS
# ============================================================

# Plot 1: PCA Biplot
fig, axes = plt.subplots(1, 2, figsize=(14, 6))

# PC1 vs PC2 for models
ax1 = axes[0]
ax1.scatter(pca_result[:, 0], pca_result[:, 1], alpha=0.7, s=100, c='#667eea')
for i, model in enumerate(model_scores.index):
    if i % 5 == 0:  # Label every 5th model
        ax1.annotate(model[:15], (pca_result[i, 0], pca_result[i, 1]), fontsize=7)
ax1.set_xlabel(f'PC1 ({pca.explained_variance_ratio_[0]*100:.1f}%)')
ax1.set_ylabel(f'PC2 ({pca.explained_variance_ratio_[1]*100:.1f}%)')
ax1.set_title('Model Positions in PCA Space')
ax1.axhline(0, color='gray', linestyle='--', alpha=0.3)
ax1.axvline(0, color='gray', linestyle='--', alpha=0.3)

# Loadings plot
ax2 = axes[1]
for i, metric in enumerate(ALL_METRICS):
    ax2.arrow(0, 0, pca.components_[0, i], pca.components_[1, i], 
              head_width=0.05, head_length=0.02, fc='#FF6B6B', ec='#FF6B6B')
    ax2.annotate(metric, (pca.components_[0, i]*1.1, pca.components_[1, i]*1.1), fontsize=8)
ax2.set_xlabel('PC1 Loading')
ax2.set_ylabel('PC2 Loading')
ax2.set_title('Metric Loadings (Direction = Contribution)')
ax2.set_xlim(-1, 1)
ax2.set_ylim(-1, 1)
ax2.axhline(0, color='gray', linestyle='--', alpha=0.3)
ax2.axvline(0, color='gray', linestyle='--', alpha=0.3)

plt.tight_layout()
plt.savefig(f'{OUT_DIR}/pca_analysis.png', bbox_inches='tight')
plt.close()

# Plot 2: Metric clustering dendrogram
fig, ax = plt.subplots(figsize=(12, 6))
hierarchy.dendrogram(linkage, labels=ALL_METRICS, ax=ax, leaf_rotation=45)
ax.set_title('Metric Clustering Dendrogram (based on correlation)')
ax.set_ylabel('Distance (1 - |correlation|)')
plt.tight_layout()
plt.savefig(f'{OUT_DIR}/metric_clustering.png', bbox_inches='tight')
plt.close()

print("\n" + "=" * 80)
print("  ANALYSIS COMPLETE")
print("  Saved: metrics_summary.csv, pca_analysis.png, metric_clustering.png")
print("=" * 80)
