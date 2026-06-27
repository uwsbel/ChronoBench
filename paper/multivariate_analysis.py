#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SimBench Multivariate Analysis
================================
Comprehensive multivariate analysis covering:
1. Descriptive Statistics
2. Three-way ANOVA (Model × Category × Round)
3. Metric Correlation Analysis
4. PCA on metric space
5. Interaction Effects (heatmaps)
6. Linear Regression (predicting J-LLM from CodeBLEU/ROUGE)
7. Post-hoc pairwise comparisons
8. (figures)
9. Summary
── Extended factors ──
10. Open-source vs Closed-source comparison
11. Model size effect (open-source only)
12. Release date effect
13. Multi-factor regression (source type + size + date)
14. Extended figures

Primary outcome: score_reference_document (J-LLM_ref_doc, 0-100)
"""

import sys, io, os
import warnings
warnings.filterwarnings('ignore')
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

import pandas as pd
import numpy as np
from scipy import stats
from scipy.stats import f_oneway, pearsonr, spearmanr
from sklearn.preprocessing import StandardScaler
from sklearn.decomposition import PCA
from sklearn.linear_model import LinearRegression, Ridge
from sklearn.metrics import r2_score
import statsmodels.api as sm
import statsmodels.formula.api as smf
from statsmodels.stats.multicomp import pairwise_tukeyhsd
from statsmodels.stats.anova import anova_lm
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import seaborn as sns
from pathlib import Path

# ── Paths ──────────────────────────────────────────────────────────────────
SCRIPT_DIR = Path(__file__).resolve().parent
DATA_FILE  = SCRIPT_DIR / 'out' / 'all_metrics_merged_pretrain_only.csv'
OUT_DIR    = SCRIPT_DIR / 'out' / 'multivariate'
OUT_DIR.mkdir(parents=True, exist_ok=True)

# ── Style ──────────────────────────────────────────────────────────────────
plt.style.use('seaborn-v0_8-whitegrid')
plt.rcParams.update({
    'figure.dpi': 150, 'savefig.dpi': 200,
    'font.size': 10, 'axes.titlesize': 12, 'axes.labelsize': 11,
    'xtick.labelsize': 9, 'ytick.labelsize': 9,
    'legend.fontsize': 9, 'figure.facecolor': 'white',
})
CB_PALETTE = ['#1f77b4','#ff7f0e','#2ca02c','#d62728','#9467bd']

# ── Category map ───────────────────────────────────────────────────────────
MBS = ['pendulum','slider_crank','gear','mass_spring_damper','particles']
FEA = ['beam','buckling','rotor','tablecloth','cable']
SEN = ['gps_imu','lidar','veh_app','camera']
RBT = ['turtlebot','viper','curiosity','vehros','sensros','handler']
VEH = ['citybus','feda','gator','hmmwv','kraz','art',
       'rigid_highway','rigid_multipatches','scm','scm_hill',
       'uazbus','m113','sedan','man']
CAT_MAP = {}
for s in MBS: CAT_MAP[s] = 'MBS'
for s in FEA: CAT_MAP[s] = 'FEA'
for s in SEN: CAT_MAP[s] = 'SEN'
for s in RBT: CAT_MAP[s] = 'RBT'
for s in VEH: CAT_MAP[s] = 'VEH'
CAT_ORDER = ['SEN','RBT','VEH','MBS','FEA']

# ── Metrics ────────────────────────────────────────────────────────────────
CODE_METRICS = ['codebleu','ngram_match_score','weighted_ngram_match_score',
                'syntax_match_score','dataflow_match_score',
                'rouge1','rouge2','rougeL','rougeLsum']
LLM_METRICS  = ['score_document','score_reference','score_reference_document']
TARGET       = 'score_reference_document'   # primary outcome

# ── Load data ──────────────────────────────────────────────────────────────
print("="*70)
print("SIMBENCH MULTIVARIATE ANALYSIS")
print("="*70)

df = pd.read_csv(DATA_FILE)
df['category'] = df['system'].map(CAT_MAP)
df['round_num'] = df['round'].str.extract(r'(\d)').astype(int)
df['round_label'] = 'T' + df['round_num'].astype(str)

# Normalize CodeBLEU/ROUGE to 0-100 to match J-LLM scale
for m in CODE_METRICS:
    df[m+'_100'] = df[m] * 100

print(f"\nDataset: {df.shape[0]} rows × {df.shape[1]} cols")
print(f"  Models   : {df['model'].nunique()} unique")
print(f"  Systems  : {df['system'].nunique()} unique")
print(f"  Rounds   : {sorted(df['round_label'].unique())}")
print(f"  Categories: {df['category'].unique().tolist()}")

# ─────────────────────────────────────────────────────────────────────────────
# 1. DESCRIPTIVE STATISTICS
# ─────────────────────────────────────────────────────────────────────────────
print("\n" + "="*70)
print("1. DESCRIPTIVE STATISTICS")
print("="*70)

desc_overall = df[TARGET].describe()
print(f"\nOverall {TARGET} (0-100):")
print(desc_overall.round(2))

# By category
desc_cat = df.groupby('category')[TARGET].agg(['mean','std','min','max','count'])
desc_cat = desc_cat.reindex(CAT_ORDER)
desc_cat.columns = ['Mean','Std','Min','Max','N']
print(f"\nBy Category:\n{desc_cat.round(2)}")

# By round
desc_round = df.groupby('round_label')[TARGET].agg(['mean','std','min','max'])
print(f"\nBy Round:\n{desc_round.round(2)}")

# By model (top/bottom 5)
desc_model = df.groupby('model')[TARGET].mean().sort_values(ascending=False)
print(f"\nTop-5 Models:\n{desc_model.head(5).round(2)}")
print(f"\nBottom-5 Models:\n{desc_model.tail(5).round(2)}")

# Save descriptive stats
desc_cat.to_csv(OUT_DIR / 'desc_by_category.csv')
desc_round.to_csv(OUT_DIR / 'desc_by_round.csv')
desc_model.to_csv(OUT_DIR / 'desc_by_model.csv')

# ─────────────────────────────────────────────────────────────────────────────
# 2. THREE-WAY ANOVA  (Category + Round + their interaction)
# ─────────────────────────────────────────────────────────────────────────────
print("\n" + "="*70)
print("2. THREE-WAY ANOVA  (Category × Round, with Model as covariate)")
print("="*70)

# OLS-based ANOVA using Type-III SS
formula = f'{TARGET} ~ C(category) + C(round_label) + C(category):C(round_label)'
ols_model = smf.ols(formula, data=df).fit()
anova_table = anova_lm(ols_model, typ=3)
print("\nANOVA Table (Type III SS):")
print(anova_table.round(4))
anova_table.to_csv(OUT_DIR / 'anova_table.csv')

# Effect sizes (η²)
ss_total = anova_table['sum_sq'].sum()
anova_table['eta_sq'] = anova_table['sum_sq'] / ss_total
print(f"\nEffect sizes (η²):")
for idx, row in anova_table.iterrows():
    if idx != 'Residual':
        print(f"  {idx:45s}  η²={row['eta_sq']:.4f}  p={row['PR(>F)']:.4e}")

# One-way ANOVA per factor (simple)
groups_cat   = [df[df['category']==c][TARGET].values for c in CAT_ORDER]
groups_round = [df[df['round_label']==r][TARGET].values for r in ['T1','T2','T3']]
F_cat,  p_cat  = f_oneway(*groups_cat)
F_rnd,  p_rnd  = f_oneway(*groups_round)
print(f"\nOne-way ANOVA — Category:  F={F_cat:.2f}, p={p_cat:.4e}")
print(f"One-way ANOVA — Round:     F={F_rnd:.2f}, p={p_rnd:.4e}")

# ─────────────────────────────────────────────────────────────────────────────
# 3. POST-HOC: TUKEY HSD
# ─────────────────────────────────────────────────────────────────────────────
print("\n" + "="*70)
print("3. POST-HOC COMPARISONS (Tukey HSD)")
print("="*70)

# Category post-hoc
tukey_cat = pairwise_tukeyhsd(df[TARGET], df['category'], alpha=0.05)
print("\nCategory Tukey HSD:")
print(tukey_cat.summary())
tukey_df_cat = pd.DataFrame(data=tukey_cat._results_table.data[1:],
                             columns=tukey_cat._results_table.data[0])
tukey_df_cat.to_csv(OUT_DIR / 'tukey_category.csv', index=False)

# Round post-hoc
tukey_rnd = pairwise_tukeyhsd(df[TARGET], df['round_label'], alpha=0.05)
print("\nRound Tukey HSD:")
print(tukey_rnd.summary())
tukey_df_rnd = pd.DataFrame(data=tukey_rnd._results_table.data[1:],
                              columns=tukey_rnd._results_table.data[0])
tukey_df_rnd.to_csv(OUT_DIR / 'tukey_round.csv', index=False)

# ─────────────────────────────────────────────────────────────────────────────
# 4. METRIC CORRELATION ANALYSIS
# ─────────────────────────────────────────────────────────────────────────────
print("\n" + "="*70)
print("4. METRIC CORRELATION ANALYSIS")
print("="*70)

metric_cols = CODE_METRICS + LLM_METRICS
corr_pearson  = df[metric_cols].corr(method='pearson')
corr_spearman = df[metric_cols].corr(method='spearman')

print("\nPearson r — J-LLM_ref_doc vs other metrics:")
for m in metric_cols:
    if m != TARGET:
        r, p = pearsonr(df[m].fillna(0), df[TARGET])
        rs, ps = spearmanr(df[m].fillna(0), df[TARGET])
        sig = '***' if p < 0.001 else ('**' if p < 0.01 else ('*' if p < 0.05 else ''))
        print(f"  {m:45s}  Pearson r={r:+.3f}{sig}  Spearman ρ={rs:+.3f}")

corr_pearson.to_csv(OUT_DIR / 'corr_pearson.csv')
corr_spearman.to_csv(OUT_DIR / 'corr_spearman.csv')

# ─────────────────────────────────────────────────────────────────────────────
# 5. PCA ON METRIC SPACE
# ─────────────────────────────────────────────────────────────────────────────
print("\n" + "="*70)
print("5. PCA ON METRIC SPACE")
print("="*70)

df_pca = df[metric_cols].fillna(0)
scaler = StandardScaler()
X_scaled = scaler.fit_transform(df_pca)

pca = PCA()
pca.fit(X_scaled)
exp_var = pca.explained_variance_ratio_
cum_var = np.cumsum(exp_var)

print("\nPCA Explained Variance:")
for i, (ev, cv) in enumerate(zip(exp_var, cum_var)):
    print(f"  PC{i+1}: {ev*100:.2f}%  (cumulative: {cv*100:.2f}%)")
    if cv > 0.95:
        break

# Loadings for PC1 and PC2
loadings = pd.DataFrame(
    pca.components_[:4].T,
    index=metric_cols,
    columns=[f'PC{i+1}' for i in range(4)]
)
print(f"\nPCA Loadings (PC1-PC4):\n{loadings.round(3)}")
loadings.to_csv(OUT_DIR / 'pca_loadings.csv')

# PCA scores per observation
X_pca = pca.transform(X_scaled)
df['PC1'] = X_pca[:,0]
df['PC2'] = X_pca[:,1]

# ─────────────────────────────────────────────────────────────────────────────
# 6. LINEAR REGRESSION: Predicting J-LLM from CodeBLEU/ROUGE
# ─────────────────────────────────────────────────────────────────────────────
print("\n" + "="*70)
print("6. LINEAR REGRESSION  (J-LLM_ref_doc ~ CodeBLEU + ROUGE)")
print("="*70)

# Predictors: all code metrics (not LLM metrics)
X_reg = df[CODE_METRICS].fillna(0).values
y_reg = df[TARGET].values

# OLS
X_ols = sm.add_constant(X_reg)
ols_reg = sm.OLS(y_reg, X_ols).fit()
print(f"\nOLS Regression  R²={ols_reg.rsquared:.4f}  adj-R²={ols_reg.rsquared_adj:.4f}")
print(f"  F-statistic: {ols_reg.fvalue:.2f}, p={ols_reg.f_pvalue:.4e}")
coef_df = pd.DataFrame({
    'coef': ols_reg.params[1:],
    'std_err': ols_reg.bse[1:],
    't': ols_reg.tvalues[1:],
    'p': ols_reg.pvalues[1:]
}, index=CODE_METRICS)
print(f"\nCoefficients:\n{coef_df.round(4)}")
coef_df.to_csv(OUT_DIR / 'regression_coefs.csv')

# By-round regression (does relationship change across turns?)
print("\nR² by Round:")
for r in ['T1','T2','T3']:
    sub = df[df['round_label']==r]
    Xr = sub[CODE_METRICS].fillna(0).values
    yr = sub[TARGET].values
    Xr_c = sm.add_constant(Xr)
    m = sm.OLS(yr, Xr_c).fit()
    print(f"  {r}: R²={m.rsquared:.4f}  adj-R²={m.rsquared_adj:.4f}")

# ─────────────────────────────────────────────────────────────────────────────
# 7. INTERACTION EFFECTS  (Category × Round)
# ─────────────────────────────────────────────────────────────────────────────
print("\n" + "="*70)
print("7. INTERACTION EFFECTS")
print("="*70)

pivot_cat_round = df.pivot_table(
    values=TARGET, index='category', columns='round_label', aggfunc='mean'
)[['T1','T2','T3']]
pivot_cat_round = pivot_cat_round.reindex(CAT_ORDER)
pivot_cat_round['Δ12'] = pivot_cat_round['T2'] - pivot_cat_round['T1']
pivot_cat_round['Δ23'] = pivot_cat_round['T3'] - pivot_cat_round['T2']
pivot_cat_round['Overall'] = (pivot_cat_round['T1']+pivot_cat_round['T2']+pivot_cat_round['T3'])/3
print("\nCategory × Round interaction (mean score_reference_document):")
print(pivot_cat_round.round(2))
pivot_cat_round.to_csv(OUT_DIR / 'interaction_cat_round.csv')

# Top-10 models interaction
top10_models = desc_model.head(10).index.tolist()
pivot_model_round = df[df['model'].isin(top10_models)].pivot_table(
    values=TARGET, index='model', columns='round_label', aggfunc='mean'
)[['T1','T2','T3']]
pivot_model_round['Δ12'] = pivot_model_round['T2'] - pivot_model_round['T1']
pivot_model_round['Δ23'] = pivot_model_round['T3'] - pivot_model_round['T2']
print(f"\nTop-10 Models × Round:\n{pivot_model_round.round(2)}")

# ─────────────────────────────────────────────────────────────────────────────
# 8. FIGURES
# ─────────────────────────────────────────────────────────────────────────────
print("\n" + "="*70)
print("8. GENERATING FIGURES")
print("="*70)

# ── Figure 1: Correlation heatmap (all metrics) ─────────────────────────────
fig, axes = plt.subplots(1, 2, figsize=(18, 7))
for ax, corr_mat, title in zip(
        axes,
        [corr_pearson, corr_spearman],
        ['Pearson Correlation', 'Spearman Rank Correlation']):
    mask = np.zeros_like(corr_mat, dtype=bool)
    mask[np.triu_indices_from(mask, k=1)] = True
    sns.heatmap(corr_mat, ax=ax, mask=mask, cmap='RdYlGn', center=0,
                vmin=-1, vmax=1, annot=True, fmt='.2f', annot_kws={'size':7},
                linewidths=0.3, cbar_kws={'shrink':0.8})
    ax.set_title(title, fontweight='bold', pad=12)
    ax.tick_params(axis='x', rotation=45)
    ax.tick_params(axis='y', rotation=0)
plt.suptitle('Metric Correlation Analysis — SimBench (pretrain models)',
             fontsize=13, fontweight='bold', y=1.01)
plt.tight_layout()
fig.savefig(OUT_DIR / 'fig1_correlation_heatmap.pdf', bbox_inches='tight')
fig.savefig(OUT_DIR / 'fig1_correlation_heatmap.png', bbox_inches='tight')
plt.close()
print("  Saved: fig1_correlation_heatmap")

# ── Figure 2: PCA biplot ────────────────────────────────────────────────────
fig, axes = plt.subplots(1, 2, figsize=(16, 6))

# Scree plot
ax = axes[0]
n_components = len(exp_var)
bars = ax.bar(range(1, n_components+1), exp_var*100, color=CB_PALETTE[0],
              alpha=0.7, edgecolor='white')
ax.plot(range(1, n_components+1), cum_var*100, 'o-', color=CB_PALETTE[1],
        linewidth=2, markersize=5, label='Cumulative')
ax.axhline(95, color='gray', linestyle='--', linewidth=1, label='95% threshold')
ax.set_xlabel('Principal Component')
ax.set_ylabel('Explained Variance (%)')
ax.set_title('PCA Scree Plot', fontweight='bold')
ax.legend()
ax.set_xticks(range(1, n_components+1))

# Biplot (PC1 vs PC2, colored by category)
ax = axes[1]
cat_colors = {c: CB_PALETTE[i] for i, c in enumerate(CAT_ORDER)}
for cat in CAT_ORDER:
    mask = df['category'] == cat
    ax.scatter(df.loc[mask,'PC1'], df.loc[mask,'PC2'],
               c=cat_colors[cat], alpha=0.3, s=15, label=cat)

# Loading arrows (scaled)
scale = 3.5
for j, feat in enumerate(metric_cols):
    lx, ly = pca.components_[0,j]*scale, pca.components_[1,j]*scale
    ax.annotate('', xy=(lx, ly), xytext=(0,0),
                arrowprops=dict(arrowstyle='->', color='black', lw=1.2))
    ax.text(lx*1.15, ly*1.15, feat, fontsize=6.5, ha='center', va='center',
            color='black')
ax.axhline(0, color='gray', lw=0.5); ax.axvline(0, color='gray', lw=0.5)
ax.set_xlabel(f'PC1 ({exp_var[0]*100:.1f}%)')
ax.set_ylabel(f'PC2 ({exp_var[1]*100:.1f}%)')
ax.set_title('PCA Biplot (Loadings + Category Scores)', fontweight='bold')
ax.legend(loc='upper right', markerscale=2)
plt.suptitle('Principal Component Analysis of SimBench Metrics',
             fontsize=13, fontweight='bold')
plt.tight_layout()
fig.savefig(OUT_DIR / 'fig2_pca.pdf', bbox_inches='tight')
fig.savefig(OUT_DIR / 'fig2_pca.png', bbox_inches='tight')
plt.close()
print("  Saved: fig2_pca")

# ── Figure 3: Category × Round interaction (line + bar) ─────────────────────
fig, axes = plt.subplots(1, 2, figsize=(16, 6))

# Interaction line plot
ax = axes[0]
for cat in CAT_ORDER:
    row = pivot_cat_round.loc[cat]
    vals = [row['T1'], row['T2'], row['T3']]
    ax.plot(['T1','T2','T3'], vals, 'o-', linewidth=2.5, markersize=8,
            label=cat, color=cat_colors[cat])
ax.set_xlabel('Interaction Round')
ax.set_ylabel('Mean J-LLM Score (0–100)')
ax.set_title('Category × Round Interaction Plot', fontweight='bold')
ax.legend(title='Category')
ax.set_ylim(0, 80)

# Δ12 and Δ23 grouped bar
ax = axes[1]
x = np.arange(len(CAT_ORDER))
w = 0.35
ax.bar(x - w/2, pivot_cat_round.loc[CAT_ORDER,'Δ12'],
       width=w, label='Δ T1→T2', color=CB_PALETTE[2], alpha=0.85, edgecolor='white')
ax.bar(x + w/2, pivot_cat_round.loc[CAT_ORDER,'Δ23'],
       width=w, label='Δ T2→T3', color=CB_PALETTE[3], alpha=0.85, edgecolor='white')
ax.axhline(0, color='black', linewidth=0.8)
ax.set_xticks(x); ax.set_xticklabels(CAT_ORDER)
ax.set_xlabel('Category')
ax.set_ylabel('Score Change (Δ)')
ax.set_title('Multi-Turn Score Changes by Category', fontweight='bold')
ax.legend()

plt.suptitle('Category × Round Interaction Effects on J-LLM Score',
             fontsize=13, fontweight='bold')
plt.tight_layout()
fig.savefig(OUT_DIR / 'fig3_interaction_cat_round.pdf', bbox_inches='tight')
fig.savefig(OUT_DIR / 'fig3_interaction_cat_round.png', bbox_inches='tight')
plt.close()
print("  Saved: fig3_interaction_cat_round")

# ── Figure 4: ANOVA violin plots ────────────────────────────────────────────
fig, axes = plt.subplots(1, 2, figsize=(16, 6))

ax = axes[0]
order = CAT_ORDER
palette = {c: cat_colors[c] for c in CAT_ORDER}
sns.violinplot(data=df, x='category', y=TARGET, order=order,
               palette=palette, inner='box', ax=ax, linewidth=1.2)
ax.set_xlabel('Category')
ax.set_ylabel('J-LLM Score (score_reference_document)')
ax.set_title(f'Score Distribution by Category\n(F={F_cat:.1f}, p={p_cat:.2e})',
             fontweight='bold')

ax = axes[1]
round_colors = {'T1': CB_PALETTE[0], 'T2': CB_PALETTE[2], 'T3': CB_PALETTE[1]}
sns.violinplot(data=df, x='round_label', y=TARGET, order=['T1','T2','T3'],
               palette=round_colors, inner='box', ax=ax, linewidth=1.2)
ax.set_xlabel('Interaction Round')
ax.set_ylabel('J-LLM Score (score_reference_document)')
ax.set_title(f'Score Distribution by Round\n(F={F_rnd:.1f}, p={p_rnd:.2e})',
             fontweight='bold')

plt.suptitle('ANOVA: Score Distributions by Category and Round',
             fontsize=13, fontweight='bold')
plt.tight_layout()
fig.savefig(OUT_DIR / 'fig4_anova_violin.pdf', bbox_inches='tight')
fig.savefig(OUT_DIR / 'fig4_anova_violin.png', bbox_inches='tight')
plt.close()
print("  Saved: fig4_anova_violin")

# ── Figure 5: Regression — actual vs predicted + coefficient plot ────────────
fig, axes = plt.subplots(1, 2, figsize=(16, 6))

# Actual vs. predicted scatter
ax = axes[0]
y_pred = ols_reg.fittedvalues
ax.scatter(y_pred, y_reg, alpha=0.15, s=10, color=CB_PALETTE[0])
mn, mx = min(y_pred.min(), y_reg.min()), max(y_pred.max(), y_reg.max())
ax.plot([mn,mx],[mn,mx],'r--', linewidth=1.5, label='y=x')
ax.set_xlabel('Predicted J-LLM Score')
ax.set_ylabel('Actual J-LLM Score')
ax.set_title(f'Regression: Actual vs Predicted\n(R²={ols_reg.rsquared:.3f})',
             fontweight='bold')
ax.legend()

# Coefficient plot
ax = axes[1]
coef_sorted = coef_df.sort_values('coef')
colors = [CB_PALETTE[2] if v > 0 else CB_PALETTE[3] for v in coef_sorted['coef']]
ax.barh(range(len(coef_sorted)), coef_sorted['coef'], color=colors, alpha=0.85)
ax.errorbar(coef_sorted['coef'], range(len(coef_sorted)),
            xerr=1.96*coef_sorted['std_err'], fmt='none', color='black',
            capsize=3, linewidth=1)
ax.axvline(0, color='black', linewidth=0.8)
ax.set_yticks(range(len(coef_sorted)))
ax.set_yticklabels(coef_sorted.index, fontsize=8)
ax.set_xlabel('OLS Coefficient (95% CI)')
ax.set_title('Regression Coefficients\n(Predicting J-LLM from CodeBLEU/ROUGE)',
             fontweight='bold')

plt.suptitle('Linear Regression Analysis: J-LLM Score Predictors',
             fontsize=13, fontweight='bold')
plt.tight_layout()
fig.savefig(OUT_DIR / 'fig5_regression.pdf', bbox_inches='tight')
fig.savefig(OUT_DIR / 'fig5_regression.png', bbox_inches='tight')
plt.close()
print("  Saved: fig5_regression")

# ── Figure 6: Full model heatmap (model × category) ─────────────────────────
pivot_mc = df.pivot_table(values=TARGET, index='model', columns='category', aggfunc='mean')
pivot_mc = pivot_mc[CAT_ORDER]
pivot_mc['Average'] = pivot_mc.mean(axis=1)
pivot_mc = pivot_mc.sort_values('Average', ascending=False)

fig, ax = plt.subplots(figsize=(10, 14))
sns.heatmap(pivot_mc.drop(columns='Average'), ax=ax,
            cmap='YlOrRd', vmin=0, vmax=70,
            annot=True, fmt='.0f', annot_kws={'size':8},
            linewidths=0.3, cbar_kws={'label':'Mean J-LLM Score','shrink':0.6})
ax.set_title('Model × Category: Mean J-LLM Score (score_reference_document)\n'
             'Rows sorted by average performance', fontweight='bold', pad=12)
ax.set_xlabel('Simulation Category'); ax.set_ylabel('Model')
ax.tick_params(axis='x', rotation=0)
ax.tick_params(axis='y', rotation=0, labelsize=8)
plt.tight_layout()
fig.savefig(OUT_DIR / 'fig6_model_category_heatmap.pdf', bbox_inches='tight')
fig.savefig(OUT_DIR / 'fig6_model_category_heatmap.png', bbox_inches='tight')
plt.close()
pivot_mc.to_csv(OUT_DIR / 'model_category_means.csv')
print("  Saved: fig6_model_category_heatmap")

# ── Figure 7: Summary dashboard ─────────────────────────────────────────────
fig = plt.figure(figsize=(20, 14))
gs = gridspec.GridSpec(2, 3, figure=fig, hspace=0.4, wspace=0.35)

# 7a: Category means with error bars
ax = fig.add_subplot(gs[0,0])
means = desc_cat['Mean']; stds = desc_cat['Std']
bars = ax.bar(CAT_ORDER, means, yerr=stds, capsize=5,
              color=[cat_colors[c] for c in CAT_ORDER], alpha=0.85,
              edgecolor='white', error_kw={'linewidth':1.5})
ax.set_xlabel('Category'); ax.set_ylabel('Mean Score ± SD')
ax.set_title('Mean J-LLM Score by Category', fontweight='bold')
ax.set_ylim(0, 75)
for bar, v in zip(bars, means):
    ax.text(bar.get_x()+bar.get_width()/2, bar.get_height()+1, f'{v:.1f}',
            ha='center', va='bottom', fontsize=9, fontweight='bold')

# 7b: Round means
ax = fig.add_subplot(gs[0,1])
round_means = desc_round['mean']; round_stds = desc_round['std']
bars = ax.bar(['T1','T2','T3'], round_means, yerr=round_stds, capsize=5,
              color=[CB_PALETTE[0],CB_PALETTE[2],CB_PALETTE[1]], alpha=0.85,
              edgecolor='white', error_kw={'linewidth':1.5})
ax.set_xlabel('Round'); ax.set_ylabel('Mean Score ± SD')
ax.set_title('Mean J-LLM Score by Round', fontweight='bold')
ax.set_ylim(0, 75)
for bar, v in zip(bars, round_means):
    ax.text(bar.get_x()+bar.get_width()/2, bar.get_height()+1, f'{v:.1f}',
            ha='center', va='bottom', fontsize=9, fontweight='bold')

# 7c: PCA scree
ax = fig.add_subplot(gs[0,2])
ax.bar(range(1,len(exp_var)+1), exp_var*100, color=CB_PALETTE[0], alpha=0.7)
ax.plot(range(1,len(exp_var)+1), cum_var*100, 'o-', color=CB_PALETTE[1],
        markersize=5, linewidth=2)
ax.axhline(95, color='gray', linestyle='--', linewidth=1)
ax.set_xlabel('PC'); ax.set_ylabel('Explained Variance (%)')
ax.set_title('PCA Scree Plot', fontweight='bold')
ax.set_xticks(range(1,len(exp_var)+1))

# 7d: Interaction heatmap
ax = fig.add_subplot(gs[1,0])
heat_data = pivot_cat_round[['T1','T2','T3']].reindex(CAT_ORDER)
sns.heatmap(heat_data, ax=ax, cmap='YlOrRd', vmin=0, vmax=70,
            annot=True, fmt='.1f', annot_kws={'size':10},
            linewidths=0.5, cbar_kws={'label':'Mean Score','shrink':0.8})
ax.set_title('Category × Round\nMean J-LLM Score', fontweight='bold')
ax.tick_params(axis='x', rotation=0); ax.tick_params(axis='y', rotation=0)

# 7e: Δ changes
ax = fig.add_subplot(gs[1,1])
x = np.arange(len(CAT_ORDER)); w = 0.35
ax.bar(x-w/2, pivot_cat_round.loc[CAT_ORDER,'Δ12'], width=w,
       label='Δ T1→T2', color=CB_PALETTE[2], alpha=0.85)
ax.bar(x+w/2, pivot_cat_round.loc[CAT_ORDER,'Δ23'], width=w,
       label='Δ T2→T3', color=CB_PALETTE[3], alpha=0.85)
ax.axhline(0, color='black', lw=0.8)
ax.set_xticks(x); ax.set_xticklabels(CAT_ORDER)
ax.set_ylabel('Score Change Δ'); ax.set_title('Multi-Turn Score Changes', fontweight='bold')
ax.legend(fontsize=8)

# 7f: Actual vs predicted
ax = fig.add_subplot(gs[1,2])
ax.scatter(y_pred, y_reg, alpha=0.1, s=8, color=CB_PALETTE[0])
ax.plot([0,100],[0,100],'r--', linewidth=1.5)
ax.set_xlabel('Predicted Score'); ax.set_ylabel('Actual Score')
ax.set_title(f'OLS Regression\nR²={ols_reg.rsquared:.3f}', fontweight='bold')

fig.suptitle('SimBench Multivariate Analysis — Summary Dashboard',
             fontsize=15, fontweight='bold', y=1.01)
fig.savefig(OUT_DIR / 'fig7_summary_dashboard.pdf', bbox_inches='tight')
fig.savefig(OUT_DIR / 'fig7_summary_dashboard.png', bbox_inches='tight')
plt.close()
print("  Saved: fig7_summary_dashboard")

# ─────────────────────────────────────────────────────────────────────────────
# 9. SUMMARY STATISTICS TABLE
# ─────────────────────────────────────────────────────────────────────────────
print("\n" + "="*70)
print("9. MULTIVARIATE SUMMARY")
print("="*70)

summary = {
    'N_total': len(df),
    'N_models': df['model'].nunique(),
    'N_systems': df['system'].nunique(),
    'N_categories': df['category'].nunique(),
    'Overall_mean': df[TARGET].mean(),
    'Overall_std': df[TARGET].std(),
    'ANOVA_Category_F': F_cat,
    'ANOVA_Category_p': p_cat,
    'ANOVA_Round_F': F_rnd,
    'ANOVA_Round_p': p_rnd,
    'ANOVA_TypeIII_Category_eta2': anova_table.loc[anova_table.index.str.contains('category'),'eta_sq'].values[0],
    'ANOVA_TypeIII_Round_eta2': anova_table.loc[anova_table.index.str.contains('round'),'eta_sq'].values[0],
    'PCA_PC1_var': exp_var[0],
    'PCA_PC2_var': exp_var[1],
    'PCA_n_for_95pct': int(np.searchsorted(cum_var, 0.95))+1,
    'Regression_R2': ols_reg.rsquared,
    'Regression_adjR2': ols_reg.rsquared_adj,
    'Best_category': desc_cat['Mean'].idxmax(),
    'Hardest_category': desc_cat['Mean'].idxmin(),
    'Best_round': desc_round['mean'].idxmax(),
    'Best_model': desc_model.index[0],
    'Worst_model': desc_model.index[-1],
}
summary_df = pd.DataFrame(list(summary.items()), columns=['Metric','Value'])
summary_df.to_csv(OUT_DIR / 'multivariate_summary.csv', index=False)

for k,v in summary.items():
    if isinstance(v, float):
        print(f"  {k:45s}: {v:.4f}")
    else:
        print(f"  {k:45s}: {v}")

print(f"\n{'='*70}")
print(f"Sections 1-9 outputs saved to: {OUT_DIR}")
print("="*70)

# ═════════════════════════════════════════════════════════════════════════════
# EXTENDED ANALYSIS  — Model Metadata Factors
# (Open/Closed source · Model Size · Release Date)
# ═════════════════════════════════════════════════════════════════════════════

from datetime import datetime
import matplotlib.dates as mdates

# ── Model metadata (from plot_with_exact_dates_ref_doc.py) ────────────────
MODEL_METADATA = {
    'claude-4-sonnet-20250514':     {'company':'Anthropic', 'size':None,  'date':'2025-05-22'},
    'o3':                           {'company':'OpenAI',    'size':None,  'date':'2025-04-16'},
    'claude-3-7-sonnet-20250219':   {'company':'Anthropic', 'size':None,  'date':'2025-02-24'},
    'o4-mini':                      {'company':'OpenAI',    'size':None,  'date':'2025-04-16'},
    'qwen3-235b-a22b':              {'company':'Alibaba',   'size':235,   'date':'2025-04-29'},
    'Gemini-2.5-pro':               {'company':'Google',    'size':None,  'date':'2025-03-25'},
    'gpt-4.1-mini':                 {'company':'OpenAI',    'size':None,  'date':'2025-04-14'},
    'gpt-4o-mini':                  {'company':'OpenAI',    'size':None,  'date':'2024-07-18'},
    'llama4_maverick':              {'company':'Meta',      'size':400,   'date':'2025-04-05'},
    'llama4_scout':                 {'company':'Meta',      'size':109,   'date':'2025-04-05'},
    'llama-3.3-70b-instruct':       {'company':'Meta',      'size':70,    'date':'2024-12-06'},
    'deepseek-r1-32b':              {'company':'DeepSeek',  'size':32,    'date':'2025-01-20'},
    'gpt-4.1-nano':                 {'company':'OpenAI',    'size':None,  'date':'2025-04-14'},
    'llama-3.1-70b-instruct':       {'company':'Meta',      'size':70,    'date':'2024-07-23'},
    'gpt-4.1':                      {'company':'OpenAI',    'size':None,  'date':'2025-04-14'},
    'Gemini-1.5-pro':               {'company':'Google',    'size':None,  'date':'2024-02-15'},
    'codestral-22b-instruct-v0.1':  {'company':'Mistral',   'size':22,    'date':'2024-05-29'},
    'llama-3.1-405b-instruct':      {'company':'Meta',      'size':405,   'date':'2024-07-23'},
    'mixtral-8x22b-instruct-v0.1':  {'company':'Mistral',   'size':176,   'date':'2024-04-17'},
    'llama-3.1-8b-instruct':        {'company':'Meta',      'size':8,     'date':'2024-07-23'},
    'mistral-nemo-12b-instruct':    {'company':'Mistral',   'size':12,    'date':'2024-07-18'},
    'gemma-2-27b-it':               {'company':'Google',    'size':27,    'date':'2024-06-26'},
    'mixtral-8x7b-instruct-v0.1':   {'company':'Mistral',   'size':47,    'date':'2023-12-11'},
    'claude-3-5-sonnet':            {'company':'Anthropic', 'size':None,  'date':'2024-06-21'},
    'deepseek-r1-8b':               {'company':'DeepSeek',  'size':8,     'date':'2025-01-20'},
    'gpt-4o':                       {'company':'OpenAI',    'size':None,  'date':'2024-05-13'},
    'nemotron-4-340b-instruct':     {'company':'NVIDIA',    'size':340,   'date':'2024-06-14'},
    'gemma-2-9b-it':                {'company':'Google',    'size':9,     'date':'2024-06-27'},
    'gemma-2-2b-it':                {'company':'Google',    'size':2,     'date':'2024-07-31'},
    'mamba-codestral-7b-v0.1':      {'company':'Mistral',   'size':7,     'date':'2024-07-16'},
    'gemma-3-1b-it':                {'company':'Google',    'size':1,     'date':'2025-03-12'},
    'phi-3-mini-128k-instruct':     {'company':'Microsoft', 'size':3.8,   'date':'2024-04-24'},
    'phi-3-medium-128k-instruct':   {'company':'Microsoft', 'size':14,    'date':'2024-05-21'},
}

# Closed-source = weights not publicly released
CLOSED_SOURCE = {'Anthropic', 'OpenAI', 'Google'}   # Gemini is API-only
# Note: Google Gemma is open-weight; Google Gemini is closed. Split carefully:
CLOSED_SOURCE_MODELS = {m for m, v in MODEL_METADATA.items()
                        if v['company'] in CLOSED_SOURCE and v['size'] is None}

# Build per-model metadata table aligned with the scores data
model_meta_rows = []
for model, meta in MODEL_METADATA.items():
    is_closed = model in CLOSED_SOURCE_MODELS
    model_meta_rows.append({
        'model':       model,
        'company':     meta['company'],
        'size_b':      meta['size'],          # billions of params (None = unknown/closed)
        'release_date': datetime.strptime(meta['date'], '%Y-%m-%d'),
        'source_type': 'Closed' if is_closed else 'Open',
    })
meta_df = pd.DataFrame(model_meta_rows)

# Per-model mean score
model_scores = df.groupby('model')[TARGET].mean().reset_index()
model_scores.columns = ['model', 'mean_score']

# Merge
mdf = model_scores.merge(meta_df, on='model', how='inner')
mdf['days_since_ref'] = (mdf['release_date'] - datetime(2023, 12, 1)).dt.days
mdf['log_size'] = np.log10(mdf['size_b'].where(mdf['size_b'].notna()))

print(f"\nMetadata matched: {len(mdf)} / {model_scores['model'].nunique()} models")
print(f"  Open-source : {(mdf['source_type']=='Open').sum()}")
print(f"  Closed-source: {(mdf['source_type']=='Closed').sum()}")

# ─────────────────────────────────────────────────────────────────────────────
# 10. OPEN vs CLOSED SOURCE COMPARISON
# ─────────────────────────────────────────────────────────────────────────────
print("\n" + "="*70)
print("10. OPEN vs CLOSED SOURCE COMPARISON")
print("="*70)

open_scores   = mdf[mdf['source_type']=='Open']['mean_score']
closed_scores = mdf[mdf['source_type']=='Closed']['mean_score']

print(f"\n  Open-source  (n={len(open_scores)}): mean={open_scores.mean():.2f} ± {open_scores.std():.2f}"
      f"  range=[{open_scores.min():.1f}, {open_scores.max():.1f}]")
print(f"  Closed-source (n={len(closed_scores)}): mean={closed_scores.mean():.2f} ± {closed_scores.std():.2f}"
      f"  range=[{closed_scores.min():.1f}, {closed_scores.max():.1f}]")

# Mann-Whitney U (non-parametric, appropriate for small groups)
from scipy.stats import mannwhitneyu, ttest_ind
U, p_mw = mannwhitneyu(open_scores, closed_scores, alternative='two-sided')
t, p_tt = ttest_ind(open_scores, closed_scores)

# Cohen's d
pooled_std = np.sqrt((open_scores.std()**2 + closed_scores.std()**2) / 2)
cohens_d   = (closed_scores.mean() - open_scores.mean()) / pooled_std
print(f"\n  Mann-Whitney U={U:.0f}, p={p_mw:.4f}")
print(f"  Independent t={t:.3f}, p={p_tt:.4f}")
print(f"  Cohen's d={cohens_d:.3f}  ({'large' if abs(cohens_d)>0.8 else 'medium' if abs(cohens_d)>0.5 else 'small'} effect)")

# Also merge source type into full df for ANOVA
df = df.merge(mdf[['model','source_type','size_b','release_date','days_since_ref','log_size','company']],
              on='model', how='left')

# ANOVA with source type
F_src, p_src = f_oneway(
    df[df['source_type']=='Open'][TARGET].dropna(),
    df[df['source_type']=='Closed'][TARGET].dropna()
)
print(f"  One-way ANOVA (source type): F={F_src:.2f}, p={p_src:.4e}")

# Per-category open vs closed
print("\n  Per-category gap (Closed mean − Open mean):")
for cat in CAT_ORDER:
    sub = df[df['category']==cat]
    o = sub[sub['source_type']=='Open'][TARGET].mean()
    c = sub[sub['source_type']=='Closed'][TARGET].mean()
    print(f"    {cat:4s}: Closed={c:.1f}  Open={o:.1f}  Δ={c-o:+.1f}")

open_closed_cat = df.groupby(['category','source_type'])[TARGET].mean().unstack()
open_closed_cat.to_csv(OUT_DIR / 'open_closed_by_category.csv')

# ─────────────────────────────────────────────────────────────────────────────
# 11. MODEL SIZE EFFECT (Open-source only)
# ─────────────────────────────────────────────────────────────────────────────
print("\n" + "="*70)
print("11. MODEL SIZE EFFECT  (open-source, known size only)")
print("="*70)

mdf_open = mdf[(mdf['source_type']=='Open') & (mdf['size_b'].notna())].copy()
print(f"\n  Open-source with known size: {len(mdf_open)} models")
print(f"  Size range: {mdf_open['size_b'].min():.1f}B – {mdf_open['size_b'].max():.0f}B")

# Pearson / Spearman on raw size
r_size,  p_size  = pearsonr(mdf_open['size_b'],  mdf_open['mean_score'])
rs_size, ps_size = spearmanr(mdf_open['size_b'], mdf_open['mean_score'])
# On log10(size)
r_lsize,  p_lsize  = pearsonr(mdf_open['log_size'],  mdf_open['mean_score'])
rs_lsize, ps_lsize = spearmanr(mdf_open['log_size'], mdf_open['mean_score'])

print(f"\n  Size vs score:      Pearson r={r_size:+.3f} (p={p_size:.4f})"
      f"   Spearman ρ={rs_size:+.3f} (p={ps_size:.4f})")
print(f"  log10(Size) vs score: Pearson r={r_lsize:+.3f} (p={p_lsize:.4f})"
      f"   Spearman ρ={rs_lsize:+.3f} (p={ps_lsize:.4f})")

# OLS: score ~ log_size
X_sz = sm.add_constant(mdf_open['log_size'])
ols_sz = sm.OLS(mdf_open['mean_score'], X_sz).fit()
print(f"\n  OLS score ~ log10(size):  R²={ols_sz.rsquared:.4f}  "
      f"β_log_size={ols_sz.params['log_size']:.3f} (p={ols_sz.pvalues['log_size']:.4f})")

# Size bins analysis
mdf_open['size_bin'] = pd.cut(mdf_open['size_b'],
                               bins=[0, 10, 30, 100, 500],
                               labels=['Small\n(≤10B)', 'Mid\n(10–30B)',
                                       'Large\n(30–100B)', 'XLarge\n(>100B)'])
size_bin_stats = mdf_open.groupby('size_bin', observed=True)['mean_score'].agg(['mean','std','count'])
print(f"\n  By size bin:\n{size_bin_stats.round(2)}")
size_bin_stats.to_csv(OUT_DIR / 'size_bin_stats.csv')

# ─────────────────────────────────────────────────────────────────────────────
# 12. RELEASE DATE EFFECT
# ─────────────────────────────────────────────────────────────────────────────
print("\n" + "="*70)
print("12. RELEASE DATE EFFECT")
print("="*70)

# Overall date vs score
r_date,  p_date  = pearsonr(mdf['days_since_ref'],  mdf['mean_score'])
rs_date, ps_date = spearmanr(mdf['days_since_ref'], mdf['mean_score'])
print(f"\n  All models — date vs score:  Pearson r={r_date:+.3f} (p={p_date:.4f})"
      f"   Spearman ρ={rs_date:+.3f} (p={ps_date:.4f})")

# By source type
for src in ['Open','Closed']:
    sub = mdf[mdf['source_type']==src]
    if len(sub) > 3:
        r, p = pearsonr(sub['days_since_ref'], sub['mean_score'])
        slope, intercept, _, _, _ = stats.linregress(sub['days_since_ref'], sub['mean_score'])
        print(f"  {src:6s} only (n={len(sub)}) — Pearson r={r:+.3f} (p={p:.4f})"
              f"  slope={slope*30:.3f} pts/month")

# OLS: score ~ days
X_date = sm.add_constant(mdf['days_since_ref'])
ols_date = sm.OLS(mdf['mean_score'], X_date).fit()
print(f"\n  OLS score ~ days:  R²={ols_date.rsquared:.4f}  "
      f"β={ols_date.params['days_since_ref']:.4f} pts/day  "
      f"(≈{ols_date.params['days_since_ref']*30:.2f} pts/month)")

# Yearly breakdown
mdf['release_year'] = mdf['release_date'].dt.year
year_stats = mdf.groupby('release_year')['mean_score'].agg(['mean','std','count'])
print(f"\n  By release year:\n{year_stats.round(2)}")
year_stats.to_csv(OUT_DIR / 'year_stats.csv')

# ─────────────────────────────────────────────────────────────────────────────
# 13. MULTI-FACTOR REGRESSION  (source_type + log_size + days)
# ─────────────────────────────────────────────────────────────────────────────
print("\n" + "="*70)
print("13. MULTI-FACTOR REGRESSION  (source + log_size + days)")
print("="*70)

# Encode source type
mdf['is_closed'] = (mdf['source_type'] == 'Closed').astype(int)

# Full model (all 35 models; use 0 for log_size when unknown)
mdf_full = mdf.copy()
mdf_full['log_size_imp'] = mdf_full['log_size'].fillna(mdf_full['log_size'].median())

# Model 1: score ~ is_closed + days
f1 = 'mean_score ~ is_closed + days_since_ref'
m1 = smf.ols(f1, data=mdf_full).fit()
print(f"\n  Model 1: {f1}")
print(f"    R²={m1.rsquared:.4f}  adj-R²={m1.rsquared_adj:.4f}  F={m1.fvalue:.2f}  p={m1.f_pvalue:.4f}")
print(m1.params.round(3).to_string())

# Model 2: score ~ is_closed + days + log_size (open-source only, imputed)
f2 = 'mean_score ~ is_closed + days_since_ref + log_size_imp'
m2 = smf.ols(f2, data=mdf_full).fit()
print(f"\n  Model 2: {f2}")
print(f"    R²={m2.rsquared:.4f}  adj-R²={m2.rsquared_adj:.4f}  F={m2.fvalue:.2f}  p={m2.f_pvalue:.4f}")
print(m2.params.round(3).to_string())

# Model 3: open-source only, score ~ log_size + days
mdf_os = mdf_open.copy()
f3 = 'mean_score ~ log_size + days_since_ref'
m3 = smf.ols(f3, data=mdf_os).fit()
print(f"\n  Model 3 (open only, n={len(mdf_os)}): {f3}")
print(f"    R²={m3.rsquared:.4f}  adj-R²={m3.rsquared_adj:.4f}  F={m3.fvalue:.2f}  p={m3.f_pvalue:.4f}")
print(m3.params.round(3).to_string())

# Save regression summary
reg_results = pd.DataFrame({
    'Model': ['M1: closed+date', 'M2: closed+date+size', 'M3: open, size+date'],
    'n': [len(mdf_full), len(mdf_full), len(mdf_os)],
    'R2': [m1.rsquared, m2.rsquared, m3.rsquared],
    'adjR2': [m1.rsquared_adj, m2.rsquared_adj, m3.rsquared_adj],
    'F': [m1.fvalue, m2.fvalue, m3.fvalue],
    'p_F': [m1.f_pvalue, m2.f_pvalue, m3.f_pvalue],
})
reg_results.to_csv(OUT_DIR / 'extended_regression_summary.csv', index=False)
print(f"\n{reg_results.round(4)}")

# ─────────────────────────────────────────────────────────────────────────────
# 14. EXTENDED FIGURES
# ─────────────────────────────────────────────────────────────────────────────
print("\n" + "="*70)
print("14. GENERATING EXTENDED FIGURES")
print("="*70)

COMPANY_COLORS = {
    'Anthropic': '#E53935', 'OpenAI': '#8E24AA', 'Google': '#F9A825',
    'Meta': '#43A047', 'Microsoft': '#039BE5', 'Mistral': '#00ACC1',
    'DeepSeek': '#F06292', 'Alibaba': '#FFB300', 'NVIDIA': '#5C6BC0',
}

# ── Fig 8: Open vs Closed — boxplot + strip ──────────────────────────────────
fig, axes = plt.subplots(1, 3, figsize=(18, 6))

# 8a: overall open vs closed
ax = axes[0]
order_src = ['Open', 'Closed']
palette_src = {'Open': '#43A047', 'Closed': '#E53935'}
sns.boxplot(data=mdf, x='source_type', y='mean_score', order=order_src,
            palette=palette_src, width=0.5, ax=ax, linewidth=1.5,
            flierprops={'marker':'o','markersize':4,'alpha':0.5})
sns.stripplot(data=mdf, x='source_type', y='mean_score', order=order_src,
              palette=palette_src, jitter=True, size=6, alpha=0.7, ax=ax)
ax.set_xlabel('Source Type'); ax.set_ylabel('Mean J-LLM Score')
ax.set_title(f'Open vs Closed Source\n'
             f'(U={U:.0f}, p={p_mw:.3f}, d={cohens_d:.2f})', fontweight='bold')
# annotate means
for i, src in enumerate(order_src):
    m = mdf[mdf['source_type']==src]['mean_score'].mean()
    ax.text(i, m+1.5, f'{m:.1f}', ha='center', fontsize=10, fontweight='bold')

# 8b: per-category breakdown
ax = axes[1]
open_closed_plot = df.groupby(['category','source_type'])[TARGET].mean().reset_index()
open_closed_plot = open_closed_plot[open_closed_plot['category'].isin(CAT_ORDER)]
x = np.arange(len(CAT_ORDER)); w = 0.35
for i, src in enumerate(['Open','Closed']):
    vals = [open_closed_plot[(open_closed_plot['category']==c) &
                              (open_closed_plot['source_type']==src)][TARGET].values
            for c in CAT_ORDER]
    vals = [v[0] if len(v) else np.nan for v in vals]
    ax.bar(x + (i-0.5)*w, vals, width=w, label=src,
           color=palette_src[src], alpha=0.85, edgecolor='white')
ax.set_xticks(x); ax.set_xticklabels(CAT_ORDER)
ax.set_xlabel('Category'); ax.set_ylabel('Mean J-LLM Score')
ax.set_title('Open vs Closed by Category', fontweight='bold')
ax.legend()

# 8c: per-round breakdown
ax = axes[2]
open_closed_round = df.groupby(['round_label','source_type'])[TARGET].mean().reset_index()
x = np.arange(3); w = 0.35
for i, src in enumerate(['Open','Closed']):
    vals = [open_closed_round[(open_closed_round['round_label']==r) &
                               (open_closed_round['source_type']==src)][TARGET].values
            for r in ['T1','T2','T3']]
    vals = [v[0] if len(v) else np.nan for v in vals]
    ax.bar(x + (i-0.5)*w, vals, width=w, label=src,
           color=palette_src[src], alpha=0.85, edgecolor='white')
ax.set_xticks(x); ax.set_xticklabels(['T1','T2','T3'])
ax.set_xlabel('Round'); ax.set_ylabel('Mean J-LLM Score')
ax.set_title('Open vs Closed by Round', fontweight='bold')
ax.legend()

plt.suptitle('Open-Source vs Closed-Source Model Comparison',
             fontsize=13, fontweight='bold')
plt.tight_layout()
fig.savefig(OUT_DIR / 'fig8_open_vs_closed.pdf', bbox_inches='tight')
fig.savefig(OUT_DIR / 'fig8_open_vs_closed.png', bbox_inches='tight')
plt.close()
print("  Saved: fig8_open_vs_closed")

# ── Fig 9: Model size effect (open-source) ────────────────────────────────────
fig, axes = plt.subplots(1, 2, figsize=(16, 6))

# 9a: scatter log_size vs score, colored by company
ax = axes[0]
for company in mdf_open['company'].unique():
    sub = mdf_open[mdf_open['company']==company]
    ax.scatter(sub['log_size'], sub['mean_score'],
               c=COMPANY_COLORS.get(company,'#888'),
               s=90, alpha=0.85, label=company, edgecolors='white', linewidths=0.5, zorder=3)
    for _, row in sub.iterrows():
        ax.annotate(row['model'].replace('-instruct','').replace('-v0.1','')[:18],
                    (row['log_size'], row['mean_score']),
                    fontsize=6.5, ha='left', va='bottom',
                    xytext=(3, 3), textcoords='offset points', color='#333')

# regression line
x_fit = np.linspace(mdf_open['log_size'].min(), mdf_open['log_size'].max(), 100)
y_fit = ols_sz.params['const'] + ols_sz.params['log_size'] * x_fit
ax.plot(x_fit, y_fit, 'k--', linewidth=1.8, alpha=0.6,
        label=f'OLS (R²={ols_sz.rsquared:.3f})')
ax.set_xlabel('log₁₀(Model Size / B)')
ax.set_ylabel('Mean J-LLM Score')
ax.set_title(f'Model Size vs Performance (Open-Source)\n'
             f'Pearson r={r_lsize:.3f}, p={p_lsize:.3f}', fontweight='bold')
# custom x-ticks showing actual sizes
xticks = [0, np.log10(10), np.log10(30), np.log10(100), np.log10(400)]
xlabels = ['1B', '10B', '30B', '100B', '400B']
ax.set_xticks(xticks); ax.set_xticklabels(xlabels)
ax.legend(fontsize=8, loc='upper left')

# 9b: size bins box
ax = axes[1]
bin_order = ['Small\n(≤10B)', 'Mid\n(10–30B)', 'Large\n(30–100B)', 'XLarge\n(>100B)']
bin_colors = ['#AED6F1','#85C1E9','#5DADE2','#2E86C1']
sns.boxplot(data=mdf_open, x='size_bin', y='mean_score', order=bin_order,
            palette=bin_colors, ax=ax, linewidth=1.5,
            flierprops={'marker':'o','markersize':5,'alpha':0.5})
sns.stripplot(data=mdf_open, x='size_bin', y='mean_score', order=bin_order,
              color='#1A5276', jitter=True, size=6, alpha=0.7, ax=ax)
ax.set_xlabel('Model Size Category')
ax.set_ylabel('Mean J-LLM Score')
ax.set_title('Performance by Model Size Bin', fontweight='bold')
for i, bin_label in enumerate(bin_order):
    n = size_bin_stats.loc[bin_label, 'count'] if bin_label in size_bin_stats.index else 0
    ax.text(i, ax.get_ylim()[0]+0.5, f'n={int(n)}', ha='center', fontsize=9, color='gray')

plt.suptitle('Model Size Effect on J-LLM Performance (Open-Source Models)',
             fontsize=13, fontweight='bold')
plt.tight_layout()
fig.savefig(OUT_DIR / 'fig9_model_size_effect.pdf', bbox_inches='tight')
fig.savefig(OUT_DIR / 'fig9_model_size_effect.png', bbox_inches='tight')
plt.close()
print("  Saved: fig9_model_size_effect")

# ── Fig 10: Release date effect ───────────────────────────────────────────────
fig, axes = plt.subplots(1, 2, figsize=(18, 7))

# 10a: scatter date vs score, open vs closed with regression lines
ax = axes[0]
src_markers = {'Open': 'o', 'Closed': 's'}
for src, marker in src_markers.items():
    sub = mdf[mdf['source_type']==src]
    for company in sub['company'].unique():
        csub = sub[sub['company']==company]
        ax.scatter(csub['release_date'], csub['mean_score'],
                   c=COMPANY_COLORS.get(company,'#888'),
                   s=110 if src=='Closed' else 80,
                   marker=marker, alpha=0.85, zorder=4,
                   edgecolors='white', linewidths=0.5)
    # regression line per source type
    if len(sub) > 3:
        dates_num = mdates.date2num(sub['release_date'])
        slope, intercept, r_val, p_val, _ = stats.linregress(dates_num, sub['mean_score'])
        x_rng = pd.date_range(sub['release_date'].min(), sub['release_date'].max(), freq='D')
        y_rng = slope * mdates.date2num(x_rng) + intercept
        color = palette_src[src]
        ax.plot(x_rng, y_rng, color=color, linewidth=2.2, alpha=0.7,
                label=f'{src} (r={r_val:.2f}, p={p_val:.3f})')

# model name labels
for _, row in mdf.iterrows():
    ax.annotate(row['model'].replace('-instruct','').replace('-v0.1','')[:16],
                (row['release_date'], row['mean_score']),
                fontsize=6, ha='center', va='bottom',
                xytext=(0, 5), textcoords='offset points', color='#444')

ax.axhline(mdf['mean_score'].mean(), color='gray', linestyle='--',
           linewidth=1, alpha=0.5, label=f'Overall mean={mdf["mean_score"].mean():.1f}')
ax.xaxis.set_major_formatter(mdates.DateFormatter('%Y-%m'))
ax.xaxis.set_major_locator(mdates.MonthLocator(interval=3))
plt.setp(ax.xaxis.get_majorticklabels(), rotation=45, ha='right')
ax.set_xlabel('Release Date')
ax.set_ylabel('Mean J-LLM Score')
ax.set_title('Performance vs Release Date\n(○=Open, □=Closed; colored by company)', fontweight='bold')
ax.legend(fontsize=9)

# 10b: company-level bar chart, sorted by mean score
ax = axes[1]
company_stats = mdf.groupby('company').agg(
    mean_score=('mean_score','mean'),
    n=('mean_score','count'),
    std=('mean_score','std'),
    source_type=('source_type', lambda x: x.iloc[0])
).sort_values('mean_score', ascending=True)
colors_bar = [COMPANY_COLORS.get(c,'#888') for c in company_stats.index]
bars = ax.barh(range(len(company_stats)), company_stats['mean_score'],
               color=colors_bar, alpha=0.85, edgecolor='white')
ax.errorbar(company_stats['mean_score'], range(len(company_stats)),
            xerr=company_stats['std'].fillna(0), fmt='none',
            color='black', capsize=3, linewidth=1)
ax.set_yticks(range(len(company_stats)))
ax.set_yticklabels([f"{c} (n={int(r['n'])})" for c, r in company_stats.iterrows()])
ax.set_xlabel('Mean J-LLM Score')
ax.set_title('Mean Performance by Company\n(sorted ascending)', fontweight='bold')
for i, (c, row) in enumerate(company_stats.iterrows()):
    label = '🔒' if row['source_type']=='Closed' else '🔓'
    ax.text(row['mean_score']+0.3, i, f"{row['mean_score']:.1f} {label}",
            va='center', fontsize=9)

plt.suptitle('Release Date Effect and Company-Level Performance',
             fontsize=13, fontweight='bold')
plt.tight_layout()
fig.savefig(OUT_DIR / 'fig10_release_date_company.pdf', bbox_inches='tight')
fig.savefig(OUT_DIR / 'fig10_release_date_company.png', bbox_inches='tight')
plt.close()
print("  Saved: fig10_release_date_company")

# ── Fig 11: Extended summary — 2×3 dashboard ──────────────────────────────────
fig = plt.figure(figsize=(20, 12))
gs = gridspec.GridSpec(2, 3, figure=fig, hspace=0.42, wspace=0.35)

# 11a: open vs closed boxplot
ax = fig.add_subplot(gs[0,0])
for i, src in enumerate(['Open','Closed']):
    vals = mdf[mdf['source_type']==src]['mean_score']
    bp = ax.boxplot(vals, positions=[i], widths=0.5, patch_artist=True,
                    medianprops={'color':'black','linewidth':2},
                    boxprops={'facecolor':palette_src[src],'alpha':0.7})
    ax.scatter([i+np.random.uniform(-0.12,0.12,len(vals)) for _ in [None]][0],
               vals, c=palette_src[src], s=30, alpha=0.6, zorder=3)
    ax.text(i, vals.mean()+1.5, f'{vals.mean():.1f}', ha='center', fontsize=10, fontweight='bold')
ax.set_xticks([0,1]); ax.set_xticklabels(['Open','Closed'])
ax.set_ylabel('Mean Score'); ax.set_title(f'Open vs Closed\n(d={cohens_d:.2f}, p={p_mw:.3f})', fontweight='bold')

# 11b: size vs score scatter (open only)
ax = fig.add_subplot(gs[0,1])
ax.scatter(mdf_open['log_size'], mdf_open['mean_score'],
           c=[COMPANY_COLORS.get(c,'#888') for c in mdf_open['company']],
           s=80, alpha=0.8, edgecolors='white', linewidths=0.5)
ax.plot(x_fit, y_fit, 'k--', linewidth=1.5, alpha=0.6)
ax.set_xlabel('log₁₀(Size / B)')
ax.set_ylabel('Mean Score')
ax.set_title(f'Size vs Score (Open)\nr={r_lsize:.3f}, p={p_lsize:.3f}', fontweight='bold')
ax.set_xticks(xticks); ax.set_xticklabels(xlabels, fontsize=8)

# 11c: date vs score scatter
ax = fig.add_subplot(gs[0,2])
for src in ['Open','Closed']:
    sub = mdf[mdf['source_type']==src]
    ax.scatter(sub['days_since_ref'], sub['mean_score'],
               c=palette_src[src], s=70, alpha=0.8,
               marker='o' if src=='Open' else 's',
               label=src, edgecolors='white', linewidths=0.5)
# overall regression
xd = np.linspace(mdf['days_since_ref'].min(), mdf['days_since_ref'].max(), 100)
yd = ols_date.params['const'] + ols_date.params['days_since_ref']*xd
ax.plot(xd, yd, 'k--', linewidth=1.5, alpha=0.5)
ax.set_xlabel('Days Since Dec 2023')
ax.set_ylabel('Mean Score')
ax.set_title(f'Release Date vs Score\nr={r_date:.3f}, p={p_date:.3f}', fontweight='bold')
ax.legend(fontsize=8)

# 11d: per-category open vs closed heatmap
ax = fig.add_subplot(gs[1,0])
oc_heat = open_closed_cat.reindex(CAT_ORDER)
sns.heatmap(oc_heat, ax=ax, cmap='RdYlGn', vmin=20, vmax=55,
            annot=True, fmt='.1f', annot_kws={'size':11},
            linewidths=0.5, cbar_kws={'label':'Mean Score','shrink':0.7})
ax.set_title('Category × Source Type\nMean J-LLM Score', fontweight='bold')
ax.tick_params(axis='x', rotation=0); ax.tick_params(axis='y', rotation=0)

# 11e: size bin box
ax = fig.add_subplot(gs[1,1])
bin_data = [mdf_open[mdf_open['size_bin']==b]['mean_score'].values
            for b in bin_order if b in mdf_open['size_bin'].values]
valid_bins = [b for b in bin_order if b in mdf_open['size_bin'].values]
bps = ax.boxplot(bin_data, positions=range(len(valid_bins)), widths=0.5,
                 patch_artist=True, medianprops={'color':'black','linewidth':2})
for bp_patch, c in zip(bps['boxes'], bin_colors[:len(valid_bins)]):
    bp_patch.set_facecolor(c); bp_patch.set_alpha(0.7)
ax.set_xticks(range(len(valid_bins)))
ax.set_xticklabels(valid_bins, fontsize=8)
ax.set_xlabel('Size Bin'); ax.set_ylabel('Mean Score')
ax.set_title('Performance by Size Bin', fontweight='bold')

# 11f: regression comparison bar
ax = fig.add_subplot(gs[1,2])
models_r2 = ['Sections 1-9\n(CodeBLEU→J-LLM)', 'M1: source+date', 'M2: source+date+size', 'M3: open, size+date']
r2_vals = [ols_reg.rsquared, m1.rsquared, m2.rsquared, m3.rsquared]
colors_r2 = [CB_PALETTE[0], CB_PALETTE[1], CB_PALETTE[2], CB_PALETTE[3]]
ax.barh(range(4), r2_vals, color=colors_r2, alpha=0.85, edgecolor='white')
ax.set_yticks(range(4)); ax.set_yticklabels(models_r2, fontsize=8)
ax.set_xlabel('R²'); ax.set_title('Model Comparison: R²', fontweight='bold')
ax.set_xlim(0, 0.7)
for i, v in enumerate(r2_vals):
    ax.text(v+0.01, i, f'{v:.3f}', va='center', fontsize=9)

fig.suptitle('Extended Multivariate Analysis — Model Metadata Factors\n'
             '(Open/Closed Source · Model Size · Release Date)',
             fontsize=14, fontweight='bold')
fig.savefig(OUT_DIR / 'fig11_extended_dashboard.pdf', bbox_inches='tight')
fig.savefig(OUT_DIR / 'fig11_extended_dashboard.png', bbox_inches='tight')
plt.close()
print("  Saved: fig11_extended_dashboard")

# ─────────────────────────────────────────────────────────────────────────────
# FINAL SUMMARY
# ─────────────────────────────────────────────────────────────────────────────
print(f"\n{'='*70}")
print("COMPLETE — All outputs saved to:")
print(f"  {OUT_DIR}")
print(f"\nNew files (Sections 10-14):")
new_files = ['open_closed_by_category.csv','size_bin_stats.csv','year_stats.csv',
             'extended_regression_summary.csv',
             'fig8_open_vs_closed.png','fig8_open_vs_closed.pdf',
             'fig9_model_size_effect.png','fig9_model_size_effect.pdf',
             'fig10_release_date_company.png','fig10_release_date_company.pdf',
             'fig11_extended_dashboard.png','fig11_extended_dashboard.pdf']
for f in new_files:
    print(f"  {f}")
print("="*70)
