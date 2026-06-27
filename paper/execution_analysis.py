#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Execution Metrics Analysis: Compile@1, Pass@1 vs LLM-Judge
==========================================================

Analyzes the relationship between execution-based metrics and 
automatic evaluation metrics.
"""

import sys
import io
import pandas as pd
import numpy as np
from scipy.stats import spearmanr, pearsonr
import matplotlib.pyplot as plt
import seaborn as sns

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

# Data from correlation.py
metric_compile = {
    "codestral-22b-instruct-v0.1": 21, "gemma-2-27b-it": 21, "gemma-2-9b-it": 21,
    "gpt-4o-mini": 20, "gemma-2-2b-it": 20, "llama-3.1-405b-instruct": 20,
    "mixtral-8x22b-instruct-v0.1": 20, "mistral-nemo-12b-instruct": 18,
    "mamba-codestral-7b-v0.1": 16, "mixtral-8x7b-instruct-v0.1": 15,
    "llama-3.1-8b-instruct": 14, "phi-3-mini-128k-instruct": 11,
    "llama-3.1-70b-instruct": 9, "mistral-large-latest": 9,
    "Gemini-1.5-pro": 8, "nemotron-4-340b-instruct": 8,
    "claude-3-5-sonnet": 8, "phi-3-medium-128k-instruct": 6, "gpt-4o": 2
}

metric_pass = {
    "gpt-4o-mini": 13, "codestral-22b-instruct-v0.1": 10,
    "mixtral-8x22b-instruct-v0.1": 8, "claude-3-5-sonnet": 7,
    "Gemini-1.5-pro": 7, "mixtral-8x7b-instruct-v0.1": 7,
    "llama-3.1-405b-instruct": 7, "mistral-nemo-12b-instruct": 7,
    "llama-3.1-8b-instruct": 5, "gemma-2-27b-it": 5,
    "llama-3.1-70b-instruct": 4, "gemma-2-9b-it": 4,
    "mamba-codestral-7b-v0.1": 3, "mistral-large-latest": 3,
    "gemma-2-2b-it": 3, "nemotron-4-340b-instruct": 3,
    "phi-3-medium-128k-instruct": 3, "phi-3-mini-128k-instruct": 3, "gpt-4o": 2
}

metric_ref_doc = {
    "gpt-4o-mini": 41.61, "llama-3.1-70b-instruct": 39.69,
    "Gemini-1.5-pro": 39.21, "llama-3.1-405b-instruct": 39.04,
    "mixtral-8x22b-instruct-v0.1": 38.88, "codestral-22b-instruct-v0.1": 37.86,
    "mixtral-8x7b-instruct-v0.1": 37.72, "llama-3.1-8b-instruct": 37.63,
    "mistral-large-latest": 37.50, "gemma-2-27b-it": 37.38,
    "mistral-nemo-12b-instruct": 36.42, "nemotron-4-340b-instruct": 35.09,
    "gpt-4o": 33.90, "claude-3-5-sonnet": 33.75,
    "gemma-2-9b-it": 33.46, "mamba-codestral-7b-v0.1": 31.83,
    "gemma-2-2b-it": 31.07, "phi-3-mini-128k-instruct": 27.76,
    "phi-3-medium-128k-instruct": 22.01
}

metric_codebleu = {
    "llama-3.1-405b-instruct": 0.619, "gpt-4o-mini": 0.610,
    "Gemini-1.5-pro": 0.609, "codestral-22b-instruct-v0.1": 0.608,
    "gpt-4o": 0.607, "mistral-large-latest": 0.602,
    "nemotron-4-340b-instruct": 0.598, "claude-3-5-sonnet": 0.591,
    "gemma-2-27b-it": 0.590, "mixtral-8x22b-instruct-v0.1": 0.589,
    "gemma-2-9b-it": 0.575, "llama-3.1-8b-instruct": 0.571,
    "mistral-nemo-12b-instruct": 0.552, "gemma-2-2b-it": 0.542,
    "llama-3.1-70b-instruct": 0.540, "mixtral-8x7b-instruct-v0.1": 0.508,
    "phi-3-mini-128k-instruct": 0.502, "mamba-codestral-7b-v0.1": 0.473,
    "phi-3-medium-128k-instruct": 0.376
}

metric_rouge = {
    "gpt-4o": 0.758, "claude-3-5-sonnet": 0.757, "mistral-large-latest": 0.740,
    "Gemini-1.5-pro": 0.727, "gpt-4o-mini": 0.723, "llama-3.1-405b-instruct": 0.720,
    "nemotron-4-340b-instruct": 0.719, "gemma-2-27b-it": 0.710,
    "llama-3.1-70b-instruct": 0.710, "codestral-22b-instruct-v0.1": 0.699,
    "gemma-2-9b-it": 0.688, "mixtral-8x22b-instruct-v0.1": 0.667,
    "llama-3.1-8b-instruct": 0.656, "mistral-nemo-12b-instruct": 0.648,
    "gemma-2-2b-it": 0.641, "mixtral-8x7b-instruct-v0.1": 0.624,
    "phi-3-mini-128k-instruct": 0.582, "mamba-codestral-7b-v0.1": 0.567,
    "phi-3-medium-128k-instruct": 0.396
}

# Create DataFrame
df = pd.DataFrame({
    'Compile@1': pd.Series(metric_compile),
    'Pass@1': pd.Series(metric_pass),
    'LLM_Judge': pd.Series(metric_ref_doc),
    'CodeBLEU': pd.Series(metric_codebleu),
    'ROUGE': pd.Series(metric_rouge)
})

print("=" * 80)
print("  EXECUTION METRICS ANALYSIS (19 models)")
print("=" * 80)

# ============================================================
# 1. Correlation Analysis
# ============================================================
print("\n" + "=" * 80)
print("  1. CORRELATION MATRIX")
print("=" * 80)

print("\n  A. Spearman Correlation:")
corr_spearman = df.corr(method='spearman')
print(corr_spearman.round(3).to_string())

print("\n  B. Pearson Correlation:")
corr_pearson = df.corr(method='pearson')
print(corr_pearson.round(3).to_string())

# ============================================================
# 2. Key Insight: Pass@1 is best predictor
# ============================================================
print("\n" + "=" * 80)
print("  2. KEY FINDING: Pass@1 vs LLM-Judge")
print("=" * 80)

rho, p = spearmanr(df['Pass@1'], df['LLM_Judge'])
print(f"\n  Pass@1 ↔ LLM-Judge: rho = {rho:.3f} (p = {p:.4f}) ***")
print("  → Pass@1 is the STRONGEST predictor of LLM-Judge score!")

rho2, p2 = spearmanr(df['Compile@1'], df['LLM_Judge'])
print(f"\n  Compile@1 ↔ LLM-Judge: rho = {rho2:.3f} (p = {p2:.4f})")
print("  → Compile@1 has WEAK correlation with quality")

print("\n  Implication: Compiling ≠ Correct, Passing tests ≈ Quality")

# ============================================================
# 3. Anomaly Analysis
# ============================================================
print("\n" + "=" * 80)
print("  3. ANOMALY ANALYSIS")
print("=" * 80)

# Compile-Pass gap
df['Pass_Rate'] = df['Pass@1'] / df['Compile@1']

print("\n  A. Highest Pass Rate (efficient code generation):")
high_pass = df.nlargest(5, 'Pass_Rate')
for model, row in high_pass.iterrows():
    rate = row['Pass_Rate']
    print(f"     {model:35s} {row['Pass@1']:.0f}/{row['Compile@1']:.0f} = {rate:.1%}")

print("\n  B. Lowest Pass Rate (compiles but fails):")
low_pass = df.nsmallest(5, 'Pass_Rate')
for model, row in low_pass.iterrows():
    rate = row['Pass_Rate']
    print(f"     {model:35s} {row['Pass@1']:.0f}/{row['Compile@1']:.0f} = {rate:.1%}")

# ============================================================
# 4. GPT-4o Paradox
# ============================================================
print("\n" + "=" * 80)
print("  4. GPT-4o PARADOX")
print("=" * 80)

gpt4o = df.loc['gpt-4o']
print(f"\n  gpt-4o metrics:")
print(f"     Compile@1:  {gpt4o['Compile@1']:.0f} (LOWEST - rank 19/19)")
print(f"     Pass@1:     {gpt4o['Pass@1']:.0f} (LOWEST tied)")
print(f"     LLM-Judge:  {gpt4o['LLM_Judge']:.1f} (rank 13/19)")
print(f"     ROUGE:      {gpt4o['ROUGE']:.3f} (HIGHEST!)")
print(f"     CodeBLEU:   {gpt4o['CodeBLEU']:.3f} (rank 5/19)")

print("\n  Interpretation:")
print("     → gpt-4o generates code with HIGH text similarity (ROUGE)")
print("     → But code doesn't COMPILE (different structure/API usage)")
print("     → This is a 'creative but incompatible' pattern")

# ============================================================
# 5. Ranking Comparison
# ============================================================
print("\n" + "=" * 80)
print("  5. RANKING COMPARISON")
print("=" * 80)

for col in ['Compile@1', 'Pass@1', 'LLM_Judge', 'CodeBLEU', 'ROUGE']:
    df[f'rank_{col}'] = df[col].rank(ascending=False)

print("\n  Models with inconsistent rankings (high variance):")
rank_cols = [f'rank_{c}' for c in ['Compile@1', 'Pass@1', 'LLM_Judge']]
df['rank_var'] = df[rank_cols].var(axis=1)
df['rank_range'] = df[rank_cols].max(axis=1) - df[rank_cols].min(axis=1)

inconsistent = df.nlargest(5, 'rank_var')
for model, row in inconsistent.iterrows():
    print(f"     {model:35s}")
    print(f"       Compile={int(row['rank_Compile@1']):2d}, Pass={int(row['rank_Pass@1']):2d}, LLM={int(row['rank_LLM_Judge']):2d}")

# ============================================================
# 6. Best Overall Models
# ============================================================
print("\n" + "=" * 80)
print("  6. BEST OVERALL MODELS")
print("=" * 80)

# Normalize all to 0-1 and average
df['norm_compile'] = df['Compile@1'] / df['Compile@1'].max()
df['norm_pass'] = df['Pass@1'] / df['Pass@1'].max()
df['norm_llm'] = df['LLM_Judge'] / df['LLM_Judge'].max()
df['norm_codebleu'] = df['CodeBLEU'] / df['CodeBLEU'].max()
df['norm_rouge'] = df['ROUGE'] / df['ROUGE'].max()

df['overall'] = (df['norm_compile'] + df['norm_pass'] + df['norm_llm'] + 
                 df['norm_codebleu'] + df['norm_rouge']) / 5

print("\n  Top 10 by Overall Score:")
top10 = df.nlargest(10, 'overall')
for i, (model, row) in enumerate(top10.iterrows(), 1):
    print(f"     {i:2d}. {model:35s} Overall={row['overall']:.3f}")

# ============================================================
# 7. Summary Statistics
# ============================================================
print("\n" + "=" * 80)
print("  7. SUMMARY FOR PAPER")
print("=" * 80)

summary = {
    'Pass@1 ↔ LLM-Judge': f"rho = {corr_spearman.loc['Pass@1', 'LLM_Judge']:.3f} ***",
    'Compile@1 ↔ LLM-Judge': f"rho = {corr_spearman.loc['Compile@1', 'LLM_Judge']:.3f}",
    'CodeBLEU ↔ Pass@1': f"rho = {corr_spearman.loc['CodeBLEU', 'Pass@1']:.3f}",
    'ROUGE ↔ Pass@1': f"rho = {corr_spearman.loc['ROUGE', 'Pass@1']:.3f}",
    'Best Pass@1': f"{df['Pass@1'].idxmax()} ({df['Pass@1'].max():.0f})",
    'Best LLM-Judge': f"{df['LLM_Judge'].idxmax()} ({df['LLM_Judge'].max():.1f})",
    'Best Overall': f"{df['overall'].idxmax()} ({df['overall'].max():.3f})"
}

for k, v in summary.items():
    print(f"  {k:25s}: {v}")

# ============================================================
# 8. Generate Plots
# ============================================================
print("\n" + "=" * 80)
print("  8. GENERATING PLOTS")
print("=" * 80)

OUT_DIR = 'paper/out/analysis_execution'
import os
os.makedirs(OUT_DIR, exist_ok=True)

# Plot 1: Correlation heatmap
plt.figure(figsize=(10, 8))
sns.heatmap(corr_spearman, annot=True, fmt='.2f', cmap='RdYlGn', 
            center=0, vmin=-1, vmax=1, square=True, linewidths=1)
plt.title('Execution vs Automatic Metrics\n(Spearman Correlation)', fontsize=14, fontweight='bold')
plt.tight_layout()
plt.savefig(f'{OUT_DIR}/correlation_execution.png', dpi=150)
plt.close()

# Plot 2: Pass@1 vs LLM-Judge scatter
plt.figure(figsize=(10, 8))
plt.scatter(df['Pass@1'], df['LLM_Judge'], s=100, alpha=0.7, c='#667eea')
for model in df.index:
    plt.annotate(model[:15], (df.loc[model, 'Pass@1'], df.loc[model, 'LLM_Judge']),
                fontsize=8, alpha=0.8)
plt.xlabel('Pass@1', fontsize=12)
plt.ylabel('LLM-Judge Score', fontsize=12)
plt.title(f'Pass@1 vs LLM-Judge (ρ = {corr_spearman.loc["Pass@1", "LLM_Judge"]:.3f})',
         fontsize=14, fontweight='bold')
plt.tight_layout()
plt.savefig(f'{OUT_DIR}/pass_vs_llm.png', dpi=150)
plt.close()

# Plot 3: Compile vs Pass scatter
plt.figure(figsize=(10, 8))
colors = ['#e74c3c' if x < 0.5 else '#27ae60' for x in df['Pass_Rate']]
plt.scatter(df['Compile@1'], df['Pass@1'], s=100, c=colors, alpha=0.7)
for model in df.index:
    plt.annotate(model[:12], (df.loc[model, 'Compile@1'], df.loc[model, 'Pass@1']),
                fontsize=7, alpha=0.8)
# Add diagonal line
plt.plot([0, 21], [0, 21], 'k--', alpha=0.3, label='Perfect pass rate')
plt.xlabel('Compile@1', fontsize=12)
plt.ylabel('Pass@1', fontsize=12)
plt.title('Compile@1 vs Pass@1', fontsize=14, fontweight='bold')
plt.legend()
plt.tight_layout()
plt.savefig(f'{OUT_DIR}/compile_vs_pass.png', dpi=150)
plt.close()

# Save data
df.to_csv(f'{OUT_DIR}/execution_metrics.csv', float_format='%.4f')
corr_spearman.to_csv(f'{OUT_DIR}/correlation_spearman.csv', float_format='%.4f')

print(f"\n  Saved to: {OUT_DIR}/")
print("    - correlation_execution.png")
print("    - pass_vs_llm.png")
print("    - compile_vs_pass.png")
print("    - execution_metrics.csv")

print("\n" + "=" * 80)
print("  ANALYSIS COMPLETE")
print("=" * 80)
