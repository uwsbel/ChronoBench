#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Correlation between ELO and Execution Metrics (Pass@1, Compile@1)
"""

import sys
import io
import pandas as pd
import numpy as np
from scipy.stats import spearmanr, pearsonr
import matplotlib.pyplot as plt
import seaborn as sns

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

# Execution metrics from correlation.py
metric_compile = {
    'codestral-22b-instruct-v0.1': 21, 'gemma-2-27b-it': 21, 'gemma-2-9b-it': 21,
    'gpt-4o-mini': 20, 'gemma-2-2b-it': 20, 'llama-3.1-405b-instruct': 20,
    'mixtral-8x22b-instruct-v0.1': 20, 'mistral-nemo-12b-instruct': 18,
    'mamba-codestral-7b-v0.1': 16, 'mixtral-8x7b-instruct-v0.1': 15,
    'llama-3.1-8b-instruct': 14, 'phi-3-mini-128k-instruct': 11,
    'llama-3.1-70b-instruct': 9, 'mistral-large-latest': 9,
    'Gemini-1.5-pro': 8, 'nemotron-4-340b-instruct': 8,
    'claude-3-5-sonnet': 8, 'phi-3-medium-128k-instruct': 6, 'gpt-4o': 2
}

metric_pass = {
    'gpt-4o-mini': 13, 'codestral-22b-instruct-v0.1': 10,
    'mixtral-8x22b-instruct-v0.1': 8, 'claude-3-5-sonnet': 7,
    'Gemini-1.5-pro': 7, 'mixtral-8x7b-instruct-v0.1': 7,
    'llama-3.1-405b-instruct': 7, 'mistral-nemo-12b-instruct': 7,
    'llama-3.1-8b-instruct': 5, 'gemma-2-27b-it': 5,
    'llama-3.1-70b-instruct': 4, 'gemma-2-9b-it': 4,
    'mamba-codestral-7b-v0.1': 3, 'mistral-large-latest': 3,
    'gemma-2-2b-it': 3, 'nemotron-4-340b-instruct': 3,
    'phi-3-medium-128k-instruct': 3, 'phi-3-mini-128k-instruct': 3, 'gpt-4o': 2
}

# Load ELO rankings
elo_df = pd.read_csv('scoring/out/elo_winrate/elo_winrate.csv')
elo_dict = dict(zip(elo_df['Model'], elo_df['ELO']))
winrate_dict = dict(zip(elo_df['Model'], elo_df['WinRate']))

# Get mean scores
df = pd.read_csv('output_llms/combined_evaluation_scores.csv')
df = df.rename(columns={'Test Model': 'Model'})
mean_scores = df.groupby('Model')['Score Reference Document'].mean().to_dict()

# Find common models
common = set(metric_pass.keys()) & set(elo_dict.keys())
print(f"Common models: {len(common)}")
print(f"Models: {sorted(common)}")

# Create aligned arrays
models = sorted(common)
compile_arr = np.array([metric_compile[m] for m in models])
pass_arr = np.array([metric_pass[m] for m in models])
elo_arr = np.array([elo_dict[m] for m in models])
winrate_arr = np.array([winrate_dict[m] for m in models])
mean_arr = np.array([mean_scores[m] for m in models])

print("\n" + "=" * 70)
print("  CORRELATION WITH EXECUTION METRICS")
print("=" * 70)

metrics_data = {
    'ELO': elo_arr,
    'Win Rate': winrate_arr,
    'Mean Score': mean_arr
}

print("\n  Spearman Correlation:")
print(f"  {'Metric':<20} {'Pass@1':>12} {'Compile@1':>12}")
print("-" * 50)

results_spearman = {}
for name, vals in metrics_data.items():
    rho_pass, p_pass = spearmanr(vals, pass_arr)
    rho_comp, p_comp = spearmanr(vals, compile_arr)
    sig_pass = '***' if p_pass < 0.001 else '**' if p_pass < 0.01 else '*' if p_pass < 0.05 else ''
    sig_comp = '***' if p_comp < 0.001 else '**' if p_comp < 0.01 else '*' if p_comp < 0.05 else ''
    print(f"  {name:<20} {rho_pass:>10.3f}{sig_pass:<2} {rho_comp:>10.3f}{sig_comp:<2}")
    results_spearman[name] = {'Pass@1': rho_pass, 'Compile@1': rho_comp}

print("\n  Pearson Correlation:")
print(f"  {'Metric':<20} {'Pass@1':>12} {'Compile@1':>12}")
print("-" * 50)

results_pearson = {}
for name, vals in metrics_data.items():
    r_pass, p_pass = pearsonr(vals, pass_arr)
    r_comp, p_comp = pearsonr(vals, compile_arr)
    sig_pass = '***' if p_pass < 0.001 else '**' if p_pass < 0.01 else '*' if p_pass < 0.05 else ''
    sig_comp = '***' if p_comp < 0.001 else '**' if p_comp < 0.01 else '*' if p_comp < 0.05 else ''
    print(f"  {name:<20} {r_pass:>10.3f}{sig_pass:<2} {r_comp:>10.3f}{sig_comp:<2}")
    results_pearson[name] = {'Pass@1': r_pass, 'Compile@1': r_comp}

# Compare with previous results (LLM-Judge direct)
print("\n" + "=" * 70)
print("  COMPARISON: ELO vs Direct LLM-Judge Score")
print("=" * 70)

# From execution_analysis.py we had:
# Pass@1 ↔ LLM-Judge (Score Ref Doc Mean): rho = 0.695
# Compile@1 ↔ LLM-Judge: rho = 0.209

print("\n  Previous results (Mean Score directly):")
print(f"    Pass@1 ↔ Mean Score:    ρ = {results_spearman['Mean Score']['Pass@1']:.3f}")
print(f"    Compile@1 ↔ Mean Score: ρ = {results_spearman['Mean Score']['Compile@1']:.3f}")

print("\n  New results (ELO from Win Rate):")
print(f"    Pass@1 ↔ ELO:    ρ = {results_spearman['ELO']['Pass@1']:.3f}")
print(f"    Compile@1 ↔ ELO: ρ = {results_spearman['ELO']['Compile@1']:.3f}")

# Summary table
print("\n" + "=" * 70)
print("  SUMMARY: Which metric correlates best with Pass@1?")
print("=" * 70)

summary = []
for name, vals in metrics_data.items():
    rho, p = spearmanr(vals, pass_arr)
    summary.append((name, rho, p))

summary.sort(key=lambda x: abs(x[1]), reverse=True)

print(f"\n  {'Metric':<20} {'Spearman ρ':>12} {'p-value':>12}")
print("-" * 50)
for name, rho, p in summary:
    sig = '***' if p < 0.001 else '**' if p < 0.01 else '*' if p < 0.05 else ''
    print(f"  {name:<20} {rho:>12.3f} {p:>11.4f} {sig}")

# Plot scatter
fig, axes = plt.subplots(1, 3, figsize=(15, 5))

for ax, (name, vals) in zip(axes, metrics_data.items()):
    ax.scatter(vals, pass_arr, s=80, alpha=0.7, c='#667eea')
    
    # Add trend line
    z = np.polyfit(vals, pass_arr, 1)
    p = np.poly1d(z)
    x_line = np.linspace(min(vals), max(vals), 100)
    ax.plot(x_line, p(x_line), 'r--', alpha=0.5)
    
    rho, pval = spearmanr(vals, pass_arr)
    ax.set_xlabel(name, fontsize=12)
    ax.set_ylabel('Pass@1', fontsize=12)
    ax.set_title(f'{name} vs Pass@1\nρ = {rho:.3f}', fontsize=11, fontweight='bold')
    ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('scoring/out/elo_winrate/elo_vs_pass1.png', dpi=150)
plt.close()

print("\nSaved plot to: scoring/out/elo_winrate/elo_vs_pass1.png")
