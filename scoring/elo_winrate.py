#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Win Rate Based ELO
==================

Uses actual win rate to compute ELO-like rating.
More stable and interpretable than iterative ELO.

ELO from Win Rate: ELO = 1500 + 400 * log10(WinRate / (1 - WinRate))
"""

import sys
import io
import pandas as pd
import numpy as np
import os
import matplotlib.pyplot as plt

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

def winrate_to_elo(winrate):
    """Convert win rate to ELO-equivalent rating"""
    if winrate <= 0:
        return 800  # floor
    if winrate >= 1:
        return 2200  # ceiling
    return 1500 + 400 * np.log10(winrate / (1 - winrate))

# Load data
df = pd.read_csv('output_llms/combined_evaluation_scores.csv')
df = df.rename(columns={'Test Model': 'Model'})

# Filter base models
def is_base(name):
    n = name.lower()
    return not any(x in n for x in ['_f1', '_f3', '_lora', '_sft', 'pe_', '-f1', '-f3'])

df_base = df[df['Model'].apply(is_base)].copy()

print("=" * 70)
print("  WIN RATE BASED ELO RANKINGS")
print("=" * 70)
print(f"\nBase models: {df_base['Model'].nunique()}")

# Calculate win rates
models = df_base['Model'].unique()
arenas = df_base.groupby(['System', 'Round']).size().index.tolist()

win_counts = {m: {'w': 0, 'l': 0, 'd': 0} for m in models}

for system, rnd in arenas:
    arena = df_base[(df_base['System'] == system) & (df_base['Round'] == rnd)]
    scores = {}
    for m in arena['Model'].unique():
        s = arena[arena['Model'] == m]['Score Reference Document'].values
        if len(s) > 0 and not pd.isna(s[0]):
            scores[m] = s[0]
    
    model_list = list(scores.keys())
    for i in range(len(model_list)):
        for j in range(i+1, len(model_list)):
            ma, mb = model_list[i], model_list[j]
            diff = scores[ma] - scores[mb]
            if abs(diff) < 1.0:  # draw margin
                win_counts[ma]['d'] += 1
                win_counts[mb]['d'] += 1
            elif diff > 0:
                win_counts[ma]['w'] += 1
                win_counts[mb]['l'] += 1
            else:
                win_counts[ma]['l'] += 1
                win_counts[mb]['w'] += 1

# Calculate win rates and ELO
results = []
for m, c in win_counts.items():
    total = c['w'] + c['l'] + c['d']
    if total > 0:
        winrate = (c['w'] + 0.5 * c['d']) / total
        elo = winrate_to_elo(winrate)
        results.append({
            'Model': m,
            'Wins': c['w'],
            'Losses': c['l'],
            'Draws': c['d'],
            'Total': total,
            'WinRate': winrate,
            'ELO': elo
        })

# Sort by win rate
results.sort(key=lambda x: x['WinRate'], reverse=True)

# Display
print(f"\n{'Rank':<5} {'Model':<35} {'W':>5} {'L':>5} {'D':>4} {'Rate':>7} {'ELO':>6}")
print("-" * 75)

for i, r in enumerate(results, 1):
    print(f"{i:<5} {r['Model']:<35} {r['Wins']:>5} {r['Losses']:>5} {r['Draws']:>4} {r['WinRate']:>6.1%} {r['ELO']:>6.0f}")

# Save
OUT_DIR = 'scoring/out/elo_winrate'
os.makedirs(OUT_DIR, exist_ok=True)

result_df = pd.DataFrame([
    {'Rank': i, **r} for i, r in enumerate(results, 1)
])
result_df.to_csv(f'{OUT_DIR}/elo_winrate.csv', index=False, float_format='%.3f')

# Mean score comparison
print("\n" + "=" * 70)
print("  COMPARISON: Win Rate ELO vs Mean Score")
print("=" * 70)

mean_scores = df_base.groupby('Model')['Score Reference Document'].mean()
mean_sorted = mean_scores.sort_values(ascending=False)

print(f"\n{'Model':<35} {'WR Rank':>8} {'Mean Rank':>10} {'Diff':>6}")
print("-" * 65)

wr_ranks = {r['Model']: i for i, r in enumerate(results, 1)}
mean_ranks = {m: i for i, m in enumerate(mean_sorted.index, 1)}

for r in results[:15]:
    m = r['Model']
    wr_r = wr_ranks[m]
    mean_r = mean_ranks.get(m, 99)
    diff = mean_r - wr_r
    print(f"{m:<35} {wr_r:>8} {mean_r:>10} {diff:>+6}")

# Correlation
from scipy.stats import spearmanr
common = [m for m in wr_ranks if m in mean_ranks]
wr_list = [wr_ranks[m] for m in common]
mean_list = [mean_ranks[m] for m in common]
rho, p = spearmanr(wr_list, mean_list)
print(f"\nSpearman correlation (WinRate Rank vs Mean Rank): ρ = {rho:.3f} (p = {p:.4f})")

# Plot
fig, ax = plt.subplots(figsize=(12, 10))

top20 = results[:20]
models_plot = [r['Model'][:28] for r in top20]
elos_plot = [r['ELO'] for r in top20]
winrates_plot = [r['WinRate'] for r in top20]

colors = plt.cm.RdYlGn(np.array(winrates_plot))
bars = ax.barh(range(len(models_plot)), elos_plot, color=colors)
ax.set_yticks(range(len(models_plot)))
ax.set_yticklabels(models_plot)
ax.set_xlabel('ELO (from Win Rate)', fontsize=12)
ax.set_title('ELO Rankings Based on Win Rate\n(Score Reference Document)', fontsize=14, fontweight='bold')

for i, (bar, e, wr) in enumerate(zip(bars, elos_plot, winrates_plot)):
    ax.text(e + 5, i, f'{e:.0f} ({wr:.0%})', va='center', fontsize=9)

ax.axvline(x=1500, color='gray', linestyle='--', alpha=0.5, label='50% WinRate')
ax.legend()
ax.invert_yaxis()
plt.tight_layout()
plt.savefig(f'{OUT_DIR}/elo_winrate.png', dpi=150)
plt.close()

print(f"\nSaved to: {OUT_DIR}/")
