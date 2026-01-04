#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ELO Rating - Base Models Only"""

import sys
import io
import pandas as pd
import numpy as np
import random
import os
from collections import defaultdict
import matplotlib.pyplot as plt

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

INITIAL_ELO = 1500
K_FACTOR = 32

def expected_score(ra, rb):
    return 1 / (1 + 10 ** ((rb - ra) / 400))

def update_elo(ra, rb, sa, k=K_FACTOR):
    ea = expected_score(ra, rb)
    return ra + k * (sa - ea)

def determine_winner(sa, sb, margin=1.0):
    if abs(sa - sb) < margin:
        return 0.5
    return 1.0 if sa > sb else 0.0

# Load data
df = pd.read_csv('output_llms/combined_evaluation_scores.csv')
df = df.rename(columns={'Test Model': 'Model'})

# STRICT Base models only
def is_strict_base(name):
    n = name.lower()
    return not any(x in n for x in ['_f1', '_f3', '_lora', '_sft', 'pe_', '-f1', '-f3', '-lora', '-sft'])

df_base = df[df['Model'].apply(is_strict_base)].copy()
print(f"Base models (strict): {df_base['Model'].nunique()}")
print(f"Models: {sorted(df_base['Model'].unique())}")

# Run ELO
models = df_base['Model'].unique()
elo = {m: INITIAL_ELO for m in models}
arenas = df_base.groupby(['System', 'Round']).size().index.tolist()

random.seed(42)
for _ in range(20):
    random.shuffle(arenas)
    for system, rnd in arenas:
        arena = df_base[(df_base['System'] == system) & (df_base['Round'] == rnd)]
        scores = {}
        for m in arena['Model'].unique():
            s = arena[arena['Model'] == m]['Score Reference Document'].values
            if len(s) > 0 and not pd.isna(s[0]):
                scores[m] = s[0]
        
        ml = list(scores.keys())
        for i in range(len(ml)):
            for j in range(i+1, len(ml)):
                ma, mb = ml[i], ml[j]
                result = determine_winner(scores[ma], scores[mb])
                elo[ma] = update_elo(elo[ma], elo[mb], result)
                elo[mb] = update_elo(elo[mb], elo[ma], 1-result)

# Sort and print
sorted_elo = sorted(elo.items(), key=lambda x: x[1], reverse=True)

print()
print("=" * 70)
print("  ELO RANKINGS - BASE MODELS ONLY (No Fine-tuning)")
print("=" * 70)

for i, (m, e) in enumerate(sorted_elo, 1):
    tier = "S" if e > 1700 else "A" if e > 1550 else "B" if e > 1400 else "C" if e > 1250 else "D"
    print(f"  {i:2d}. [{tier}] {m:40s} ELO: {e:.0f}")

# Win rate analysis
print()
print("=" * 70)
print("  WIN RATE ANALYSIS")
print("=" * 70)

wins = defaultdict(lambda: {'w': 0, 'l': 0, 'd': 0})

for system, rnd in arenas:
    arena = df_base[(df_base['System'] == system) & (df_base['Round'] == rnd)]
    scores = {}
    for m in arena['Model'].unique():
        s = arena[arena['Model'] == m]['Score Reference Document'].values
        if len(s) > 0 and not pd.isna(s[0]):
            scores[m] = s[0]
    
    ml = list(scores.keys())
    for i in range(len(ml)):
        for j in range(i+1, len(ml)):
            ma, mb = ml[i], ml[j]
            diff = scores[ma] - scores[mb]
            if abs(diff) < 1.0:
                wins[ma]['d'] += 1
                wins[mb]['d'] += 1
            elif diff > 0:
                wins[ma]['w'] += 1
                wins[mb]['l'] += 1
            else:
                wins[ma]['l'] += 1
                wins[mb]['w'] += 1

# Calculate win rates
win_rates = []
for m, c in wins.items():
    total = c['w'] + c['l'] + c['d']
    rate = (c['w'] + 0.5 * c['d']) / total if total > 0 else 0
    win_rates.append((m, rate, c['w'], c['l'], c['d']))

win_rates.sort(key=lambda x: x[1], reverse=True)

print()
print(f"  {'Model':<40} {'W':>5} {'L':>5} {'D':>5} {'Rate':>8}")
print("-" * 70)
for m, rate, w, l, d in win_rates[:15]:
    print(f"  {m:<40} {w:>5} {l:>5} {d:>5} {rate:>7.1%}")

# Save results
OUT_DIR = 'scoring/out/elo_base'
os.makedirs(OUT_DIR, exist_ok=True)

elo_df = pd.DataFrame([
    {'Rank': i, 'Model': m, 'ELO': e}
    for i, (m, e) in enumerate(sorted_elo, 1)
])
elo_df.to_csv(f'{OUT_DIR}/elo_base_rankings.csv', index=False)

# Plot
plt.figure(figsize=(12, 10))
models_plot = [x[0][:30] for x in sorted_elo[:20]]
elos_plot = [x[1] for x in sorted_elo[:20]]

colors = plt.cm.RdYlGn(np.linspace(0.8, 0.2, len(models_plot)))
bars = plt.barh(range(len(models_plot)), elos_plot, color=colors)
plt.yticks(range(len(models_plot)), models_plot)
plt.xlabel('ELO Rating', fontsize=12)
plt.title('ELO Rankings (Base Models Only)', fontsize=14, fontweight='bold')

for i, (bar, e) in enumerate(zip(bars, elos_plot)):
    plt.text(e + 5, i, f'{e:.0f}', va='center', fontsize=9)

plt.axvline(x=INITIAL_ELO, color='gray', linestyle='--', alpha=0.5, label=f'Initial ({INITIAL_ELO})')
plt.legend()
plt.gca().invert_yaxis()
plt.tight_layout()
plt.savefig(f'{OUT_DIR}/elo_base_rankings.png', dpi=150)
plt.close()

print(f"\nSaved to: {OUT_DIR}/")
