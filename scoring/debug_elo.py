#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Debug ELO calculation"""

import sys
import io
import pandas as pd
import numpy as np

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

df = pd.read_csv('output_llms/combined_evaluation_scores.csv')
df = df.rename(columns={'Test Model': 'Model'})

# Filter base models
def is_base(name):
    n = name.lower()
    return not any(x in n for x in ['_f1', '_f3', '_lora', '_sft', 'pe_', '-f1', '-f3'])

df_base = df[df['Model'].apply(is_base)].copy()
print(f"Base models: {df_base['Model'].nunique()}")
print(f"Models: {sorted(df_base['Model'].unique())}")

# Mean score ranking
print("\n" + "=" * 70)
print("  MEAN SCORE RANKING (Base Models)")
print("=" * 70)

mean_scores = df_base.groupby('Model')['Score Reference Document'].mean().sort_values(ascending=False)
for i, (m, s) in enumerate(mean_scores.items(), 1):
    flag = " <--" if 'gemma-2-27b' in m else ""
    print(f"  {i:2d}. {m:40s} {s:.2f}{flag}")

# Head-to-head analysis
print("\n" + "=" * 70)
print("  HEAD-TO-HEAD ANALYSIS")
print("=" * 70)

arenas = df_base.groupby(['System', 'Round']).size().index.tolist()

# Calculate win/loss for each model
records = {}
for model in df_base['Model'].unique():
    records[model] = {'w': 0, 'l': 0, 'd': 0}

for system, rnd in arenas:
    arena = df_base[(df_base['System'] == system) & (df_base['Round'] == rnd)]
    
    scores = {}
    for model in arena['Model'].unique():
        s = arena[arena['Model'] == model]['Score Reference Document'].values
        if len(s) > 0 and not pd.isna(s[0]):
            scores[model] = s[0]
    
    models = list(scores.keys())
    for i in range(len(models)):
        for j in range(i+1, len(models)):
            ma, mb = models[i], models[j]
            diff = scores[ma] - scores[mb]
            
            if abs(diff) < 1.0:
                records[ma]['d'] += 1
                records[mb]['d'] += 1
            elif diff > 0:
                records[ma]['w'] += 1
                records[mb]['l'] += 1
            else:
                records[ma]['l'] += 1
                records[mb]['w'] += 1

# Calculate win rates and sort
win_rates = []
for model, rec in records.items():
    total = rec['w'] + rec['l'] + rec['d']
    if total > 0:
        rate = (rec['w'] + 0.5 * rec['d']) / total
        win_rates.append((model, rate, rec['w'], rec['l'], rec['d'], total))

win_rates.sort(key=lambda x: x[1], reverse=True)

print(f"\n  {'Model':<40} {'W':>5} {'L':>5} {'D':>5} {'Total':>6} {'Rate':>7}")
print("-" * 75)
for model, rate, w, l, d, total in win_rates:
    flag = " <--" if 'gemma-2-27b' in model else ""
    print(f"  {model:<40} {w:>5} {l:>5} {d:>5} {total:>6} {rate:>6.1%}{flag}")

# Check gemma specifically
print("\n" + "=" * 70)
print("  GEMMA-2-27B-IT DETAILED CHECK")
print("=" * 70)

gemma = 'gemma-2-27b-it'
gemma_rec = records.get(gemma, {'w': 0, 'l': 0, 'd': 0})
print(f"\n  Wins: {gemma_rec['w']}, Losses: {gemma_rec['l']}, Draws: {gemma_rec['d']}")

# Check how many arenas gemma participated in
gemma_arenas = df_base[df_base['Model'] == gemma].groupby(['System', 'Round']).size()
print(f"  Arenas participated: {len(gemma_arenas)}")

# Sample some head-to-head results
print("\n  Sample matchups against top models:")
top_models = [m for m, _, _, _, _, _ in win_rates[:5] if m != gemma]

for system, rnd in arenas[:5]:  # Check first 5 arenas
    arena = df_base[(df_base['System'] == system) & (df_base['Round'] == rnd)]
    gemma_score = arena[arena['Model'] == gemma]['Score Reference Document'].values
    
    if len(gemma_score) == 0:
        continue
    gemma_score = gemma_score[0]
    
    print(f"\n  Arena: {system} / Round {rnd}")
    print(f"    gemma-2-27b-it: {gemma_score}")
    
    for model in arena['Model'].unique():
        if model == gemma:
            continue
        score = arena[arena['Model'] == model]['Score Reference Document'].values[0]
        result = "WIN" if gemma_score > score + 1 else "LOSS" if gemma_score < score - 1 else "DRAW"
        print(f"    vs {model[:30]:30s}: {score:5.1f} -> {result}")
