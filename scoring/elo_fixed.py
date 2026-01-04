#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Fixed ELO Rating System
=======================

Key fix: Update ELO for both players using ORIGINAL ratings, not updated ones.
"""

import sys
import io
import pandas as pd
import numpy as np
import random
import os
import matplotlib.pyplot as plt

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

INITIAL_ELO = 1500
K_FACTOR = 32

def expected_score(ra, rb):
    return 1 / (1 + 10 ** ((rb - ra) / 400))

def update_elo_pair(ra, rb, result_a, k=K_FACTOR):
    """
    Update ELO for BOTH players at once using ORIGINAL ratings.
    result_a: 1 (A wins), 0.5 (draw), 0 (A loses)
    Returns: (new_ra, new_rb)
    """
    ea = expected_score(ra, rb)
    eb = 1 - ea
    
    new_ra = ra + k * (result_a - ea)
    new_rb = rb + k * ((1 - result_a) - eb)
    
    return new_ra, new_rb

def determine_winner(sa, sb, margin=1.0):
    if abs(sa - sb) < margin:
        return 0.5
    return 1.0 if sa > sb else 0.0

# Test the fixed version
print("=" * 70)
print("  TESTING FIXED ELO")
print("=" * 70)

# Test: 50 wins then 50 losses should return to ~1500
elo_a, elo_b = 1500, 1500
for _ in range(50):
    elo_a, elo_b = update_elo_pair(elo_a, elo_b, 1.0)
for _ in range(50):
    elo_a, elo_b = update_elo_pair(elo_a, elo_b, 0.0)
print(f"\nTest 1 (sequential 50W-50L): A={elo_a:.0f}, B={elo_b:.0f}")

# Test: alternating wins/losses
elo_a, elo_b = 1500, 1500
for _ in range(50):
    elo_a, elo_b = update_elo_pair(elo_a, elo_b, 1.0)
    elo_a, elo_b = update_elo_pair(elo_a, elo_b, 0.0)
print(f"Test 2 (alternating W-L): A={elo_a:.0f}, B={elo_b:.0f}")

# Test: random 50% win rate
random.seed(42)
elo_a, elo_b = 1500, 1500
results = [1.0] * 50 + [0.0] * 50
random.shuffle(results)
for r in results:
    elo_a, elo_b = update_elo_pair(elo_a, elo_b, r)
print(f"Test 3 (shuffled 50W-50L): A={elo_a:.0f}, B={elo_b:.0f}")

print("\n" + "=" * 70)
print("  RUNNING FIXED ELO ON ACTUAL DATA")
print("=" * 70)

# Load data
df = pd.read_csv('output_llms/combined_evaluation_scores.csv')
df = df.rename(columns={'Test Model': 'Model'})

# Filter base models
def is_base(name):
    n = name.lower()
    return not any(x in n for x in ['_f1', '_f3', '_lora', '_sft', 'pe_', '-f1', '-f3'])

df_base = df[df['Model'].apply(is_base)].copy()
print(f"\nBase models: {df_base['Model'].nunique()}")

# Initialize ELO
models = df_base['Model'].unique()
elo = {m: INITIAL_ELO for m in models}

# Get arenas
arenas = df_base.groupby(['System', 'Round']).size().index.tolist()
print(f"Arenas: {len(arenas)}")

# Run tournament
random.seed(42)
for tournament_round in range(10):  # 10 rounds for stability
    random.shuffle(arenas)
    
    for system, rnd in arenas:
        arena = df_base[(df_base['System'] == system) & (df_base['Round'] == rnd)]
        
        # Get scores
        scores = {}
        for m in arena['Model'].unique():
            s = arena[arena['Model'] == m]['Score Reference Document'].values
            if len(s) > 0 and not pd.isna(s[0]):
                scores[m] = s[0]
        
        # Pairwise comparisons - collect all updates first
        updates = []
        model_list = list(scores.keys())
        for i in range(len(model_list)):
            for j in range(i+1, len(model_list)):
                ma, mb = model_list[i], model_list[j]
                result = determine_winner(scores[ma], scores[mb])
                updates.append((ma, mb, result))
        
        # Apply updates using CURRENT elo values
        for ma, mb, result in updates:
            new_a, new_b = update_elo_pair(elo[ma], elo[mb], result)
            elo[ma] = new_a
            elo[mb] = new_b

# Sort and display
sorted_elo = sorted(elo.items(), key=lambda x: x[1], reverse=True)

print("\n" + "=" * 70)
print("  FIXED ELO RANKINGS")
print("=" * 70)

print(f"\n{'Rank':<5} {'Model':<40} {'ELO':>8}")
print("-" * 55)
for i, (m, e) in enumerate(sorted_elo, 1):
    flag = " <-- GEMMA" if 'gemma-2-27b' in m else ""
    print(f"{i:<5} {m:<40} {e:>8.0f}{flag}")

# Verify with win rate
print("\n" + "=" * 70)
print("  VERIFICATION: Win Rate vs ELO")
print("=" * 70)

# Calculate win rates again
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
            if abs(diff) < 1.0:
                win_counts[ma]['d'] += 1
                win_counts[mb]['d'] += 1
            elif diff > 0:
                win_counts[ma]['w'] += 1
                win_counts[mb]['l'] += 1
            else:
                win_counts[ma]['l'] += 1
                win_counts[mb]['w'] += 1

print(f"\n{'Model':<35} {'ELO':>8} {'Win%':>8}")
print("-" * 55)
for m, e in sorted_elo[:15]:
    c = win_counts[m]
    total = c['w'] + c['l'] + c['d']
    rate = (c['w'] + 0.5 * c['d']) / total if total > 0 else 0
    print(f"{m:<35} {e:>8.0f} {rate:>7.1%}")

# Save results
OUT_DIR = 'scoring/out/elo_fixed'
os.makedirs(OUT_DIR, exist_ok=True)

pd.DataFrame([
    {'Rank': i, 'Model': m, 'ELO': e}
    for i, (m, e) in enumerate(sorted_elo, 1)
]).to_csv(f'{OUT_DIR}/elo_rankings.csv', index=False)

# Plot
plt.figure(figsize=(12, 10))
top20 = sorted_elo[:20]
models_plot = [x[0][:30] for x in top20]
elos_plot = [x[1] for x in top20]

colors = plt.cm.RdYlGn(np.linspace(0.8, 0.2, len(models_plot)))
bars = plt.barh(range(len(models_plot)), elos_plot, color=colors)
plt.yticks(range(len(models_plot)), models_plot)
plt.xlabel('ELO Rating', fontsize=12)
plt.title('Fixed ELO Rankings (Base Models)', fontsize=14, fontweight='bold')

for i, (bar, e) in enumerate(zip(bars, elos_plot)):
    plt.text(e + 5, i, f'{e:.0f}', va='center', fontsize=9)

plt.axvline(x=INITIAL_ELO, color='gray', linestyle='--', alpha=0.5)
plt.gca().invert_yaxis()
plt.tight_layout()
plt.savefig(f'{OUT_DIR}/elo_rankings.png', dpi=150)
plt.close()

print(f"\nSaved to: {OUT_DIR}/")
