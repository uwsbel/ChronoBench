#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ELO Rating System - Multi-Metric Version
=========================================

Calculates ELO based on multiple metrics and provides different perspectives.
"""

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

def determine_winner(sa, sb, margin_pct=0.02):
    """Winner determined by relative margin"""
    if sa == 0 and sb == 0:
        return 0.5
    avg = (abs(sa) + abs(sb)) / 2
    margin = max(avg * margin_pct, 0.5)  # At least 0.5 absolute or 2% relative
    if abs(sa - sb) < margin:
        return 0.5
    return 1.0 if sa > sb else 0.0

def run_elo_for_metric(df, metric_col, margin=0.02):
    """Run ELO tournament for a single metric"""
    models = df['Model'].unique()
    elo = {m: INITIAL_ELO for m in models}
    arenas = df.groupby(['System', 'Round']).size().index.tolist()
    
    random.seed(42)
    for _ in range(20):
        random.shuffle(arenas)
        for system, rnd in arenas:
            arena = df[(df['System'] == system) & (df['Round'] == rnd)]
            scores = {}
            for m in arena['Model'].unique():
                s = arena[arena['Model'] == m][metric_col].values
                if len(s) > 0 and not pd.isna(s[0]):
                    scores[m] = s[0]
            
            ml = list(scores.keys())
            for i in range(len(ml)):
                for j in range(i+1, len(ml)):
                    ma, mb = ml[i], ml[j]
                    result = determine_winner(scores[ma], scores[mb], margin)
                    elo[ma] = update_elo(elo[ma], elo[mb], result)
                    elo[mb] = update_elo(elo[mb], elo[ma], 1-result)
    
    return elo

def main():
    print("=" * 80)
    print("  MULTI-METRIC ELO ANALYSIS")
    print("=" * 80)
    
    # Load combined metrics
    try:
        df = pd.read_csv('scoring/out/all_metrics_merged.csv')
    except:
        df = pd.read_csv('D:/SimBench/scoring/out/all_metrics_merged.csv')
    
    # Standardize column names
    col_map = {
        'model': 'Model', 'Test Model': 'Model',
        'system': 'System', 'round': 'Round',
        'score_document': 'Score Document',
        'score_reference': 'Score Reference', 
        'score_reference_document': 'Score Reference Document',
        'codebleu': 'CodeBLEU',
        'rougeL': 'ROUGE-L', 'rouge1': 'ROUGE-1', 'rouge2': 'ROUGE-2'
    }
    df = df.rename(columns={k: v for k, v in col_map.items() if k in df.columns})
    
    print(f"\nLoaded {len(df)} records")
    print(f"Columns: {df.columns.tolist()}")
    
    # Filter base models only
    def is_base(name):
        n = name.lower()
        return not any(x in n for x in ['_f1', '_f3', '_lora', '_sft', 'pe_', '-f1', '-f3'])
    
    df = df[df['Model'].apply(is_base)].copy()
    print(f"Base models: {df['Model'].nunique()}")
    
    # Define metrics to use for ELO
    llm_judge_metrics = ['Score Document', 'Score Reference', 'Score Reference Document']
    code_metrics = ['CodeBLEU', 'ROUGE-L', 'ROUGE-1', 'ROUGE-2']
    
    # Check which metrics exist
    available_llm = [m for m in llm_judge_metrics if m in df.columns]
    available_code = [m for m in code_metrics if m in df.columns]
    
    print(f"\nAvailable LLM-Judge metrics: {available_llm}")
    print(f"Available Code metrics: {available_code}")
    
    all_elos = {}
    
    # Calculate ELO for each metric
    print("\n" + "=" * 80)
    print("  ELO BY METRIC")
    print("=" * 80)
    
    for metric in available_llm + available_code:
        print(f"\n--- {metric} ---")
        elo = run_elo_for_metric(df, metric)
        all_elos[metric] = elo
        
        # Top 5
        sorted_elo = sorted(elo.items(), key=lambda x: x[1], reverse=True)[:5]
        for i, (m, e) in enumerate(sorted_elo, 1):
            print(f"  {i}. {m[:35]:35s} {e:.0f}")
    
    # Combined ELO (average across all metrics)
    print("\n" + "=" * 80)
    print("  COMBINED ELO (All Metrics Averaged)")
    print("=" * 80)
    
    all_models = set()
    for elos in all_elos.values():
        all_models.update(elos.keys())
    
    combined_elo = {}
    for model in all_models:
        elos = [all_elos[m].get(model, INITIAL_ELO) for m in all_elos.keys()]
        combined_elo[model] = np.mean(elos)
    
    sorted_combined = sorted(combined_elo.items(), key=lambda x: x[1], reverse=True)
    
    print(f"\n{'Rank':<5} {'Model':<40} {'ELO':>8}")
    print("-" * 55)
    for i, (m, e) in enumerate(sorted_combined, 1):
        print(f"{i:<5} {m:<40} {e:>8.0f}")
    
    # Compare rankings by metric type
    print("\n" + "=" * 80)
    print("  LLM-JUDGE vs CODE METRICS ELO COMPARISON")
    print("=" * 80)
    
    # LLM-Judge combined
    llm_combined = {}
    for model in all_models:
        elos = [all_elos[m].get(model, INITIAL_ELO) for m in available_llm if m in all_elos]
        llm_combined[model] = np.mean(elos) if elos else INITIAL_ELO
    
    # Code metrics combined
    code_combined = {}
    for model in all_models:
        elos = [all_elos[m].get(model, INITIAL_ELO) for m in available_code if m in all_elos]
        code_combined[model] = np.mean(elos) if elos else INITIAL_ELO
    
    # Find disagreements
    llm_sorted = sorted(llm_combined.items(), key=lambda x: x[1], reverse=True)
    code_sorted = sorted(code_combined.items(), key=lambda x: x[1], reverse=True)
    
    llm_ranks = {m: i for i, (m, _) in enumerate(llm_sorted, 1)}
    code_ranks = {m: i for i, (m, _) in enumerate(code_sorted, 1)}
    
    print(f"\n{'Model':<35} {'LLM-J Rank':>12} {'Code Rank':>12} {'Diff':>8}")
    print("-" * 70)
    
    disagreements = []
    for model in all_models:
        llm_r = llm_ranks.get(model, 999)
        code_r = code_ranks.get(model, 999)
        diff = abs(llm_r - code_r)
        disagreements.append((model, llm_r, code_r, diff))
    
    disagreements.sort(key=lambda x: x[3], reverse=True)
    
    for model, llm_r, code_r, diff in disagreements[:10]:
        direction = "↑" if code_r < llm_r else "↓" if code_r > llm_r else "="
        print(f"{model[:35]:<35} {llm_r:>12} {code_r:>12} {diff:>6} {direction}")
    
    # Correlation between metric ELOs
    print("\n" + "=" * 80)
    print("  ELO CORRELATION BETWEEN METRICS")
    print("=" * 80)
    
    from scipy.stats import spearmanr
    
    metrics_list = list(all_elos.keys())
    print(f"\n{'':20}", end="")
    for m in metrics_list:
        print(f"{m[:8]:>10}", end="")
    print()
    
    for m1 in metrics_list:
        print(f"{m1[:20]:<20}", end="")
        for m2 in metrics_list:
            models_common = set(all_elos[m1].keys()) & set(all_elos[m2].keys())
            if models_common:
                vals1 = [all_elos[m1][m] for m in models_common]
                vals2 = [all_elos[m2][m] for m in models_common]
                rho, _ = spearmanr(vals1, vals2)
                print(f"{rho:>10.2f}", end="")
            else:
                print(f"{'N/A':>10}", end="")
        print()
    
    # Save results
    OUT_DIR = 'scoring/out/elo_multi'
    os.makedirs(OUT_DIR, exist_ok=True)
    
    # Save combined ELO
    pd.DataFrame([
        {'Model': m, 'ELO_Combined': combined_elo[m], 
         'ELO_LLM_Judge': llm_combined.get(m, INITIAL_ELO),
         'ELO_Code_Metrics': code_combined.get(m, INITIAL_ELO)}
        for m, _ in sorted_combined
    ]).to_csv(f'{OUT_DIR}/elo_multi_metric.csv', index=False)
    
    # Save per-metric ELO
    per_metric_df = pd.DataFrame([
        {'Model': m, **{metric: all_elos[metric].get(m, INITIAL_ELO) for metric in all_elos}}
        for m in all_models
    ])
    per_metric_df.to_csv(f'{OUT_DIR}/elo_per_metric.csv', index=False)
    
    # Plot
    fig, axes = plt.subplots(1, 2, figsize=(16, 8))
    
    # LLM-Judge ELO
    ax1 = axes[0]
    top15_llm = sorted(llm_combined.items(), key=lambda x: x[1], reverse=True)[:15]
    models_llm = [x[0][:25] for x in top15_llm]
    elos_llm = [x[1] for x in top15_llm]
    colors_llm = plt.cm.Blues(np.linspace(0.8, 0.3, len(models_llm)))
    ax1.barh(range(len(models_llm)), elos_llm, color=colors_llm)
    ax1.set_yticks(range(len(models_llm)))
    ax1.set_yticklabels(models_llm)
    ax1.axvline(x=INITIAL_ELO, color='gray', linestyle='--', alpha=0.5)
    ax1.set_xlabel('ELO')
    ax1.set_title('ELO by LLM-Judge Metrics', fontweight='bold')
    ax1.invert_yaxis()
    
    # Code Metrics ELO
    ax2 = axes[1]
    top15_code = sorted(code_combined.items(), key=lambda x: x[1], reverse=True)[:15]
    models_code = [x[0][:25] for x in top15_code]
    elos_code = [x[1] for x in top15_code]
    colors_code = plt.cm.Greens(np.linspace(0.8, 0.3, len(models_code)))
    ax2.barh(range(len(models_code)), elos_code, color=colors_code)
    ax2.set_yticks(range(len(models_code)))
    ax2.set_yticklabels(models_code)
    ax2.axvline(x=INITIAL_ELO, color='gray', linestyle='--', alpha=0.5)
    ax2.set_xlabel('ELO')
    ax2.set_title('ELO by Code Similarity Metrics', fontweight='bold')
    ax2.invert_yaxis()
    
    plt.tight_layout()
    plt.savefig(f'{OUT_DIR}/elo_comparison.png', dpi=150)
    plt.close()
    
    print(f"\nSaved to: {OUT_DIR}/")
    print("  - elo_multi_metric.csv")
    print("  - elo_per_metric.csv")
    print("  - elo_comparison.png")
    
    print("\n" + "=" * 80)
    print("  COMPLETE")
    print("=" * 80)

if __name__ == '__main__':
    main()
