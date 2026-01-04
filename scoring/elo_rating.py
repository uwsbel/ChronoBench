#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ELO Rating System for LLM Benchmark
===================================

Implements ELO rating to rank LLMs based on pairwise comparisons.

ELO Formula:
- Expected score: E_A = 1 / (1 + 10^((R_B - R_A) / 400))
- Rating update: R'_A = R_A + K * (S_A - E_A)
  where S_A = 1 (win), 0.5 (draw), 0 (loss)
"""

import sys
import io
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from collections import defaultdict
import random

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

# ELO parameters
INITIAL_ELO = 1500  # Starting ELO for all models
K_FACTOR = 32       # How much a single match affects rating

def expected_score(rating_a, rating_b):
    """Calculate expected score for player A against player B"""
    return 1 / (1 + 10 ** ((rating_b - rating_a) / 400))

def update_elo(rating_a, rating_b, score_a, k=K_FACTOR):
    """
    Update ELO rating after a match.
    score_a: 1 (A wins), 0.5 (draw), 0 (A loses)
    """
    expected_a = expected_score(rating_a, rating_b)
    new_rating_a = rating_a + k * (score_a - expected_a)
    return new_rating_a

def determine_winner(score_a, score_b, margin=0.5):
    """
    Determine winner based on scores.
    Returns: 1 (A wins), 0.5 (draw), 0 (A loses)
    """
    if abs(score_a - score_b) < margin:
        return 0.5  # Draw
    elif score_a > score_b:
        return 1.0  # A wins
    else:
        return 0.0  # A loses

def calculate_elo_ratings(df, score_column='Score Reference Document', 
                          n_rounds=10, k=K_FACTOR, verbose=True):
    """
    Calculate ELO ratings from benchmark data.
    
    Each (system, round) combination is a "match arena" where models compete.
    """
    # Get unique models
    models = df['Model'].unique()
    
    # Initialize ELO ratings
    elo_ratings = {model: INITIAL_ELO for model in models}
    
    # Track history
    history = defaultdict(list)
    
    # Get unique arenas (system + round combinations)
    arenas = df.groupby(['System', 'Round']).size().index.tolist()
    
    if verbose:
        print(f"  Models: {len(models)}")
        print(f"  Arenas (System x Round): {len(arenas)}")
    
    # Run multiple rounds of tournament for stability
    for tournament_round in range(n_rounds):
        # Shuffle arenas for each round
        random.shuffle(arenas)
        
        for system, round_num in arenas:
            # Get all models that participated in this arena
            arena_data = df[(df['System'] == system) & (df['Round'] == round_num)]
            arena_models = arena_data['Model'].unique()
            
            if len(arena_models) < 2:
                continue
            
            # Get scores for each model
            model_scores = {}
            for model in arena_models:
                score = arena_data[arena_data['Model'] == model][score_column].values
                if len(score) > 0 and not pd.isna(score[0]):
                    model_scores[model] = score[0]
            
            if len(model_scores) < 2:
                continue
            
            # Pairwise comparisons
            model_list = list(model_scores.keys())
            for i in range(len(model_list)):
                for j in range(i + 1, len(model_list)):
                    model_a = model_list[i]
                    model_b = model_list[j]
                    
                    score_a = model_scores[model_a]
                    score_b = model_scores[model_b]
                    
                    # Determine winner
                    result_a = determine_winner(score_a, score_b, margin=1.0)
                    
                    # Update ELO ratings
                    old_elo_a = elo_ratings[model_a]
                    old_elo_b = elo_ratings[model_b]
                    
                    new_elo_a = update_elo(old_elo_a, old_elo_b, result_a, k)
                    new_elo_b = update_elo(old_elo_b, old_elo_a, 1 - result_a, k)
                    
                    elo_ratings[model_a] = new_elo_a
                    elo_ratings[model_b] = new_elo_b
        
        # Record history
        for model in models:
            history[model].append(elo_ratings[model])
    
    return elo_ratings, history

def run_elo_analysis():
    """Main function to run ELO analysis"""
    
    print("=" * 80)
    print("  ELO RATING ANALYSIS FOR SIMBENCH")
    print("=" * 80)
    
    # Load data
    try:
        df = pd.read_csv('output_llms/combined_evaluation_scores.csv')
        print(f"\n  Loaded {len(df)} records")
    except FileNotFoundError:
        df = pd.read_csv('D:/SimBench/output_llms/combined_evaluation_scores.csv')
        print(f"\n  Loaded {len(df)} records")
    
    # Rename column if needed
    if 'Test Model' in df.columns:
        df = df.rename(columns={'Test Model': 'Model'})
    
    # Filter base models only
    def is_base_model(name):
        name_lower = name.lower()
        if any(x in name_lower for x in ['_f1', '_f3', '_lora', '_sft', 'pe_']):
            return False
        return True
    
    df_base = df[df['Model'].apply(is_base_model)].copy()
    print(f"  Base models only: {len(df_base)} records")
    print(f"  Unique models: {df_base['Model'].nunique()}")
    
    # Calculate ELO for different metrics
    metrics = ['Score Document', 'Score Reference', 'Score Reference Document']
    
    results = {}
    
    for metric in metrics:
        print(f"\n" + "-" * 60)
        print(f"  Calculating ELO for: {metric}")
        print("-" * 60)
        
        # Set random seed for reproducibility
        random.seed(42)
        
        elo_ratings, history = calculate_elo_ratings(
            df_base, 
            score_column=metric,
            n_rounds=20,  # More rounds for stability
            k=32,
            verbose=True
        )
        
        results[metric] = elo_ratings
        
        # Sort and display
        sorted_ratings = sorted(elo_ratings.items(), key=lambda x: x[1], reverse=True)
        
        print(f"\n  Top 10 models by ELO ({metric}):")
        for i, (model, elo) in enumerate(sorted_ratings[:10], 1):
            print(f"    {i:2d}. {model:35s} ELO: {elo:.0f}")
    
    # Create combined ranking
    print("\n" + "=" * 80)
    print("  COMBINED ELO RANKING")
    print("=" * 80)
    
    # Average ELO across all metrics
    all_models = set()
    for ratings in results.values():
        all_models.update(ratings.keys())
    
    combined_elo = {}
    for model in all_models:
        elos = [results[m].get(model, INITIAL_ELO) for m in metrics]
        combined_elo[model] = np.mean(elos)
    
    sorted_combined = sorted(combined_elo.items(), key=lambda x: x[1], reverse=True)
    
    print("\n  Final ELO Rankings (averaged across metrics):")
    print("-" * 60)
    print(f"  {'Rank':<5} {'Model':<40} {'ELO':>8}")
    print("-" * 60)
    
    for i, (model, elo) in enumerate(sorted_combined, 1):
        print(f"  {i:<5} {model:<40} {elo:>8.0f}")
    
    # Save results
    OUT_DIR = 'scoring/out/elo'
    import os
    os.makedirs(OUT_DIR, exist_ok=True)
    
    # Create DataFrame
    elo_df = pd.DataFrame([
        {
            'Rank': i,
            'Model': model,
            'ELO_Combined': combined_elo[model],
            'ELO_Document': results['Score Document'].get(model, INITIAL_ELO),
            'ELO_Reference': results['Score Reference'].get(model, INITIAL_ELO),
            'ELO_RefDoc': results['Score Reference Document'].get(model, INITIAL_ELO),
        }
        for i, (model, _) in enumerate(sorted_combined, 1)
    ])
    elo_df.to_csv(f'{OUT_DIR}/elo_rankings.csv', index=False, float_format='%.1f')
    
    # Generate visualization
    plt.figure(figsize=(14, 10))
    
    # Bar chart of ELO ratings
    models = [x[0][:25] for x in sorted_combined[:20]]  # Top 20, truncated names
    elos = [x[1] for x in sorted_combined[:20]]
    
    colors = plt.cm.RdYlGn(np.linspace(0.8, 0.2, len(models)))
    
    bars = plt.barh(range(len(models)), elos, color=colors)
    plt.yticks(range(len(models)), models)
    plt.xlabel('ELO Rating', fontsize=12)
    plt.title('LLM ELO Rankings (SimBench)', fontsize=14, fontweight='bold')
    
    # Add ELO values on bars
    for i, (bar, elo) in enumerate(zip(bars, elos)):
        plt.text(elo + 5, i, f'{elo:.0f}', va='center', fontsize=10)
    
    plt.axvline(x=INITIAL_ELO, color='gray', linestyle='--', alpha=0.5, label=f'Initial ELO ({INITIAL_ELO})')
    plt.legend()
    
    plt.gca().invert_yaxis()
    plt.tight_layout()
    plt.savefig(f'{OUT_DIR}/elo_rankings.png', dpi=150, bbox_inches='tight')
    plt.close()
    
    # Win rate analysis
    print("\n" + "=" * 80)
    print("  WIN RATE ANALYSIS")
    print("=" * 80)
    
    # Calculate win rates
    win_counts = defaultdict(lambda: {'wins': 0, 'losses': 0, 'draws': 0})
    
    arenas = df_base.groupby(['System', 'Round']).size().index.tolist()
    
    for system, round_num in arenas:
        arena_data = df_base[(df_base['System'] == system) & (df_base['Round'] == round_num)]
        arena_models = arena_data['Model'].unique()
        
        if len(arena_models) < 2:
            continue
        
        model_scores = {}
        for model in arena_models:
            score = arena_data[arena_data['Model'] == model]['Score Reference Document'].values
            if len(score) > 0 and not pd.isna(score[0]):
                model_scores[model] = score[0]
        
        model_list = list(model_scores.keys())
        for i in range(len(model_list)):
            for j in range(i + 1, len(model_list)):
                model_a = model_list[i]
                model_b = model_list[j]
                
                score_a = model_scores[model_a]
                score_b = model_scores[model_b]
                
                if abs(score_a - score_b) < 1.0:
                    win_counts[model_a]['draws'] += 1
                    win_counts[model_b]['draws'] += 1
                elif score_a > score_b:
                    win_counts[model_a]['wins'] += 1
                    win_counts[model_b]['losses'] += 1
                else:
                    win_counts[model_a]['losses'] += 1
                    win_counts[model_b]['wins'] += 1
    
    # Calculate win rates
    win_rate_data = []
    for model, counts in win_counts.items():
        total = counts['wins'] + counts['losses'] + counts['draws']
        if total > 0:
            win_rate = (counts['wins'] + 0.5 * counts['draws']) / total
            win_rate_data.append({
                'Model': model,
                'Wins': counts['wins'],
                'Losses': counts['losses'],
                'Draws': counts['draws'],
                'Total': total,
                'Win Rate': win_rate
            })
    
    win_df = pd.DataFrame(win_rate_data)
    win_df = win_df.sort_values('Win Rate', ascending=False)
    win_df.to_csv(f'{OUT_DIR}/win_rates.csv', index=False, float_format='%.3f')
    
    print("\n  Top 10 by Win Rate:")
    print("-" * 70)
    print(f"  {'Model':<35} {'W':>5} {'L':>5} {'D':>5} {'Rate':>8}")
    print("-" * 70)
    for _, row in win_df.head(10).iterrows():
        print(f"  {row['Model']:<35} {row['Wins']:>5.0f} {row['Losses']:>5.0f} {row['Draws']:>5.0f} {row['Win Rate']:>8.1%}")
    
    # Compare ELO with other rankings
    print("\n" + "=" * 80)
    print("  ELO vs MEAN SCORE COMPARISON")
    print("=" * 80)
    
    # Calculate mean scores
    mean_scores = df_base.groupby('Model')['Score Reference Document'].mean()
    
    # Create comparison
    comparison = []
    for model in combined_elo.keys():
        if model in mean_scores.index:
            comparison.append({
                'Model': model,
                'ELO': combined_elo[model],
                'Mean Score': mean_scores[model],
                'ELO Rank': sorted_combined.index((model, combined_elo[model])) + 1
            })
    
    comp_df = pd.DataFrame(comparison)
    comp_df['Mean Rank'] = comp_df['Mean Score'].rank(ascending=False)
    comp_df['Rank Diff'] = abs(comp_df['ELO Rank'] - comp_df['Mean Rank'])
    comp_df = comp_df.sort_values('Rank Diff', ascending=False)
    
    print("\n  Models with largest ranking differences (ELO vs Mean):")
    print("-" * 70)
    for _, row in comp_df.head(5).iterrows():
        print(f"  {row['Model']:<35} ELO Rank: {row['ELO Rank']:.0f}, Mean Rank: {row['Mean Rank']:.0f}")
    
    from scipy.stats import spearmanr
    rho, p = spearmanr(comp_df['ELO'], comp_df['Mean Score'])
    print(f"\n  Correlation (ELO vs Mean Score): ρ = {rho:.3f} (p = {p:.4f})")
    
    print(f"\n  Results saved to: {OUT_DIR}/")
    print("    - elo_rankings.csv")
    print("    - elo_rankings.png")
    print("    - win_rates.csv")
    
    print("\n" + "=" * 80)
    print("  ELO ANALYSIS COMPLETE")
    print("=" * 80)
    
    return elo_df, win_df

if __name__ == '__main__':
    elo_df, win_df = run_elo_analysis()
