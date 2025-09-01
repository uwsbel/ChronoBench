#!/usr/bin/env python3
"""
Generate all_scores_ranked.csv for each JLLM judge.
This creates the final rankings combining JLLM scores with similarity metrics.
"""

import os
import pandas as pd
import numpy as np
from pathlib import Path
import shutil

def generate_all_scores_ranked(jllm_name):
    """
    Generate all_scores_ranked.csv by copying from llm_all_metrics_with_rank.csv
    or regenerating if needed.
    """
    
    # Paths
    judge_dir = f"/home/hongyu/Documents/SimBench/scoring/out_diff_models/out_{jllm_name.replace('.', '-')}"
    metrics_file = os.path.join(judge_dir, "llm_all_metrics_with_rank.csv")
    output_file = os.path.join(judge_dir, "all_scores_ranked.csv")
    
    # Check if llm_all_metrics_with_rank.csv exists
    if not os.path.exists(metrics_file):
        print(f"❌ Error: {metrics_file} not found")
        return False
    
    # Read the metrics file
    df = pd.read_csv(metrics_file)
    
    # Recalculate ConsensusScore if needed
    # Using a weighted average with JLLM scores having higher weight
    score_cols = []
    weights = []
    
    # JLLM scores (higher weight)
    jllm_cols = ['Score Document', 'Score Reference', 'Score Reference Document']
    for col in jllm_cols:
        if col in df.columns and df[col].notna().any():
            score_cols.append(col)
            weights.append(2.0)  # Double weight for JLLM scores
    
    # Similarity metrics (normal weight)
    sim_cols = ['codebleu', 'rouge1', 'rougeL']
    for col in sim_cols:
        if col in df.columns and df[col].notna().any():
            # Scale to 0-100 if needed
            if df[col].max() <= 1.0:
                df[col + '_scaled'] = df[col] * 100
                score_cols.append(col + '_scaled')
            else:
                score_cols.append(col)
            weights.append(1.0)  # Normal weight for similarity metrics
    
    # Calculate weighted consensus score
    if score_cols:
        weighted_sum = 0
        weight_total = 0
        for i, col in enumerate(score_cols):
            weighted_sum += df[col].fillna(0) * weights[i]
            weight_total += weights[i]
        df['ConsensusScore'] = (weighted_sum / weight_total).round(2)
    else:
        df['ConsensusScore'] = 0
    
    # Recalculate rankings
    df['Rank'] = df['ConsensusScore'].rank(ascending=False, method='min').astype(int)
    df = df.sort_values('Rank')
    
    # Remove scaled columns from output
    output_cols = [col for col in df.columns if not col.endswith('_scaled')]
    df = df[output_cols]
    
    # Save as all_scores_ranked.csv
    df.to_csv(output_file, index=False)
    
    print(f"✅ Generated: {output_file}")
    print(f"   Total models: {len(df)}")
    print(f"   Top model: {df.iloc[0]['model']} (Score: {df.iloc[0]['ConsensusScore']})")
    
    return True

def main():
    """Process all three JLLM judges."""
    
    judges = [
        "gpt-4.1-nano",
        "gpt-4.1-mini",
        "gpt-4o-mini"
    ]
    
    print("=" * 60)
    print("Generating all_scores_ranked.csv for all JLLMs")
    print("=" * 60)
    
    for judge in judges:
        print(f"\nProcessing {judge}...")
        success = generate_all_scores_ranked(judge)
        if not success:
            print(f"Failed to process {judge}")
    
    print("\n" + "=" * 60)
    print("Complete! Generated files:")
    print("=" * 60)
    
    for judge in judges:
        file_path = f"/home/hongyu/Documents/SimBench/scoring/out_diff_models/out_{judge.replace('.', '-')}/all_scores_ranked.csv"
        if os.path.exists(file_path):
            print(f"✅ {file_path}")
        else:
            print(f"❌ Missing: {file_path}")

if __name__ == "__main__":
    main()