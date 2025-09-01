#!/usr/bin/env python3
"""
Generate complete llm_all_metrics_with_rank.csv for each JLLM judge.
This script combines JLLM scores with similarity metrics and creates rankings.
"""

import os
import pandas as pd
import numpy as np
from pathlib import Path
import logging

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

def merge_evaluation_scores(judge_name):
    """Merge individual evaluation scores into combined CSV for a judge."""
    
    output_path = f"/home/hongyu/Documents/SimBench/output_llms_{judge_name.replace('.', '-')}"
    combined_csv_path = os.path.join(output_path, f"combined_evaluation_scores_{judge_name}.csv")
    
    all_data = []
    
    # Walk through all model directories
    for model_dir in os.listdir(output_path):
        model_path = os.path.join(output_path, model_dir)
        
        if not os.path.isdir(model_path):
            continue
        
        # Look for evaluation scores
        csv_candidates = [
            os.path.join(model_path, "evaluation_scores_fixed.csv"),
            os.path.join(model_path, "evaluation_scores.csv")
        ]
        
        csv_file = None
        for candidate in csv_candidates:
            if os.path.exists(candidate):
                csv_file = candidate
                break
        
        if csv_file:
            try:
                df = pd.read_csv(csv_file)
                all_data.append(df)
            except Exception as e:
                logger.warning(f"Error reading {csv_file}: {e}")
    
    if all_data:
        # Combine all dataframes
        combined_df = pd.concat(all_data, ignore_index=True)
        
        # Group by model and calculate average scores
        grouped = combined_df.groupby('Test Model').agg({
            'Score Document': 'mean',
            'Score Reference': 'mean',
            'Score Reference Document': 'mean'
        }).reset_index()
        
        grouped.columns = ['model', 'Score Document', 'Score Reference', 'Score Reference Document']
        
        # Save combined scores
        grouped.to_csv(combined_csv_path, index=False)
        logger.info(f"Created combined scores: {combined_csv_path}")
        return combined_csv_path
    
    return None

def merge_with_similarity_metrics(judge_csv_path, judge_name):
    """Merge JLLM scores with similarity metrics."""
    
    # Read JLLM scores
    jllm_df = pd.read_csv(judge_csv_path)
    
    # Read similarity metrics (evaluation_results.csv)
    similarity_path = "/home/hongyu/Documents/SimBench/scoring/out/evaluation_results.csv"
    
    if not os.path.exists(similarity_path):
        # Try alternative location
        similarity_path = "/home/hongyu/Documents/SimBench/statistic/evaluation_results.csv"
    
    if os.path.exists(similarity_path):
        sim_df = pd.read_csv(similarity_path)
        
        # Merge on model name
        merged_df = pd.merge(jllm_df, sim_df, on='model', how='outer')
    else:
        logger.warning("Similarity metrics not found, using JLLM scores only")
        merged_df = jllm_df
    
    # Calculate consensus score
    score_cols = ['Score Document', 'Score Reference', 'Score Reference Document']
    similarity_cols = ['codebleu', 'rouge1', 'rougeL']
    
    # Normalize similarity metrics to 0-100 scale if needed
    for col in similarity_cols:
        if col in merged_df.columns:
            if merged_df[col].max() <= 1.0:
                merged_df[col + '_scaled'] = merged_df[col] * 100
                score_cols.append(col + '_scaled')
            else:
                score_cols.append(col)
    
    # Calculate consensus score (average of available metrics)
    available_cols = [col for col in score_cols if col in merged_df.columns]
    if available_cols:
        merged_df['ConsensusScore'] = merged_df[available_cols].mean(axis=1, skipna=True)
    else:
        merged_df['ConsensusScore'] = 50.0  # Default
    
    # Add rankings
    merged_df['Rank'] = merged_df['ConsensusScore'].rank(ascending=False, method='min')
    merged_df = merged_df.sort_values('Rank')
    
    # Reorder columns
    priority_cols = ['Rank', 'model', 'ConsensusScore', 'Score Document', 'Score Reference', 'Score Reference Document']
    other_cols = [col for col in merged_df.columns if col not in priority_cols and not col.endswith('_scaled')]
    final_cols = priority_cols + other_cols
    final_cols = [col for col in final_cols if col in merged_df.columns]
    
    merged_df = merged_df[final_cols]
    
    # Save to output directory
    output_dir = f"/home/hongyu/Documents/SimBench/scoring/out_diff_models/out_{judge_name}"
    os.makedirs(output_dir, exist_ok=True)
    
    output_path = os.path.join(output_dir, "llm_all_metrics_with_rank.csv")
    merged_df.to_csv(output_path, index=False)
    
    logger.info(f"Created rankings: {output_path}")
    return output_path

def generate_summary_stats(ranking_path, judge_name):
    """Generate summary statistics for the rankings."""
    
    df = pd.read_csv(ranking_path)
    
    output_dir = os.path.dirname(ranking_path)
    
    # Summary statistics
    summary = {
        'Judge': judge_name,
        'Total Models': len(df),
        'Top Score': df['ConsensusScore'].max(),
        'Bottom Score': df['ConsensusScore'].min(),
        'Mean Score': df['ConsensusScore'].mean(),
        'Std Dev': df['ConsensusScore'].std(),
        'Top 5 Models': ', '.join(df.head(5)['model'].tolist())
    }
    
    # Save summary
    summary_path = os.path.join(output_dir, "summary_stats.txt")
    with open(summary_path, 'w') as f:
        for key, value in summary.items():
            f.write(f"{key}: {value}\n")
    
    logger.info(f"Created summary: {summary_path}")
    
    # Print top 10
    print(f"\n{'='*60}")
    print(f"Top 10 Models - Judge: {judge_name}")
    print(f"{'='*60}")
    print(df[['Rank', 'model', 'ConsensusScore']].head(10).to_string(index=False))

def main():
    """Main function to generate rankings for each judge."""
    
    judges = [
        "gpt-4-1-mini",
        "gpt-4-1-nano",
        "gpt-4o-mini"  # Include this for completeness
    ]
    
    for judge in judges:
        print(f"\n{'='*60}")
        print(f"Processing Judge: {judge}")
        print(f"{'='*60}")
        
        try:
            # Step 1: Merge evaluation scores
            logger.info(f"Step 1: Merging evaluation scores for {judge}...")
            combined_csv = merge_evaluation_scores(judge)
            
            if not combined_csv:
                logger.error(f"Failed to create combined scores for {judge}")
                continue
            
            # Step 2: Merge with similarity metrics and create rankings
            logger.info(f"Step 2: Creating rankings for {judge}...")
            ranking_csv = merge_with_similarity_metrics(combined_csv, judge)
            
            # Step 3: Generate summary statistics
            logger.info(f"Step 3: Generating summary for {judge}...")
            generate_summary_stats(ranking_csv, judge)
            
            logger.info(f"✅ Successfully completed processing for {judge}")
            
        except Exception as e:
            logger.error(f"Error processing {judge}: {e}")
            import traceback
            traceback.print_exc()
    
    print(f"\n{'='*60}")
    print("All judges processed!")
    print(f"{'='*60}")
    
    # Print file locations
    print("\nOutput files created:")
    for judge in judges:
        output_dir = f"/home/hongyu/Documents/SimBench/scoring/out_diff_models/out_{judge}"
        ranking_file = os.path.join(output_dir, "llm_all_metrics_with_rank.csv")
        if os.path.exists(ranking_file):
            print(f"  ✅ {judge}: {ranking_file}")
        else:
            print(f"  ❌ {judge}: File not created")

if __name__ == "__main__":
    main()