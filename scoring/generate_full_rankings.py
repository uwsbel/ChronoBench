#!/usr/bin/env python3
"""
Generate complete llm_all_metrics_with_rank.csv for each JLLM judge.
This script properly aggregates JLLM scores with similarity metrics.
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

def read_jllm_scores(jllm_name):
    """Read combined JLLM evaluation scores."""
    
    # Map judge names to directory names
    dir_name = f"output_llms_{jllm_name.replace('.', '-')}"
    base_path = f"/home/hongyu/Documents/SimBench/{dir_name}"
    
    # Try different file name patterns
    csv_candidates = [
        os.path.join(base_path, f"combined_evaluation_scores_{jllm_name}.csv"),
        os.path.join(base_path, f"combined_evaluation_scores_{jllm_name.replace('.', '-')}.csv"),
        os.path.join(base_path, f"combined_evaluation_scores_gpt-4-1-nano.csv"),  # Fallback for gpt-4.1-nano
        os.path.join(base_path, "combined_evaluation_scores.csv"),  # Generic name
    ]
    
    for csv_path in csv_candidates:
        if os.path.exists(csv_path):
            # Check if file is not empty
            if os.path.getsize(csv_path) > 0:
                logger.info(f"Reading JLLM scores from: {csv_path}")
                df = pd.read_csv(csv_path)
                logger.info(f"  Found {len(df)} models with JLLM scores")
                return df
            else:
                logger.warning(f"File is empty: {csv_path}")
    
    logger.error(f"No combined evaluation scores found for {jllm_name}")
    return None

def aggregate_similarity_metrics():
    """Read and aggregate similarity metrics from evaluation_results.csv."""
    
    # Try different locations
    paths_to_try = [
        "/home/hongyu/Documents/SimBench/statistic/evaluation_results.csv",
        "/home/hongyu/Documents/SimBench/scoring/out/evaluation_results.csv"
    ]
    
    for sim_path in paths_to_try:
        if os.path.exists(sim_path):
            logger.info(f"Reading similarity metrics from: {sim_path}")
            df = pd.read_csv(sim_path)
            
            # Aggregate by model (average across all systems and rounds)
            metric_cols = ['codebleu', 'ngram_match_score', 'weighted_ngram_match_score',
                          'syntax_match_score', 'dataflow_match_score', 
                          'rouge1', 'rouge2', 'rougeL', 'rougeLsum']
            
            # Group by model and calculate mean
            agg_df = df.groupby('model')[metric_cols].mean().reset_index()
            
            logger.info(f"  Aggregated metrics for {len(agg_df)} models")
            return agg_df
    
    logger.warning("No similarity metrics file found")
    return None

def merge_and_rank(jllm_scores, sim_metrics):
    """Merge JLLM scores with similarity metrics and calculate rankings."""
    
    if jllm_scores is None:
        logger.error("No JLLM scores available")
        return None
    
    # Start with JLLM scores
    merged_df = jllm_scores.copy()
    
    # Merge with similarity metrics if available
    if sim_metrics is not None:
        merged_df = pd.merge(merged_df, sim_metrics, on='model', how='outer')
        logger.info(f"Merged data contains {len(merged_df)} total models")
    
    # Fill missing values with 0
    score_cols = ['Score Document', 'Score Reference', 'Score Reference Document']
    for col in score_cols:
        if col in merged_df.columns:
            merged_df[col] = merged_df[col].fillna(0)
    
    # Calculate ConsensusScore
    # Include JLLM scores and similarity metrics (if similarity < 1, scale to 0-100)
    consensus_cols = []
    
    # Add JLLM scores to consensus
    for col in score_cols:
        if col in merged_df.columns:
            consensus_cols.append(col)
    
    # Add similarity metrics to consensus (scale if needed)
    similarity_cols = ['codebleu', 'rouge1', 'rougeL']
    for col in similarity_cols:
        if col in merged_df.columns:
            # Check if values are in 0-1 range
            if merged_df[col].max() <= 1.0:
                # Scale to 0-100
                merged_df[f'{col}_scaled'] = merged_df[col] * 100
                consensus_cols.append(f'{col}_scaled')
            else:
                consensus_cols.append(col)
    
    # Calculate consensus score
    if consensus_cols:
        merged_df['ConsensusScore'] = merged_df[consensus_cols].mean(axis=1, skipna=True)
    else:
        merged_df['ConsensusScore'] = 0
    
    # Round ConsensusScore to 2 decimal places
    merged_df['ConsensusScore'] = merged_df['ConsensusScore'].round(2)
    
    # Add rankings
    merged_df['Rank'] = merged_df['ConsensusScore'].rank(ascending=False, method='min').astype(int)
    
    # Sort by rank
    merged_df = merged_df.sort_values('Rank')
    
    # Reorder columns (put important ones first)
    priority_cols = ['Rank', 'model', 'ConsensusScore', 
                    'Score Document', 'Score Reference', 'Score Reference Document']
    
    # Add all similarity metrics columns (excluding scaled versions)
    sim_metric_cols = ['codebleu', 'ngram_match_score', 'weighted_ngram_match_score',
                      'syntax_match_score', 'dataflow_match_score',
                      'rouge1', 'rouge2', 'rougeL', 'rougeLsum']
    
    # Build final column list
    final_cols = []
    for col in priority_cols:
        if col in merged_df.columns:
            final_cols.append(col)
    
    for col in sim_metric_cols:
        if col in merged_df.columns:
            final_cols.append(col)
    
    # Remove scaled columns from final output
    merged_df = merged_df[final_cols]
    
    return merged_df

def save_rankings(rankings_df, jllm_name):
    """Save rankings to the appropriate output directory."""
    
    if rankings_df is None:
        logger.error(f"No rankings to save for {jllm_name}")
        return None
    
    # Create output directory
    output_dir = f"/home/hongyu/Documents/SimBench/scoring/out_diff_models/out_{jllm_name.replace('.', '-')}"
    os.makedirs(output_dir, exist_ok=True)
    
    # Save rankings
    output_path = os.path.join(output_dir, "llm_all_metrics_with_rank.csv")
    rankings_df.to_csv(output_path, index=False)
    
    logger.info(f"Saved rankings to: {output_path}")
    
    # Generate summary statistics
    summary_path = os.path.join(output_dir, "summary_stats.txt")
    with open(summary_path, 'w') as f:
        f.write(f"Judge: {jllm_name}\n")
        f.write(f"Total Models: {len(rankings_df)}\n")
        f.write(f"Top Score: {rankings_df['ConsensusScore'].max():.2f}\n")
        f.write(f"Bottom Score: {rankings_df['ConsensusScore'].min():.2f}\n")
        f.write(f"Mean Score: {rankings_df['ConsensusScore'].mean():.2f}\n")
        f.write(f"Std Dev: {rankings_df['ConsensusScore'].std():.2f}\n")
        f.write(f"Top 5 Models: {', '.join(rankings_df.head(5)['model'].tolist())}\n")
    
    logger.info(f"Saved summary to: {summary_path}")
    
    return output_path

def process_judge(jllm_name):
    """Process a single JLLM judge."""
    
    print(f"\n{'='*60}")
    print(f"Processing Judge: {jllm_name}")
    print(f"{'='*60}")
    
    # Step 1: Read JLLM scores
    logger.info("Step 1: Reading JLLM scores...")
    jllm_scores = read_jllm_scores(jllm_name)
    
    # Step 2: Read and aggregate similarity metrics
    logger.info("Step 2: Aggregating similarity metrics...")
    sim_metrics = aggregate_similarity_metrics()
    
    # Step 3: Merge and calculate rankings
    logger.info("Step 3: Merging and calculating rankings...")
    rankings = merge_and_rank(jllm_scores, sim_metrics)
    
    # Step 4: Save results
    logger.info("Step 4: Saving results...")
    output_path = save_rankings(rankings, jllm_name)
    
    if output_path:
        # Print top 10 models
        print(f"\nTop 10 Models for {jllm_name}:")
        print("-" * 60)
        if rankings is not None and len(rankings) > 0:
            print(rankings[['Rank', 'model', 'ConsensusScore']].head(10).to_string(index=False))
        
        logger.info(f"✅ Successfully processed {jllm_name}")
    else:
        logger.error(f"❌ Failed to process {jllm_name}")
    
    return output_path

def main():
    """Main function to process all JLLM judges."""
    
    judges = [
        "gpt-4.1-nano",
        "gpt-4.1-mini", 
        "gpt-4o-mini"
    ]
    
    results = {}
    
    for judge in judges:
        try:
            output_path = process_judge(judge)
            results[judge] = output_path
        except Exception as e:
            logger.error(f"Error processing {judge}: {e}")
            import traceback
            traceback.print_exc()
            results[judge] = None
    
    # Print summary
    print(f"\n{'='*60}")
    print("Processing Complete!")
    print(f"{'='*60}")
    print("\nGenerated files:")
    for judge, path in results.items():
        if path:
            print(f"  ✅ {judge}: {path}")
        else:
            print(f"  ❌ {judge}: Failed to generate")

if __name__ == "__main__":
    main()