#!/usr/bin/env python3
"""
Generate all_scores_ranked.csv for each JLLM in their output_llms_* directories.
This script reads individual evaluation scores and creates rankings.
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

def collect_evaluation_scores(output_dir):
    """
    Collect all evaluation scores from model subdirectories.
    """
    all_scores = []
    
    # Walk through model directories
    for model_dir in os.listdir(output_dir):
        model_path = os.path.join(output_dir, model_dir)
        
        if not os.path.isdir(model_path):
            continue
        
        # Skip CSV files in the root
        if model_dir.endswith('.csv'):
            continue
        
        # First check for evaluation_scores_fixed.csv in the model directory itself
        root_score_files = [
            os.path.join(model_path, "evaluation_scores_fixed.csv"),
            os.path.join(model_path, "evaluation_scores.csv")
        ]
        
        found_in_root = False
        for score_file in root_score_files:
            if os.path.exists(score_file):
                try:
                    df = pd.read_csv(score_file)
                    # Ensure numeric types for score columns
                    score_cols = ['Score Document', 'Score Reference', 'Score Reference Document']
                    for col in score_cols:
                        if col in df.columns:
                            df[col] = pd.to_numeric(df[col], errors='coerce')
                    # Add model info
                    if 'model' not in df.columns and 'Test Model' not in df.columns:
                        df['model'] = model_dir
                    elif 'Test Model' in df.columns:
                        df['model'] = df['Test Model']
                    all_scores.append(df)
                    found_in_root = True
                    break
                except Exception as e:
                    logger.warning(f"Error reading {score_file}: {e}")
        
        # If not found in root, look in system subdirectories
        if not found_in_root:
            for system_dir in os.listdir(model_path):
                system_path = os.path.join(model_path, system_dir)
                
                if not os.path.isdir(system_path):
                    continue
                
                # Check for evaluation_scores.csv or evaluation_scores_fixed.csv
                score_files = [
                    os.path.join(system_path, "evaluation_scores_fixed.csv"),
                    os.path.join(system_path, "evaluation_scores.csv")
                ]
                
                for score_file in score_files:
                    if os.path.exists(score_file):
                        try:
                            df = pd.read_csv(score_file)
                            # Ensure numeric types for score columns
                            score_cols = ['Score Document', 'Score Reference', 'Score Reference Document']
                            for col in score_cols:
                                if col in df.columns:
                                    df[col] = pd.to_numeric(df[col], errors='coerce')
                            # Add model and system info
                            df['model'] = model_dir
                            df['system'] = system_dir
                            all_scores.append(df)
                            break
                        except Exception as e:
                            logger.warning(f"Error reading {score_file}: {e}")
    
    if not all_scores:
        return None
    
    # Combine all scores
    combined_df = pd.concat(all_scores, ignore_index=True)
    
    # Clean up model column
    if 'Test Model' in combined_df.columns and 'model' not in combined_df.columns:
        combined_df['model'] = combined_df['Test Model']
    
    return combined_df

def aggregate_and_rank(scores_df):
    """
    Aggregate scores by model and create rankings.
    """
    if scores_df is None or scores_df.empty:
        return None
    
    # Ensure we have the right columns
    score_cols = ['Score Document', 'Score Reference', 'Score Reference Document']
    
    # Group by model and calculate average scores
    aggregated = scores_df.groupby('model')[score_cols].mean().reset_index()
    
    # Calculate consensus score (average of the three scores)
    aggregated['ConsensusScore'] = aggregated[score_cols].mean(axis=1).round(2)
    
    # Add rankings
    aggregated['Rank'] = aggregated['ConsensusScore'].rank(ascending=False, method='min').astype(int)
    
    # Sort by rank
    aggregated = aggregated.sort_values('Rank')
    
    # Reorder columns
    column_order = ['Rank', 'model', 'ConsensusScore'] + score_cols
    aggregated = aggregated[column_order]
    
    return aggregated

def read_similarity_metrics():
    """
    Read similarity metrics from evaluation_results.csv if available.
    """
    paths = [
        "/home/hongyu/Documents/SimBench/statistic/evaluation_results.csv",
        "/home/hongyu/Documents/SimBench/scoring/out/evaluation_results.csv"
    ]
    
    for path in paths:
        if os.path.exists(path):
            df = pd.read_csv(path)
            # Aggregate by model
            metric_cols = ['codebleu', 'ngram_match_score', 'weighted_ngram_match_score',
                          'syntax_match_score', 'dataflow_match_score',
                          'rouge1', 'rouge2', 'rougeL', 'rougeLsum']
            
            agg_df = df.groupby('model')[metric_cols].mean().reset_index()
            return agg_df
    
    return None

def merge_with_similarity(ranked_df):
    """
    Merge ranked scores with similarity metrics.
    """
    sim_df = read_similarity_metrics()
    
    if sim_df is None:
        logger.info("No similarity metrics found, using JLLM scores only")
        return ranked_df
    
    # Merge on model name
    merged = pd.merge(ranked_df, sim_df, on='model', how='left')
    
    # Recalculate ConsensusScore including similarity metrics
    score_cols = ['Score Document', 'Score Reference', 'Score Reference Document']
    sim_cols = ['codebleu', 'rouge1', 'rougeL']
    
    all_scores = []
    for col in score_cols:
        if col in merged.columns:
            all_scores.append(merged[col])
    
    for col in sim_cols:
        if col in merged.columns:
            # Scale to 0-100 if needed
            if merged[col].max() <= 1.0:
                all_scores.append(merged[col] * 100)
            else:
                all_scores.append(merged[col])
    
    if all_scores:
        merged['ConsensusScore'] = pd.concat(all_scores, axis=1).mean(axis=1, skipna=True).round(2)
        merged['Rank'] = merged['ConsensusScore'].rank(ascending=False, method='min').astype(int)
        merged = merged.sort_values('Rank')
    
    return merged

def generate_all_scores_ranked(jllm_name):
    """
    Generate all_scores_ranked.csv for a specific JLLM.
    """
    # Determine output directory
    output_dir = f"/home/hongyu/Documents/SimBench/output_llms_{jllm_name.replace('.', '-')}"
    
    if not os.path.exists(output_dir):
        logger.error(f"Directory not found: {output_dir}")
        return False
    
    logger.info(f"Processing {jllm_name}...")
    logger.info(f"Collecting scores from: {output_dir}")
    
    # Collect all evaluation scores
    scores_df = collect_evaluation_scores(output_dir)
    
    if scores_df is None:
        logger.error(f"No evaluation scores found in {output_dir}")
        return False
    
    logger.info(f"Found scores for {scores_df['model'].nunique()} models")
    
    # Aggregate and rank
    ranked_df = aggregate_and_rank(scores_df)
    
    if ranked_df is None:
        logger.error(f"Failed to aggregate scores for {jllm_name}")
        return False
    
    # Merge with similarity metrics
    final_df = merge_with_similarity(ranked_df)
    
    # Save to output directory
    output_file = os.path.join(output_dir, "all_scores_ranked.csv")
    final_df.to_csv(output_file, index=False)
    
    logger.info(f"✅ Generated: {output_file}")
    logger.info(f"   Total models: {len(final_df)}")
    logger.info(f"   Top 3 models:")
    for i, row in final_df.head(3).iterrows():
        logger.info(f"     {row['Rank']}. {row['model']}: {row['ConsensusScore']}")
    
    return True

def main():
    """
    Process all three JLLM judges.
    """
    judges = [
        "gpt-4.1-nano",
        "gpt-4.1-mini",
        "gpt-4o-mini"
    ]
    
    print("=" * 60)
    print("Generating all_scores_ranked.csv in output_llms directories")
    print("=" * 60)
    
    results = {}
    
    for judge in judges:
        print(f"\n{'='*40}")
        print(f"Processing: {judge}")
        print(f"{'='*40}")
        
        try:
            success = generate_all_scores_ranked(judge)
            results[judge] = success
        except Exception as e:
            logger.error(f"Error processing {judge}: {e}")
            import traceback
            traceback.print_exc()
            results[judge] = False
    
    # Print summary
    print("\n" + "=" * 60)
    print("Summary")
    print("=" * 60)
    
    for judge, success in results.items():
        output_file = f"/home/hongyu/Documents/SimBench/output_llms_{judge.replace('.', '-')}/all_scores_ranked.csv"
        if success and os.path.exists(output_file):
            df = pd.read_csv(output_file)
            print(f"\n✅ {judge}: {output_file}")
            print(f"   Models: {len(df)}")
            print(f"   Top model: {df.iloc[0]['model']} (Score: {df.iloc[0]['ConsensusScore']})")
        else:
            print(f"\n❌ {judge}: Failed to generate")

if __name__ == "__main__":
    main()