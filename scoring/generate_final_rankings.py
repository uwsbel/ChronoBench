#!/usr/bin/env python3
"""
Comprehensive ranking script for SimBench evaluation results.
This script merges JLLM scores with similarity metrics to generate final rankings.

Usage:
    python generate_final_rankings.py --jllm gpt-4.1-mini    # Process single JLLM
    python generate_final_rankings.py --all                  # Process all JLLMs
"""

import os
import sys
import argparse
import pandas as pd
import numpy as np
from pathlib import Path
import logging
import csv
from typing import List, Dict, Tuple, Optional

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Base paths
BASE_DIR = Path("/home/hongyu/Documents/SimBench")
SCORING_DIR = BASE_DIR / "scoring"
STATISTIC_DIR = BASE_DIR / "statistic"

# Known JLLM judges
JLLM_JUDGES = [
    "gpt-4.1-nano",
    "gpt-4.1-mini", 
    "gpt-4o-mini"
]

def normalize_model_name(name: str) -> str:
    """Normalize model names for consistent matching."""
    # Remove common prefixes
    name = name.replace("pe_", "")
    # Standardize separators
    name = name.replace("_", "-")
    return name

def collect_jllm_scores(jllm_name: str) -> pd.DataFrame:
    """
    Collect all evaluation scores for a specific JLLM judge.
    
    Returns a DataFrame with columns:
    - model: Model name
    - Score Document: Average score against documentation
    - Score Reference: Average score against reference
    - Score Reference Document: Average score against both
    """
    output_dir = BASE_DIR / f"output_llms_{jllm_name.replace('.', '-')}"
    
    if not output_dir.exists():
        logger.error(f"Directory not found: {output_dir}")
        return pd.DataFrame()
    
    all_scores = []
    failed_evaluations = []  # Track failed evaluations
    
    # Walk through model directories
    for model_dir in output_dir.iterdir():
        if not model_dir.is_dir():
            continue
        
        model_name = model_dir.name
        
        # Skip CSV files in root
        if model_name.endswith('.csv'):
            continue
        
        # Collect scores from all systems
        model_scores = []
        
        for system_dir in model_dir.iterdir():
            if not system_dir.is_dir():
                continue
            
            # Look for evaluation_scores.csv
            score_file = system_dir / "evaluation_scores.csv"
            
            if score_file.exists():
                try:
                    df = pd.read_csv(score_file)
                    # Check for FAILED values
                    if all(col in df.columns for col in ['Score Document', 'Score Reference', 'Score Reference Document']):
                        score_cols = ['Score Document', 'Score Reference', 'Score Reference Document']
                        
                        # Check if any values are FAILED
                        for idx, row in df.iterrows():
                            for col in score_cols:
                                if str(row[col]) == 'FAILED':
                                    failed_evaluations.append({
                                        'model': model_name,
                                        'system': system_dir.name,
                                        'round': row.get('Round', 'unknown'),
                                        'column': col
                                    })
                        
                        # Try to convert to numeric
                        for col in score_cols:
                            df[col] = pd.to_numeric(df[col], errors='coerce')
                        # Only add rows that have at least some valid scores
                        clean_df = df[score_cols].dropna(how='all')
                        if not clean_df.empty:
                            model_scores.append(clean_df.values)
                except Exception as e:
                    logger.warning(f"Error reading {score_file}: {e}")
    
        if model_scores:
            # Calculate average across all systems and rounds
            scores_array = np.vstack(model_scores)
            avg_scores = np.mean(scores_array, axis=0)
            
            all_scores.append({
                'model': model_name,
                'Score Document': avg_scores[0],
                'Score Reference': avg_scores[1],
                'Score Reference Document': avg_scores[2]
            })
    
    # Report failed evaluations
    if failed_evaluations:
        logger.error(f"\n{'='*60}")
        logger.error(f"ERROR: Found {len(failed_evaluations)} FAILED evaluations")
        logger.error(f"{'='*60}")
        
        # Group by model
        failed_by_model = {}
        for fail in failed_evaluations:
            model = fail['model']
            if model not in failed_by_model:
                failed_by_model[model] = []
            failed_by_model[model].append(fail)
        
        for model, failures in failed_by_model.items():
            logger.error(f"\n{model}:")
            for fail in failures[:5]:  # Show first 5 failures per model
                logger.error(f"  - {fail['system']}/{fail['round']}: {fail['column']}")
            if len(failures) > 5:
                logger.error(f"  ... and {len(failures)-5} more failures")
        
        logger.error(f"\n{'='*60}")
        logger.error("Please run: python fix_failed_simulations.py")
        logger.error("to attempt to regenerate these failed simulations")
        logger.error(f"{'='*60}\n")
        
        # Don't exit, just report the error
        # sys.exit(1)
    
    if not all_scores:
        logger.warning(f"No scores found for {jllm_name}")
        return pd.DataFrame()
    
    return pd.DataFrame(all_scores)

def load_similarity_metrics() -> pd.DataFrame:
    """
    Load similarity metrics from evaluation_results.csv.
    
    Returns a DataFrame with model-level aggregated metrics.
    """
    eval_file = STATISTIC_DIR / "evaluation_results.csv"
    
    if not eval_file.exists():
        # Try alternative location
        eval_file = SCORING_DIR / "out" / "evaluation_results.csv"
    
    if not eval_file.exists():
        logger.warning("No similarity metrics file found")
        return pd.DataFrame()
    
    try:
        df = pd.read_csv(eval_file)
        
        # Aggregate by model
        metric_cols = [
            'codebleu', 'ngram_match_score', 'weighted_ngram_match_score',
            'syntax_match_score', 'dataflow_match_score',
            'rouge1', 'rouge2', 'rougeL', 'rougeLsum'
        ]
        
        # Group by model and calculate mean
        agg_df = df.groupby('model')[metric_cols].mean().reset_index()
        
        return agg_df
    
    except Exception as e:
        logger.error(f"Error loading similarity metrics: {e}")
        return pd.DataFrame()

def calculate_consensus_score(df: pd.DataFrame) -> pd.DataFrame:
    """
    Calculate consensus score and rank models.
    
    ConsensusScore = mean of:
    - Score Document
    - Score Reference
    - Score Reference Document
    - codebleu × 100
    - rouge1 × 100
    - rougeL × 100
    """
    # Prepare score components
    score_components = []
    
    # Add JLLM scores
    for col in ['Score Document', 'Score Reference', 'Score Reference Document']:
        if col in df.columns:
            score_components.append(df[col])
    
    # Add scaled similarity metrics
    for col in ['codebleu', 'rouge1', 'rougeL']:
        if col in df.columns:
            # Scale to 0-100 if needed
            if df[col].max() <= 1.0:
                score_components.append(df[col] * 100)
            else:
                score_components.append(df[col])
    
    if score_components:
        # Calculate consensus score as mean of all components
        df['ConsensusScore'] = pd.concat(score_components, axis=1).mean(axis=1, skipna=True).round(2)
    else:
        # Fallback to just JLLM scores if no similarity metrics
        jllm_cols = ['Score Document', 'Score Reference', 'Score Reference Document']
        available_cols = [col for col in jllm_cols if col in df.columns]
        if available_cols:
            df['ConsensusScore'] = df[available_cols].mean(axis=1, skipna=True).round(2)
        else:
            df['ConsensusScore'] = 0.0
    
    # Add ranking
    df['Rank'] = df['ConsensusScore'].rank(ascending=False, method='min').astype(int)
    
    # Sort by rank
    df = df.sort_values('Rank')
    
    return df

def format_output_dataframe(df: pd.DataFrame) -> pd.DataFrame:
    """
    Format the output DataFrame to match the original format exactly.
    
    Column order:
    Rank, model, ConsensusScore, Score Document, Score Reference, Score Reference Document,
    codebleu, ngram_match_score, weighted_ngram_match_score, syntax_match_score, dataflow_match_score,
    rouge1, rouge2, rougeL, rougeLsum
    """
    # Define the exact column order
    column_order = [
        'Rank', 'model', 'ConsensusScore',
        'Score Document', 'Score Reference', 'Score Reference Document',
        'codebleu', 'ngram_match_score', 'weighted_ngram_match_score',
        'syntax_match_score', 'dataflow_match_score',
        'rouge1', 'rouge2', 'rougeL', 'rougeLsum'
    ]
    
    # Add missing columns with NaN
    for col in column_order:
        if col not in df.columns:
            df[col] = np.nan
    
    # Reorder columns
    df = df[column_order]
    
    # Format numeric columns
    numeric_cols = df.select_dtypes(include=[np.number]).columns
    for col in numeric_cols:
        if col == 'Rank':
            continue
        elif col == 'ConsensusScore':
            df[col] = df[col].round(2)
        elif col in ['Score Document', 'Score Reference', 'Score Reference Document']:
            df[col] = df[col].round(2)
        else:
            # Keep original precision for metrics
            pass
    
    return df

def save_combined_scores(jllm_name: str, scores_df: pd.DataFrame):
    """Save the combined evaluation scores for a JLLM."""
    output_dir = BASE_DIR / f"output_llms_{jllm_name.replace('.', '-')}"
    output_file = output_dir / f"combined_evaluation_scores_{jllm_name}.csv"
    
    if not scores_df.empty:
        # Save just the JLLM scores
        jllm_cols = ['model', 'Score Document', 'Score Reference', 'Score Reference Document']
        scores_df[jllm_cols].to_csv(output_file, index=False)
        logger.info(f"Saved combined scores to {output_file}")

def process_jllm(jllm_name: str) -> bool:
    """
    Process a single JLLM judge and generate rankings.
    
    Returns True if successful, False otherwise.
    """
    logger.info(f"\n{'='*60}")
    logger.info(f"Processing JLLM: {jllm_name}")
    logger.info(f"{'='*60}")
    
    # Step 1: Collect JLLM scores
    logger.info("Collecting JLLM evaluation scores...")
    jllm_scores = collect_jllm_scores(jllm_name)
    
    if jllm_scores.empty:
        logger.error(f"No JLLM scores found for {jllm_name}")
        return False
    
    logger.info(f"Found scores for {len(jllm_scores)} models")
    
    # Step 2: Load similarity metrics
    logger.info("Loading similarity metrics...")
    sim_metrics = load_similarity_metrics()
    
    if not sim_metrics.empty:
        logger.info(f"Loaded metrics for {len(sim_metrics)} models")
    else:
        logger.warning("No similarity metrics found, using JLLM scores only")
    
    # Step 3: Merge dataframes
    if not sim_metrics.empty:
        merged_df = pd.merge(jllm_scores, sim_metrics, on='model', how='left')
        logger.info(f"Merged data for {len(merged_df)} models")
    else:
        merged_df = jllm_scores
    
    # Step 4: Calculate consensus score and rank
    logger.info("Calculating consensus scores and rankings...")
    ranked_df = calculate_consensus_score(merged_df)
    
    # Step 5: Format output
    final_df = format_output_dataframe(ranked_df)
    
    # Step 6: Save results
    output_dir = BASE_DIR / f"output_llms_{jllm_name.replace('.', '-')}"
    output_dir.mkdir(exist_ok=True)
    
    # Save all_scores_ranked.csv
    output_file = output_dir / "all_scores_ranked.csv"
    final_df.to_csv(output_file, index=False, float_format='%.2f')
    logger.info(f"✓ Saved rankings to {output_file}")
    
    # Save combined_evaluation_scores.csv
    save_combined_scores(jllm_name, jllm_scores)
    
    # Print top 5 models
    logger.info("\nTop 5 models:")
    for idx, row in final_df.head(5).iterrows():
        logger.info(f"  {row['Rank']}. {row['model']}: {row['ConsensusScore']}")
    
    return True

def main():
    """Main function to process JLLM rankings."""
    parser = argparse.ArgumentParser(
        description='Generate final rankings for SimBench evaluation'
    )
    parser.add_argument(
        '--jllm',
        type=str,
        help='Specific JLLM judge to process (e.g., gpt-4.1-mini)'
    )
    parser.add_argument(
        '--all',
        action='store_true',
        help='Process all known JLLM judges'
    )
    
    args = parser.parse_args()
    
    if not args.jllm and not args.all:
        parser.error('Either --jllm or --all must be specified')
    
    # Determine which JLLMs to process
    if args.all:
        jllms_to_process = JLLM_JUDGES
    else:
        jllms_to_process = [args.jllm]
    
    # Process each JLLM
    results = {}
    for jllm in jllms_to_process:
        success = process_jllm(jllm)
        results[jllm] = success
    
    # Print summary
    print("\n" + "="*60)
    print("PROCESSING SUMMARY")
    print("="*60)
    
    for jllm, success in results.items():
        status = "✓ SUCCESS" if success else "✗ FAILED"
        print(f"{jllm}: {status}")
    
    # Check if all successful
    if all(results.values()):
        print("\n✓ All rankings generated successfully!")
        return 0
    else:
        print("\n⚠ Some rankings failed to generate")
        return 1

if __name__ == "__main__":
    sys.exit(main())