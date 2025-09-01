#!/usr/bin/env python3
"""
Merge individual JLLM evaluation scores into a combined CSV with proper aggregation.
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

def merge_evaluations_for_judge(jllm_name):
    """Merge all evaluation scores for a specific JLLM judge."""
    
    # Map judge names to directory names
    dir_name = f"output_llms_{jllm_name.replace('.', '-')}"
    base_path = f"/home/hongyu/Documents/SimBench/{dir_name}"
    
    if not os.path.exists(base_path):
        logger.error(f"Directory not found: {base_path}")
        return None
    
    all_data = []
    
    # Walk through all model directories
    for model_dir in os.listdir(base_path):
        model_path = os.path.join(base_path, model_dir)
        
        if not os.path.isdir(model_path):
            continue
        
        # Skip the combined file itself
        if model_dir.endswith('.csv'):
            continue
            
        # Look for evaluation_scores.csv in each subdirectory
        for system_dir in os.listdir(model_path):
            system_path = os.path.join(model_path, system_dir)
            
            if not os.path.isdir(system_path):
                continue
            
            eval_file = os.path.join(system_path, "evaluation_scores.csv")
            
            if os.path.exists(eval_file):
                try:
                    df = pd.read_csv(eval_file)
                    # Add model name if not present
                    if 'Test Model' not in df.columns and 'model' not in df.columns:
                        df['Test Model'] = model_dir
                    all_data.append(df)
                except Exception as e:
                    logger.warning(f"Error reading {eval_file}: {e}")
    
    if not all_data:
        logger.error(f"No evaluation scores found for {jllm_name}")
        return None
    
    # Combine all dataframes
    combined_df = pd.concat(all_data, ignore_index=True)
    
    # Standardize column names
    if 'Test Model' in combined_df.columns:
        combined_df = combined_df.rename(columns={'Test Model': 'model'})
    
    # Aggregate by model (average across all systems and rounds)
    score_cols = ['Score Document', 'Score Reference', 'Score Reference Document']
    
    # Group by model and calculate mean
    aggregated = combined_df.groupby('model')[score_cols].mean().round(2).reset_index()
    
    # Save the aggregated results
    output_file = os.path.join(base_path, f"combined_evaluation_scores_{jllm_name}.csv")
    aggregated.to_csv(output_file, index=False)
    
    logger.info(f"Created combined scores for {jllm_name}: {output_file}")
    logger.info(f"  Total models: {len(aggregated)}")
    
    return output_file

def main():
    """Main function to process all JLLM judges."""
    
    judges = [
        "gpt-4.1-mini",
        "gpt-4o-mini"
    ]
    
    for judge in judges:
        print(f"\nProcessing {judge}...")
        try:
            output_file = merge_evaluations_for_judge(judge)
            if output_file:
                print(f"✅ Successfully created: {output_file}")
        except Exception as e:
            logger.error(f"Error processing {judge}: {e}")
            import traceback
            traceback.print_exc()

if __name__ == "__main__":
    main()