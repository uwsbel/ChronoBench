#!/usr/bin/env python3
"""
Regenerate the main combined_evaluation_scores CSV file for gpt-4.1-nano
using the fixed individual evaluation_scores.csv files.
"""

import os
import csv
import pandas as pd
from pathlib import Path

def regenerate_main_csv():
    """Combine all individual evaluation_scores.csv files into main CSV."""
    
    output_path = "/home/hongyu/Documents/SimBench/output_llms_gpt-4-1-nano"
    combined_csv_path = os.path.join(output_path, "combined_evaluation_scores_gpt-4.1-nano.csv")
    
    all_data = []
    models_processed = 0
    
    # Walk through all model directories
    for model_dir in os.listdir(output_path):
        model_path = os.path.join(output_path, model_dir)
        
        if not os.path.isdir(model_path):
            continue
            
        # Look for either fixed or original evaluation scores
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
            print(f"Processing {model_dir} from {os.path.basename(csv_file)}")
            try:
                df = pd.read_csv(csv_file)
                all_data.append(df)
                models_processed += 1
            except Exception as e:
                print(f"  Error reading {csv_file}: {e}")
    
    if all_data:
        # Combine all dataframes
        combined_df = pd.concat(all_data, ignore_index=True)
        
        # Sort by model and system for consistency
        if 'Test Model' in combined_df.columns and 'System' in combined_df.columns:
            combined_df = combined_df.sort_values(['Test Model', 'System', 'Round'])
        
        # Save the combined CSV
        combined_df.to_csv(combined_csv_path, index=False)
        print(f"\n✅ Successfully regenerated combined CSV with {len(combined_df)} rows")
        print(f"   Models processed: {models_processed}")
        print(f"   Output: {combined_csv_path}")
        
        # Show summary statistics
        print("\n📊 Summary Statistics:")
        if 'Score Document' in combined_df.columns:
            print(f"   Score Document: mean={combined_df['Score Document'].mean():.1f}, "
                  f"std={combined_df['Score Document'].std():.1f}")
        if 'Score Reference' in combined_df.columns:
            print(f"   Score Reference: mean={combined_df['Score Reference'].mean():.1f}, "
                  f"std={combined_df['Score Reference'].std():.1f}")
        if 'Score Reference Document' in combined_df.columns:
            print(f"   Score Reference Document: mean={combined_df['Score Reference Document'].mean():.1f}, "
                  f"std={combined_df['Score Reference Document'].std():.1f}")
        
        # Check for any remaining zeros that might be extraction failures
        zero_counts = {}
        for col in ['Score Document', 'Score Reference', 'Score Reference Document']:
            if col in combined_df.columns:
                zero_counts[col] = (combined_df[col] == 0).sum()
        
        print("\n⚠️ Zero Score Counts (may include legitimate zeros):")
        for col, count in zero_counts.items():
            percentage = (count / len(combined_df)) * 100
            print(f"   {col}: {count} ({percentage:.1f}%)")
        
    else:
        print("❌ No data found to combine")

if __name__ == "__main__":
    regenerate_main_csv()