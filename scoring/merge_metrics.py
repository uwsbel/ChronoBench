#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Merge evaluation_results.csv and combined_evaluation_scores.csv
合并两个评估结果文件，增加所有指标列
"""

import pandas as pd
from pathlib import Path

# Auto-detect project root
SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent

# Input files
EVAL_RESULTS = PROJECT_ROOT / "statistic" / "evaluation_results.csv"
COMBINED_SCORES = PROJECT_ROOT / "output_llms" / "combined_evaluation_scores.csv"

# Output file
OUTPUT_FILE = PROJECT_ROOT / "statistic" / "all_metrics_combined.csv"

def main():
    print("=" * 60)
    print("Merging evaluation metrics...")
    print("=" * 60)
    
    # Load both files
    df1 = pd.read_csv(EVAL_RESULTS)
    df2 = pd.read_csv(COMBINED_SCORES)
    
    print(f"\nevaluation_results.csv: {len(df1)} rows, {len(df1.columns)} columns")
    print(f"  Columns: {list(df1.columns)}")
    
    print(f"\ncombined_evaluation_scores.csv: {len(df2)} rows, {len(df2.columns)} columns")
    print(f"  Columns: {list(df2.columns)}")
    
    # Normalize df1 keys
    df1['round_norm'] = df1['round'].str.replace('round_', '')
    df1['key'] = df1['model'] + '|' + df1['system'] + '|' + df1['round_norm']
    
    # Normalize df2 keys
    round_map = {'first': '1', 'second': '2', 'third': '3'}
    df2['round_norm'] = df2['Round'].map(round_map)
    df2['key'] = df2['Test Model'] + '|' + df2['System'] + '|' + df2['round_norm']
    
    # Rename df2 columns for clarity
    df2_renamed = df2.rename(columns={
        'Test Model': 'model',
        'System': 'system', 
        'Round': 'round',
        'Score Document': 'score_document',
        'Score Reference': 'score_reference',
        'Score Reference Document': 'score_reference_document'
    })
    
    # Select columns to merge from df2
    df2_to_merge = df2_renamed[['key', 'score_document', 'score_reference', 'score_reference_document']]
    
    # Merge on key
    merged = df1.merge(df2_to_merge, on='key', how='outer')
    
    # Clean up - drop helper columns
    merged = merged.drop(columns=['key', 'round_norm'])
    
    # Reorder columns - put identifiers first, then all metrics
    id_cols = ['model', 'system', 'round']
    metric_cols = [c for c in merged.columns if c not in id_cols]
    merged = merged[id_cols + metric_cols]
    
    # Sort by model, system, round
    round_order = {'round_1': 1, 'round_2': 2, 'round_3': 3}
    merged['_sort'] = merged['round'].map(round_order)
    merged = merged.sort_values(['model', 'system', '_sort']).drop(columns=['_sort'])
    merged = merged.reset_index(drop=True)
    
    # Save
    merged.to_csv(OUTPUT_FILE, index=False)
    
    print(f"\n{'=' * 60}")
    print(f"Merged file saved: {OUTPUT_FILE}")
    print(f"Total rows: {len(merged)}")
    print(f"Total columns: {len(merged.columns)}")
    print(f"{'=' * 60}")
    
    print(f"\nFinal columns:")
    for i, col in enumerate(merged.columns, 1):
        print(f"  {i:2}. {col}")
    
    # Show sample
    print(f"\nSample data (first 5 rows):")
    print(merged.head().to_string())
    
    return merged

if __name__ == "__main__":
    main()
