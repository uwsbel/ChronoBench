#!/usr/bin/env python3
"""
Create a final combined scores CSV that includes:
- JLLM scores (Score Document, Score Reference, Score Reference Document)
- Similarity metrics (CodeBLEU, ROUGE, etc.)
- Rankings based on consensus score
"""

import pandas as pd
import numpy as np
from pathlib import Path
import sys

def normalize_cols(df):
    """Normalize column names by stripping whitespace."""
    df = df.copy()
    df.columns = [c.strip() for c in df.columns]
    return df

def calculate_consensus_score(df):
    """Calculate consensus score based on available metrics."""
    # Prioritize JLLM scores for consensus
    score_cols = []
    
    # JLLM score columns
    for col in ['Score Reference Document', 'Score Reference', 'Score Document']:
        if col in df.columns:
            score_cols.append(col)
    
    # Add similarity metrics if available
    similarity_cols = ['codebleu', 'rouge1', 'rougeL']
    for col in similarity_cols:
        if col in df.columns:
            # Normalize similarity metrics to 0-100 scale
            if df[col].max() <= 1.0:
                df[col + '_scaled'] = df[col] * 100
                score_cols.append(col + '_scaled')
            else:
                score_cols.append(col)
    
    if score_cols:
        # Calculate mean of available scores
        df['ConsensusScore'] = df[score_cols].mean(axis=1, skipna=True)
        # Normalize to 0-100 range
        min_score = df['ConsensusScore'].min()
        max_score = df['ConsensusScore'].max()
        if max_score > min_score:
            df['ConsensusScore'] = ((df['ConsensusScore'] - min_score) / (max_score - min_score)) * 100
        else:
            df['ConsensusScore'] = 50.0  # Default if all scores are the same
    else:
        df['ConsensusScore'] = 0.0
    
    # Remove temporary scaled columns
    for col in df.columns:
        if col.endswith('_scaled'):
            df = df.drop(columns=[col])
    
    return df

def main():
    # Base directories
    base_dir = Path("/home/hongyu/Documents/SimBench/scoring/out_diff_models")
    output_llms_dir = Path("/home/hongyu/Documents/SimBench/output_llms")
    
    # Judge models to process
    judge_models = [
        "gpt-4.1-nano",
        "gpt-4.1-mini",
        "gpt-4.1",
        "gpt-4o-mini",
        "gpt-4o",
        "o3",
        "o4-mini"
    ]
    
    print("="*60)
    print("Creating Final Combined Scores with All Metrics")
    print("="*60)
    
    for judge_model in judge_models:
        print(f"\nProcessing judge model: {judge_model}")
        
        judge_dir = base_dir / f"out_{judge_model.replace('.', '-')}"
        if not judge_dir.exists():
            print(f"  ⚠️ Directory not found: {judge_dir}")
            continue
        
        # Load JLLM scores
        jllm_file = judge_dir / f"combined_evaluation_scores_{judge_model}.csv"
        if not jllm_file.exists():
            print(f"  ⚠️ JLLM scores file not found: {jllm_file}")
            continue
        
        # Load similarity metrics
        eval_file = judge_dir / "evaluation_results.csv"
        if not eval_file.exists():
            print(f"  ⚠️ Evaluation results file not found: {eval_file}")
            continue
        
        # Read the files
        jllm_df = normalize_cols(pd.read_csv(jllm_file))
        eval_df = normalize_cols(pd.read_csv(eval_file))
        
        # Rename columns for clarity
        if 'Test Model' in jllm_df.columns:
            jllm_df = jllm_df.rename(columns={'Test Model': 'model'})
        if 'model' in eval_df.columns:
            eval_df = eval_df.rename(columns={'model': 'model'})
        
        # Aggregate JLLM scores by model (average across all systems and rounds)
        jllm_numeric = ['Score Document', 'Score Reference', 'Score Reference Document']
        jllm_agg = jllm_df.groupby('model')[jllm_numeric].mean().round(2).reset_index()
        
        # Aggregate similarity metrics by model
        eval_numeric = [c for c in eval_df.columns if pd.api.types.is_numeric_dtype(eval_df[c])]
        eval_agg = eval_df.groupby('model')[eval_numeric].mean().round(2).reset_index()
        
        # Merge the dataframes
        merged_df = pd.merge(jllm_agg, eval_agg, on='model', how='outer')
        
        # Calculate consensus score
        merged_df = calculate_consensus_score(merged_df)
        
        # Sort by consensus score and assign ranks
        merged_df = merged_df.sort_values('ConsensusScore', ascending=False)
        merged_df['Rank'] = range(1, len(merged_df) + 1)
        
        # Round scores for better readability
        merged_df['ConsensusScore'] = merged_df['ConsensusScore'].round(2)
        
        # Reorder columns - put important ones first
        priority_cols = ['Rank', 'model', 'ConsensusScore', 
                        'Score Document', 'Score Reference', 'Score Reference Document',
                        'codebleu', 'ngram_match_score', 'weighted_ngram_match_score',
                        'syntax_match_score', 'dataflow_match_score',
                        'rouge1', 'rouge2', 'rougeL', 'rougeLsum']
        
        # Get columns that exist in the dataframe
        existing_priority = [c for c in priority_cols if c in merged_df.columns]
        remaining_cols = [c for c in merged_df.columns if c not in existing_priority]
        
        # Reorder
        merged_df = merged_df[existing_priority + remaining_cols]
        
        # Save the comprehensive file
        output_file = judge_dir / f"final_combined_scores_{judge_model}.csv"
        merged_df.to_csv(output_file, index=False, float_format='%.2f')
        
        print(f"  ✓ Created comprehensive scores file with {len(merged_df)} models")
        print(f"  ✓ Top 5 models by consensus score:")
        for i, row in merged_df.head(5).iterrows():
            print(f"    {row['Rank']}. {row['model']}: {row['ConsensusScore']:.2f}")
        print(f"  ✓ Saved to: {output_file}")
        
        # Also save a copy with simpler name for easy access
        simple_output = judge_dir / "all_scores_ranked.csv"
        merged_df.to_csv(simple_output, index=False, float_format='%.2f')
        print(f"  ✓ Also saved as: {simple_output}")
    
    print("\n" + "="*60)
    print("Final combined scores creation complete!")
    print("="*60)

if __name__ == "__main__":
    main()