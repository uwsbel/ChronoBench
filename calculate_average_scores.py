#!/usr/bin/env python3
"""
Script to calculate average scores of 29 LLMs across 3 JLLMs
and save the results to ave_scores_ranked.csv
"""

import pandas as pd
import os
from pathlib import Path

def read_jllm_scores(jllm_path):
    """Read scores from a JLLM's all_scores_ranked.csv file"""
    df = pd.read_csv(jllm_path, skipinitialspace=True)
    df.columns = df.columns.str.strip()
    return df

def calculate_average_scores():
    """Calculate average scores across 3 JLLMs"""
    
    base_dir = Path("/home/hongyu/Documents/SimBench")
    
    jllm_paths = [
        base_dir / "output_llms_gpt-4o-mini" / "all_scores_ranked.csv",
        base_dir / "output_llms_gpt-4-1-mini" / "all_scores_ranked.csv", 
        base_dir / "output_llms_gpt-4-1-nano" / "all_scores_ranked.csv"
    ]
    
    jllm_names = ["gpt-4o-mini", "gpt-4-1-mini", "gpt-4-1-nano"]
    
    print("Reading JLLM score files...")
    dataframes = {}
    for path, name in zip(jllm_paths, jllm_names):
        if not path.exists():
            print(f"Warning: {path} does not exist")
            continue
        df = read_jllm_scores(path)
        dataframes[name] = df
        print(f"  - Loaded {name}: {len(df)} models")
    
    if not dataframes:
        print("Error: No JLLM score files found")
        return None
    
    all_models = set()
    for df in dataframes.values():
        all_models.update(df['model'].str.strip().tolist())
    
    print(f"\nFound {len(all_models)} unique models across all JLLMs")
    
    score_columns = [
        'ConsensusScore', 'Score Document', 'Score Reference', 
        'Score Reference Document', 'codebleu', 'ngram_match_score',
        'weighted_ngram_match_score', 'syntax_match_score', 'dataflow_match_score',
        'rouge1', 'rouge2', 'rougeL', 'rougeLsum'
    ]
    
    average_scores = []
    
    for model in sorted(all_models):
        model_scores = {'model': model}
        
        for col in score_columns:
            scores = []
            jllm_count = 0
            
            for jllm_name, df in dataframes.items():
                model_data = df[df['model'].str.strip() == model]
                if not model_data.empty and col in df.columns:
                    score_val = model_data[col].values[0]
                    scores.append(float(score_val))
                    jllm_count += 1
            
            if scores:
                model_scores[col] = sum(scores) / len(scores)
                model_scores[f'{col}_count'] = jllm_count
            else:
                model_scores[col] = 0
                model_scores[f'{col}_count'] = 0
        
        average_scores.append(model_scores)
    
    result_df = pd.DataFrame(average_scores)
    
    result_df = result_df.sort_values('ConsensusScore', ascending=False)
    result_df.reset_index(drop=True, inplace=True)
    result_df.index += 1
    result_df.index.name = 'Rank'
    
    output_columns = ['model'] + score_columns
    result_df = result_df[output_columns]
    
    return result_df

def main():
    """Main function"""
    print("Calculating average scores across 3 JLLMs...")
    print("=" * 60)
    
    result_df = calculate_average_scores()
    
    if result_df is None:
        print("Failed to calculate average scores")
        return
    
    output_dir = Path("/home/hongyu/Documents/SimBench/statistic")
    output_dir.mkdir(exist_ok=True)
    
    output_path = output_dir / "ave_scores_ranked.csv"
    
    result_df.to_csv(output_path, float_format='%.2f')
    
    print(f"\n✓ Results saved to: {output_path}")
    print(f"  Total models: {len(result_df)}")
    print(f"\nTop 10 models by average ConsensusScore:")
    print("-" * 60)
    
    for idx in result_df.index[:10]:
        row = result_df.loc[idx]
        print(f"  {idx:2d}. {row['model']:30s} - Score: {row['ConsensusScore']:6.2f}")
    
    print("\n✓ Script completed successfully!")

if __name__ == "__main__":
    main()