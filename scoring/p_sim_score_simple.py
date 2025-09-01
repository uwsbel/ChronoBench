#!/usr/bin/env python3
"""
Simplified version of p_sim_score.py that computes metrics for missing pe_ models.
Falls back to ROUGE-only if CodeBLEU fails due to tree-sitter issues.
"""

import evaluate
import os
import json
from tqdm import tqdm
import pandas as pd
from pathlib import Path
import logging
import warnings
warnings.filterwarnings("ignore")
logging.getLogger("evaluate").setLevel(logging.ERROR)
os.environ["HF_HUB_DISABLE_PROGRESS_BARS"] = "1"

# Try to import codebleu, but continue if it fails
try:
    from codebleu import calc_codebleu
    CODEBLEU_AVAILABLE = True
except Exception as e:
    print(f"Warning: CodeBLEU not available: {e}")
    print("Will compute ROUGE metrics only")
    CODEBLEU_AVAILABLE = False

# Define paths
DATA_ROOT = Path("/home/hongyu/Documents/SimBench/")
dataset_path = DATA_ROOT / "demo_data"
output_path = DATA_ROOT / "output_llms"
output_statistic_path = DATA_ROOT / "statistic"

# Only process the missing pe_ models
test_model_list = [
    "pe_deepseek-r1-8b",
    "pe_llama-3.1-70b-instruct",
    "pe_llama4_maverick"
]

system_list = [
    "art", "beam", "buckling", "cable", "camera",
    "citybus", "curiosity", "feda", "gator", "gear",
    "gps_imu", "handler", "hmmwv", "kraz", "lidar", "m113",
    "man", "mass_spring_damper", "particles", "pendulum",
    "rigid_highway", "rigid_multipatches", "rotor", "scm",
    "scm_hill", "sedan", "sensros", "slider_crank",
    "tablecloth", "turtlebot", "uazbus", "veh_app", "vehros", "viper"
]

def read_script(file_path):
    """Read a script file safely."""
    try:
        with open(file_path, "r", encoding="utf-8") as file:
            return file.read()
    except:
        return ""

def safe_calc_codebleu(references, predictions):
    """Safely calculate CodeBLEU scores with fallback."""
    if not CODEBLEU_AVAILABLE:
        return [{'codebleu': None, 'ngram_match_score': None, 
                'weighted_ngram_match_score': None, 'syntax_match_score': None,
                'dataflow_match_score': None} for _ in references]
    
    try:
        scores = []
        for ref, pred in zip(references, predictions):
            try:
                score = calc_codebleu([ref], [pred], lang="python")
                scores.append(score)
            except Exception as e:
                # If individual calculation fails, return None values
                scores.append({'codebleu': None, 'ngram_match_score': None,
                             'weighted_ngram_match_score': None, 'syntax_match_score': None,
                             'dataflow_match_score': None})
        return scores
    except Exception as e:
        print(f"CodeBLEU calculation failed: {e}")
        return [{'codebleu': None, 'ngram_match_score': None,
                'weighted_ngram_match_score': None, 'syntax_match_score': None,
                'dataflow_match_score': None} for _ in references]

def evaluate_system(system_folder, model, output_model_path, dataset_path):
    """Evaluate a single system for a model."""
    rouge = evaluate.load('rouge')
    
    # Paths based on the current system and model
    system_folder_path = os.path.join(dataset_path, system_folder)
    output_system_path = os.path.join(output_model_path, system_folder)
    
    if not os.path.exists(system_folder_path):
        print(f"System folder not found: {system_folder_path}")
        return None
    
    if not os.path.exists(output_system_path):
        print(f"Output folder not found: {output_system_path}")
        return None
    
    # Read predictions and references
    predictions = [
        read_script(os.path.join(output_system_path, "first_cleaned_response.py")),
        read_script(os.path.join(output_system_path, "second_cleaned_response.py")),
        read_script(os.path.join(output_system_path, "third_cleaned_response.py"))
    ]
    
    references = [
        read_script(os.path.join(system_folder_path, 'cleaned_truth1.py')),
        read_script(os.path.join(system_folder_path, 'cleaned_truth2.py')),
        read_script(os.path.join(system_folder_path, 'cleaned_truth3.py'))
    ]
    
    if "" in predictions + references:
        print(f"Skipping {model}/{system_folder} due to missing files.")
        return None
    
    # Calculate CodeBLEU (with fallback)
    codebleu_scores = safe_calc_codebleu(references, predictions)
    
    # Calculate ROUGE (this should always work)
    rouge_scores = [rouge.compute(predictions=[pred], references=[ref]) 
                   for pred, ref in zip(predictions, references)]
    
    # Prepare data for the DataFrame
    data = []
    for i, (codebleu, rouge) in enumerate(zip(codebleu_scores, rouge_scores), 1):
        row = {
            'model': model,
            'system': system_folder,
            'round': f'round_{i}',
            'codebleu': codebleu.get('codebleu'),
            'ngram_match_score': codebleu.get('ngram_match_score'),
            'weighted_ngram_match_score': codebleu.get('weighted_ngram_match_score'),
            'syntax_match_score': codebleu.get('syntax_match_score'),
            'dataflow_match_score': codebleu.get('dataflow_match_score'),
            'rouge1': rouge.get('rouge1'),
            'rouge2': rouge.get('rouge2'),
            'rougeL': rouge.get('rougeL'),
            'rougeLsum': rouge.get('rougeLsum')
        }
        data.append(row)
    
    return data

def main():
    print("=" * 60)
    print("Simplified SimBench Evaluation for Missing PE Models")
    print("=" * 60)
    print(f"Models to process: {test_model_list}")
    print(f"CodeBLEU available: {CODEBLEU_AVAILABLE}")
    print()
    
    # Initialize an empty list to collect all data
    all_data = []
    
    # Process each model
    for model in test_model_list:
        print(f"\nProcessing model: {model}")
        output_model_path = os.path.join(output_path, model)
        
        if not os.path.exists(output_model_path):
            print(f"  WARNING: Model directory not found: {output_model_path}")
            continue
        
        model_data = []
        # Process each system sequentially
        for system_folder in tqdm(system_list, desc=f"  {model}"):
            result = evaluate_system(system_folder, model, output_model_path, dataset_path)
            if result is not None:
                model_data.extend(result)
        
        print(f"  Collected {len(model_data)} entries for {model}")
        all_data.extend(model_data)
    
    if not all_data:
        print("\nNo data collected. Check if model directories exist.")
        return 1
    
    # Convert to DataFrame
    df = pd.DataFrame(all_data)
    
    # Save to a temporary file first
    temp_file = os.path.join(output_statistic_path, "evaluation_results_pe_missing.csv")
    df.to_csv(temp_file, index=False)
    print(f"\nSaved {len(df)} entries to {temp_file}")
    
    # Merge with existing evaluation_results.csv
    main_file = os.path.join(output_statistic_path, "evaluation_results.csv")
    if os.path.exists(main_file):
        print(f"\nMerging with {main_file}")
        existing_df = pd.read_csv(main_file)
        
        # Remove any existing entries for these models
        existing_df = existing_df[~existing_df['model'].isin(test_model_list)]
        
        # Concatenate
        merged_df = pd.concat([existing_df, df], ignore_index=True)
        
        # Sort and save
        merged_df = merged_df.sort_values(['model', 'system', 'round']).reset_index(drop=True)
        
        # Backup the original
        backup_file = main_file.replace('.csv', '_backup.csv')
        existing_df_original = pd.read_csv(main_file)
        existing_df_original.to_csv(backup_file, index=False)
        print(f"Backed up original to {backup_file}")
        
        # Save the merged version
        merged_df.to_csv(main_file, index=False)
        print(f"Updated {main_file} - Total entries: {len(merged_df)}")
        
        # Report statistics
        models_in_file = merged_df['model'].nunique()
        print(f"\nStatistics:")
        print(f"  Total models: {models_in_file}")
        print(f"  New entries added: {len(df)}")
        print(f"  Total entries: {len(merged_df)}")
    else:
        print(f"Warning: {main_file} not found. Saved new data to {temp_file}")
    
    print("\n" + "=" * 60)
    print("Evaluation complete!")
    print("Next step: Run update_all_jllm_metrics.py to update all JLLM directories")
    print("=" * 60)
    return 0

if __name__ == '__main__':
    import sys
    sys.exit(main())