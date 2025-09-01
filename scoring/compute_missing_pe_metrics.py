#!/usr/bin/env python3
"""
Compute missing CodeBLEU and ROUGE metrics for pe_ models.
This script evaluates only the models that are missing from evaluation_results.csv
"""

import evaluate
from codebleu import calc_codebleu
import os
import json
from tqdm import tqdm
import pandas as pd
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path
import logging
logging.getLogger("evaluate").setLevel(logging.ERROR)
os.environ["HF_HUB_DISABLE_PROGRESS_BARS"] = "1"

# Define paths
DATA_ROOT                  = Path("/home/hongyu/Documents/SimBench/")
dataset_path               = DATA_ROOT / "demo_data"
output_path                = DATA_ROOT / "output_llms"
output_statistic_path      = DATA_ROOT / "statistic"

# Models that are missing metrics
missing_pe_models = [
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
    """Read a script file, return empty string if not found."""
    try:
        with open(file_path, "r", encoding="utf-8") as file:
            return file.read()
    except:
        return ""

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
        print(f"Skipping system {model}/{system_folder} due to missing files.")
        if "" in predictions:
            print(f"  Missing predictions in: {output_system_path}")
        if "" in references:
            print(f"  Missing references in: {system_folder_path}")
        return None
    
    # Calculate CodeBLEU
    codebleu_scores = [calc_codebleu([ref], [pred], lang="python") for ref, pred in zip(references, predictions)]
    
    # Calculate ROUGE
    rouge_scores = [rouge.compute(predictions=[pred], references=[ref]) for pred, ref in zip(predictions, references)]
    
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

def process_model_system_pair(model, system_folder):
    """Process a single model-system pair."""
    output_model_path = os.path.join(output_path, model)
    if not os.path.exists(output_model_path):
        print(f"Model output path not found: {output_model_path}")
        return None
    return evaluate_system(system_folder, model, output_model_path, dataset_path)

if __name__ == '__main__':
    print("=" * 60)
    print("Computing Missing PE Model Metrics")
    print("=" * 60)
    print(f"Models to process: {missing_pe_models}")
    print()
    
    # Initialize an empty list to collect all data
    all_data = []
    
    # Process each missing model
    for model in missing_pe_models:
        print(f"\nProcessing model: {model}")
        model_data = []
        
        # Check if model directory exists
        model_path = os.path.join(output_path, model)
        if not os.path.exists(model_path):
            print(f"  WARNING: Model output directory not found: {model_path}")
            continue
        
        # Process each system for this model (sequentially to avoid tree-sitter issues)
        for system_folder in tqdm(system_list, desc=f"  {model}"):
            result = process_model_system_pair(model, system_folder)
            if result is not None:
                model_data.extend(result)
        
        print(f"  Collected {len(model_data)} metric entries for {model}")
        all_data.extend(model_data)
    
    if not all_data:
        print("\nNo data collected. Check if the model directories and files exist.")
        exit(1)
    
    # Convert the collected data into a DataFrame
    df = pd.DataFrame(all_data)
    
    # Save to a separate file first
    output_file = os.path.join(output_statistic_path, "evaluation_results_missing_pe.csv")
    os.makedirs(os.path.dirname(output_file), exist_ok=True)
    df.to_csv(output_file, index=False)
    print(f"\nResults saved to {output_file}")
    
    # Now merge with existing evaluation_results.csv
    existing_file = os.path.join(output_statistic_path, "evaluation_results.csv")
    if os.path.exists(existing_file):
        print(f"\nMerging with existing {existing_file}")
        existing_df = pd.read_csv(existing_file)
        
        # Remove any existing entries for these models (in case of partial data)
        existing_df = existing_df[~existing_df['model'].isin(missing_pe_models)]
        
        # Concatenate the dataframes
        merged_df = pd.concat([existing_df, df], ignore_index=True)
        
        # Sort by model and system for consistency
        merged_df = merged_df.sort_values(['model', 'system', 'round']).reset_index(drop=True)
        
        # Save the merged result
        merged_df.to_csv(existing_file, index=False)
        print(f"Updated {existing_file} with {len(df)} new entries")
        print(f"Total entries: {len(merged_df)}")
    else:
        print(f"Warning: {existing_file} not found. Saved new data separately.")
    
    print("\n" + "=" * 60)
    print("Finished computing missing metrics")
    print("=" * 60)