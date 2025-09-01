#!/usr/bin/env python3
"""
Re-merge all individual evaluation_scores.csv files into combined files.
This fixes the issue where merge_csv_files was called before evaluations completed.
"""

import os
import csv
from pathlib import Path

def merge_csv_files(output_path, output_dir, judge_model, combined_csv_filename):
    """
    Merges all small CSV files from different models and systems into a single large CSV file.
    
    :param output_path: The root directory where all the model-specific directories are stored.
    :param output_dir: The output directory for this judge model
    :param judge_model: Name of the judge model (e.g., "gpt-4.1-nano")
    :param combined_csv_filename: The name of the resulting combined CSV file.
    """
    combined_csv_data = []
    models_processed = set()
    total_rows = 0
    
    # Iterate through all model directories
    for model_dir in os.listdir(output_path):
        model_path = os.path.join(output_path, model_dir)
        if os.path.isdir(model_path):
            # Iterate through all system directories within each model directory
            for system_dir in os.listdir(model_path):
                system_path = os.path.join(model_path, system_dir)
                if os.path.isdir(system_path):
                    # Path to the small CSV file
                    small_csv_path = os.path.join(system_path, "evaluation_scores.csv")
                    if os.path.exists(small_csv_path):
                        with open(small_csv_path, 'r', encoding="utf-8") as csvfile:
                            reader = csv.reader(csvfile)
                            headers = next(reader)
                            if not combined_csv_data:
                                # Add the header from the first file
                                combined_csv_data.append(headers)
                            # Append the rows from the current small CSV
                            rows = list(reader)
                            combined_csv_data.extend(rows)
                            total_rows += len(rows)
                            models_processed.add(model_dir)
    
    # Save the combined data into the output directory
    os.makedirs(output_dir, exist_ok=True)
    combined_csv_path = os.path.join(output_dir, combined_csv_filename)
    with open(combined_csv_path, 'w', newline='', encoding="utf-8") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerows(combined_csv_data)
    
    print(f"✓ Merged {total_rows} rows from {len(models_processed)} models")
    print(f"  Models: {', '.join(sorted(models_processed))}")
    print(f"  Saved to: {combined_csv_path}")
    
    return combined_csv_path

def main():
    # Base paths
    output_path = "/home/hongyu/Documents/SimBench/output_llms"
    scoring_dir = "/home/hongyu/Documents/SimBench/scoring/out_diff_models"
    
    # List of judge models to process
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
    print("Re-merging evaluation scores for all judge models")
    print("="*60)
    
    for judge_model in judge_models:
        print(f"\nProcessing judge model: {judge_model}")
        
        # Output directory for this judge model
        output_dir = os.path.join(scoring_dir, f"out_{judge_model.replace('.', '-')}")
        
        if not os.path.exists(output_dir):
            print(f"  ⚠️ Directory not found: {output_dir}")
            continue
        
        # Merge the CSV files
        filename = f"combined_evaluation_scores_{judge_model}.csv"
        merge_csv_files(output_path, output_dir, judge_model, filename)
    
    print("\n" + "="*60)
    print("Re-merge complete!")
    print("="*60)

if __name__ == "__main__":
    main()