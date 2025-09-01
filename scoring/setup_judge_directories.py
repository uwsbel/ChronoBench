#!/usr/bin/env python3
"""
Setup judge-specific directories with model response files.
This copies only the response files (not evaluation scores) from the main output_llms
directory to judge-specific directories.
"""

import os
import shutil
from pathlib import Path

def setup_judge_directories():
    """Setup directories for each judge model with response files"""
    
    # Define paths
    base_output = "/home/hongyu/Documents/SimBench/output_llms"
    
    # Judge models to setup
    judges = ["gpt-4-1-nano", "gpt-4-1-mini", "gpt-4o-mini"]
    
    # Files to copy (responses only, not scores)
    response_files = [
        "first_response.py", 
        "second_response.py", 
        "third_response.py",
        "first_response.txt",
        "second_response.txt", 
        "third_response.txt",
        "first_cleaned_response.py",
        "second_cleaned_response.py",
        "third_cleaned_response.py"
    ]
    
    for judge in judges:
        judge_output = f"/home/hongyu/Documents/SimBench/output_llms_{judge}"
        print(f"\nSetting up directory for {judge}...")
        print(f"  Target: {judge_output}")
        
        # Create judge directory if it doesn't exist
        os.makedirs(judge_output, exist_ok=True)
        
        # Copy model directories
        if os.path.exists(base_output):
            for model_dir in os.listdir(base_output):
                model_path = os.path.join(base_output, model_dir)
                
                # Skip if not a directory or if it's a CSV file
                if not os.path.isdir(model_path):
                    continue
                    
                # Create model directory in judge output
                judge_model_path = os.path.join(judge_output, model_dir)
                os.makedirs(judge_model_path, exist_ok=True)
                
                # Copy system directories
                for system_dir in os.listdir(model_path):
                    system_path = os.path.join(model_path, system_dir)
                    
                    if not os.path.isdir(system_path):
                        continue
                    
                    # Create system directory in judge output
                    judge_system_path = os.path.join(judge_model_path, system_dir)
                    os.makedirs(judge_system_path, exist_ok=True)
                    
                    # Copy only response files, not evaluation scores
                    files_copied = 0
                    for response_file in response_files:
                        src_file = os.path.join(system_path, response_file)
                        dst_file = os.path.join(judge_system_path, response_file)
                        
                        if os.path.exists(src_file):
                            # Only copy if destination doesn't exist
                            if not os.path.exists(dst_file):
                                shutil.copy2(src_file, dst_file)
                                files_copied += 1
                    
                    if files_copied > 0:
                        print(f"    Copied {files_copied} response files for {model_dir}/{system_dir}")
        
        print(f"  Setup complete for {judge}")
    
    print("\n" + "="*60)
    print("Judge directory setup complete!")
    print("="*60)
    print("\nNext steps:")
    print("1. Run each JLLM evaluation script to generate unique scores:")
    print("   python /home/hongyu/Documents/SimBench/scoring/v01/p_JLLM_score_gpt41nano.py")
    print("   python /home/hongyu/Documents/SimBench/scoring/v01/p_JLLM_score_gpt41mini.py")
    print("   python /home/hongyu/Documents/SimBench/scoring/v01/p_JLLM_score_gpt4omini.py")
    print("\n2. Then run create_final_combined_scores.py to generate unique rankings")

if __name__ == "__main__":
    setup_judge_directories()