#!/usr/bin/env python3
"""
Script to regenerate failed simulations and fix FAILED evaluations.
This version properly imports from the v01 directory and uses the correct API.
"""

import os
import sys
import json
import csv
import re
import time
from pathlib import Path
from typing import Dict, List, Tuple
import subprocess

# Add v01 to path for imports
sys.path.insert(0, '/home/hongyu/Documents/SimBench/scoring/v01')

def find_failed_evaluations(jllm_dir: str = "output_llms_gpt-4-1-nano") -> List[Dict]:
    """Find all failed evaluations by checking for FAILED in CSV files"""
    failed_cases = []
    
    base_path = Path(jllm_dir)
    if not base_path.exists():
        print(f"Directory {jllm_dir} not found")
        return failed_cases
    
    # Search all evaluation_scores.csv files
    for csv_file in base_path.rglob("evaluation_scores.csv"):
        with open(csv_file, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                # Check if any score is FAILED
                if (row.get('Score Document') == 'FAILED' or 
                    row.get('Score Reference') == 'FAILED' or 
                    row.get('Score Reference Document') == 'FAILED'):
                    
                    model = row.get('Test Model')
                    system = row.get('System')
                    round_name = row.get('Round')
                    
                    failed_cases.append({
                        'model': model,
                        'system': system,
                        'round': round_name,
                        'csv_path': str(csv_file),
                        'system_dir': str(csv_file.parent)
                    })
                    
                    print(f"Found failed: {model}/{system}/{round_name}")
    
    return failed_cases

def regenerate_using_pe_script(model: str, system: str, round_name: str) -> bool:
    """
    Regenerate a single response using the pe_gpt_generate_simulation module
    """
    print(f"  Regenerating {model}/{system}/{round_name}...")
    
    try:
        # Import after path is set
        import pe_gpt_generate_simulation
        
        # Fix model registry entries if needed
        if model == "mistral-nemo-12b-instruct":
            pe_gpt_generate_simulation.MODEL_REGISTRY[model] = {
                "provider": "nvidia",
                "model_id": "mistralai/mistral-nemo-12b-instruct"
            }
        elif model == "phi-3-medium-128k-instruct":
            pe_gpt_generate_simulation.MODEL_REGISTRY[model] = {
                "provider": "nvidia", 
                "model_id": "microsoft/phi-3-medium-128k-instruct"
            }
        
        # Get the chat completion function
        _chat_completion = pe_gpt_generate_simulation._chat_completion
        
        # Paths
        dataset_root = "/home/hongyu/Documents/SimBench/demo_data"
        output_root = "/home/hongyu/Documents/SimBench/output_llms"
        conv_root = "/home/hongyu/Documents/SimBench/output_conversion"
        
        # Load conversation
        conv_file = os.path.join(conv_root, f"{model}_{system}_conversation.json")
        if not os.path.exists(conv_file):
            print(f"    Conversation file not found: {conv_file}")
            return False
            
        with open(conv_file, 'r') as f:
            conversation = json.load(f)
        
        # Map round to conversation index
        round_map = {'first': 1, 'second': 3, 'third': 5}
        if round_name not in round_map:
            print(f"    Invalid round: {round_name}")
            return False
        
        round_idx = round_map[round_name]
        
        if round_idx >= len(conversation):
            print(f"    Round {round_name} not found in conversation")
            return False
        
        # Get the prompt for this round
        prompt = conversation[round_idx]["content"]
        
        print(f"    Calling API for {round_name} round...")
        
        # Get response with retries
        max_retries = 3
        response = None
        
        for attempt in range(max_retries):
            try:
                # Use _chat_completion which takes model_name and prompt
                response = _chat_completion(model, prompt)
                
                if response and not response.startswith("Error"):
                    break
                elif response and response.startswith("Error"):
                    print(f"    API returned error: {response[:100]}")
                    if attempt < max_retries - 1:
                        print(f"    Retrying in 5 seconds...")
                        time.sleep(5)
                    
            except Exception as e:
                print(f"    Attempt {attempt + 1} exception: {e}")
                if attempt < max_retries - 1:
                    time.sleep(5)
        
        if not response or response.startswith("Error"):
            print(f"    Failed to get valid response")
            return False
        
        # Save the response
        output_dir = os.path.join(output_root, model, system)
        os.makedirs(output_dir, exist_ok=True)
        
        response_file = os.path.join(output_dir, f"{round_name}_response.txt")
        with open(response_file, 'w', encoding='utf-8') as f:
            f.write(response)
        
        print(f"    ✓ Saved to {response_file}")
        
        # Also update the conversation file with the new response
        if round_idx + 1 < len(conversation):
            conversation[round_idx + 1] = {
                "role": "assistant",
                "content": response
            }
            with open(conv_file, 'w') as f:
                json.dump(conversation, f, indent=2)
            print(f"    ✓ Updated conversation file")
        
        return True
        
    except Exception as e:
        print(f"    Error in regeneration: {e}")
        import traceback
        traceback.print_exc()
        return False

def extract_python_code(model: str, system: str, round_name: str) -> bool:
    """Extract Python code from the regenerated response"""
    
    txt_file = f"/home/hongyu/Documents/SimBench/output_llms/{model}/{system}/{round_name}_response.txt"
    py_file = f"/home/hongyu/Documents/SimBench/output_llms/{model}/{system}/{round_name}_response.py"
    
    if not os.path.exists(txt_file):
        print(f"    Text file not found: {txt_file}")
        return False
    
    try:
        with open(txt_file, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # Check if it's an error
        if content.strip().startswith("Error code:"):
            print(f"    Response is still an error")
            return False
        
        python_code = ""
        
        # Extract code between ```python and ```
        matches = re.findall(r'```python(.*?)```', content, re.DOTALL)
        if matches:
            python_code = "\n\n".join(match.strip() for match in matches)
        else:
            # Try with just ``` markers
            matches = re.findall(r'```(.*?)```', content, re.DOTALL)
            if matches:
                # Filter for Python-looking code
                for match in matches:
                    if 'import' in match or 'def ' in match or 'class ' in match or '=' in match:
                        python_code += match.strip() + "\n\n"
            
            if not python_code:
                # Assume entire content is code
                python_code = content.strip()
        
        # Save extracted code
        with open(py_file, 'w', encoding='utf-8') as f:
            f.write(python_code)
        
        print(f"    ✓ Extracted Python code")
        return True
        
    except Exception as e:
        print(f"    Error extracting: {e}")
        return False

def rescore_with_jllm(model: str, system: str, round_name: str, jllm: str = "gpt-4-1-nano") -> bool:
    """Re-score using the JLLM judge"""
    
    print(f"    Re-scoring with {jllm}...")
    
    try:
        # Use the p_JLLM_score script directly via subprocess
        scoring_script = f"/home/hongyu/Documents/SimBench/scoring/v01/p_JLLM_score_{jllm.replace('.', '').replace('-', '')}.py"
        
        if not os.path.exists(scoring_script):
            # Try alternative naming
            scoring_script = f"/home/hongyu/Documents/SimBench/scoring/v01/p_JLLM_score_{jllm.replace('.', '-').replace('-', '')}.py"
        
        if not os.path.exists(scoring_script):
            print(f"    Scoring script not found: {scoring_script}")
            # Try direct import instead
            from p_JLLM_score_gpt41nano import evaluate_and_save_results, extract_scores_from_txt
            
            # Load files
            prediction_file = f"/home/hongyu/Documents/SimBench/output_llms/{model}/{system}/{round_name}_response.txt"
            reference_file = f"/home/hongyu/Documents/SimBench/demo_data/{system}/reference_{round_name}.py"
            api_file = f"/home/hongyu/Documents/SimBench/demo_data/{system}/api.py"
            output_dir = f"/home/hongyu/Documents/SimBench/output_llms_{jllm.replace('.', '-')}/{model}/{system}"
            
            with open(prediction_file, 'r') as f:
                prediction = f.read()
            with open(reference_file, 'r') as f:
                reference = f.read()
            with open(api_file, 'r') as f:
                api_code = f.read()
            
            os.makedirs(output_dir, exist_ok=True)
            
            # Call evaluation
            evaluate_and_save_results(round_name, prediction, reference, api_code, output_dir)
            
            # Check if scores were generated
            score_files = [
                f"{output_dir}/{round_name}_score_document.txt",
                f"{output_dir}/{round_name}_score_reference.txt",
                f"{output_dir}/{round_name}_score_reference_document.txt"
            ]
            
            for sf in score_files:
                if os.path.exists(sf):
                    score = extract_scores_from_txt(sf)
                    if score != -1:
                        print(f"      Score extracted: {score}")
                    
            print(f"    ✓ Re-scored successfully")
            return True
            
    except Exception as e:
        print(f"    Error scoring: {e}")
        import traceback
        traceback.print_exc()
        return False

def update_csv_with_scores(model: str, system: str, jllm: str = "gpt-4-1-nano"):
    """Update the CSV file with new scores"""
    
    csv_path = f"/home/hongyu/Documents/SimBench/output_llms_{jllm.replace('.', '-')}/{model}/{system}/evaluation_scores.csv"
    system_path = f"/home/hongyu/Documents/SimBench/output_llms_{jllm.replace('.', '-')}/{model}/{system}"
    
    print(f"    Updating CSV: {csv_path}")
    
    # Import the extraction function
    try:
        from p_JLLM_score_gpt41nano import extract_scores_from_txt
    except:
        # Fallback extraction
        def extract_scores_from_txt(file_path):
            if not os.path.exists(file_path):
                return "FAILED"
            with open(file_path, 'r') as f:
                content = f.read()
            if content.startswith("FAILED:") or content.startswith("Error:"):
                return "FAILED"
            match = re.search(r"\[\[(\d+)\]\]", content)
            if match:
                return int(match.group(1))
            return 0
    
    # Read existing CSV
    rows = []
    if os.path.exists(csv_path):
        with open(csv_path, 'r') as f:
            reader = csv.DictReader(f)
            rows = list(reader)
    
    # Update scores for each round
    rounds = ["first", "second", "third"]
    
    for round_name in rounds:
        # Extract new scores
        score_doc_file = os.path.join(system_path, f"{round_name}_score_document.txt")
        score_ref_file = os.path.join(system_path, f"{round_name}_score_reference.txt")
        score_ref_doc_file = os.path.join(system_path, f"{round_name}_score_reference_document.txt")
        
        score_doc = extract_scores_from_txt(score_doc_file)
        score_ref = extract_scores_from_txt(score_ref_file)
        score_ref_doc = extract_scores_from_txt(score_ref_doc_file)
        
        # Convert -1 to FAILED for consistency
        if score_doc == -1:
            score_doc = "FAILED"
        if score_ref == -1:
            score_ref = "FAILED"
        if score_ref_doc == -1:
            score_ref_doc = "FAILED"
        
        # Update or add row
        found = False
        for row in rows:
            if row.get('Round') == round_name:
                row['Score Document'] = str(score_doc)
                row['Score Reference'] = str(score_ref)
                row['Score Reference Document'] = str(score_ref_doc)
                found = True
                print(f"      Updated {round_name}: {score_doc}, {score_ref}, {score_ref_doc}")
                break
        
        if not found:
            rows.append({
                'Test Model': model,
                'System': system,
                'Round': round_name,
                'Score Document': str(score_doc),
                'Score Reference': str(score_ref),
                'Score Reference Document': str(score_ref_doc)
            })
            print(f"      Added {round_name}: {score_doc}, {score_ref}, {score_ref_doc}")
    
    # Write updated CSV
    with open(csv_path, 'w', newline='') as f:
        fieldnames = ['Test Model', 'System', 'Round', 'Score Document', 'Score Reference', 'Score Reference Document']
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    
    print(f"    ✓ CSV updated")

def main():
    """Main function"""
    print("="*60)
    print("REGENERATING FAILED SIMULATIONS")
    print("="*60)
    
    # Find failed evaluations
    print("\n1. Finding failed evaluations...")
    failed_cases = find_failed_evaluations()
    
    if not failed_cases:
        print("No failed evaluations found!")
        return
    
    print(f"\nFound {len(failed_cases)} failed evaluations")
    
    # Group by model
    failed_by_model = {}
    for case in failed_cases:
        model = case['model']
        if model not in failed_by_model:
            failed_by_model[model] = []
        failed_by_model[model].append(case)
    
    # Process each model
    total_success = 0
    
    for model, cases in failed_by_model.items():
        print(f"\n2. Processing {model} ({len(cases)} failures)...")
        
        for case in cases:
            system = case['system']
            round_name = case['round']
            
            print(f"\n  {model}/{system}/{round_name}:")
            
            # Step 1: Regenerate
            if regenerate_using_pe_script(model, system, round_name):
                # Step 2: Extract Python
                if extract_python_code(model, system, round_name):
                    # Step 3: Re-score
                    if rescore_with_jllm(model, system, round_name):
                        total_success += 1
            
            # Step 4: Update CSV regardless
            update_csv_with_scores(model, system)
            
            # Small delay between API calls
            time.sleep(2)
    
    # Summary
    print("\n" + "="*60)
    print(f"SUMMARY: Successfully fixed {total_success}/{len(failed_cases)} evaluations")
    print("="*60)
    
    if total_success == len(failed_cases):
        print("\n✓ All failed evaluations have been fixed!")
        print("You can now run: python generate_final_rankings.py --all")
    else:
        remaining = len(failed_cases) - total_success
        print(f"\n⚠ {remaining} evaluations still have issues")
        print("The CSV files have been updated. Failed entries remain as 'FAILED'.")

if __name__ == "__main__":
    main()