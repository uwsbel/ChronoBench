#!/usr/bin/env python3
"""
Script to fix failed simulations by:
1. Identifying failed evaluations (FAILED values in CSV)
2. Re-running only those specific simulations
3. Extracting Python code from responses
4. Re-scoring the regenerated responses
"""

import os
import sys
import json
import csv
import re
import time
from pathlib import Path
from typing import Dict, List, Tuple
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

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

def check_model_availability(model: str) -> bool:
    """Check if a model is available on NVIDIA API"""
    try:
        from openai import OpenAI
        
        # Map model names to correct NVIDIA model IDs
        model_id_mapping = {
            "mistral-nemo-12b-instruct": "nv-mistralai/mistral-nemo-12b-instruct",
            "phi-3-medium-128k-instruct": "microsoft/Phi-3-medium-128k-instruct"  # Note capital P
        }
        
        model_id = model_id_mapping.get(model)
        if not model_id:
            return True  # Assume other models are OK
        
        api_key = os.getenv("NVIDIA_API_KEY")
        if not api_key:
            print(f"    Warning: NVIDIA_API_KEY not found in environment")
            return False
            
        client = OpenAI(
            base_url="https://integrate.api.nvidia.com/v1",
            api_key=api_key,
            timeout=10
        )
        
        # Test with a minimal request
        response = client.chat.completions.create(
            model=model_id,
            messages=[{"role": "user", "content": "test"}],
            max_tokens=1,
            temperature=0.1
        )
        
        print(f"    Model {model} is available")
        return True
        
    except Exception as e:
        error_str = str(e)
        if "404" in error_str or "500" in error_str:
            print(f"    Model {model} is NOT available: {error_str[:100]}")
            return False
        else:
            # Other errors might be transient
            print(f"    Model check inconclusive: {error_str[:100]}")
            return True

def regenerate_using_pe_script(model: str, system: str, round_name: str) -> bool:
    """
    Regenerate a single response using the pe_gpt_generate_simulation module
    """
    print(f"  Regenerating {model}/{system}/{round_name}...")
    
    # Check model availability first
    if not check_model_availability(model):
        print(f"    Skipping - model not available")
        return False
    
    try:
        # Import after path is set
        import pe_gpt_generate_simulation
        
        # Fix model registry entries with correct NVIDIA model IDs
        if model == "mistral-nemo-12b-instruct":
            pe_gpt_generate_simulation.MODEL_REGISTRY[model] = {
                "provider": "nvidia",
                "model_id": "nv-mistralai/mistral-nemo-12b-instruct"
            }
        elif model == "phi-3-medium-128k-instruct":
            pe_gpt_generate_simulation.MODEL_REGISTRY[model] = {
                "provider": "nvidia", 
                "model_id": "microsoft/Phi-3-medium-128k-instruct"  # Correct capitalization
            }
        
        # Paths
        dataset_root = "/home/hongyu/Documents/SimBench/demo_data"
        output_root = "/home/hongyu/Documents/SimBench/output_llms"
        
        # Read the instruction files from demo_data
        # Files are named input1.txt, input2.txt, input3.txt
        round_to_file = {'first': 'input1.txt', 'second': 'input2.txt', 'third': 'input3.txt'}
        
        instruction_file = os.path.join(dataset_root, system, round_to_file[round_name])
        if not os.path.exists(instruction_file):
            print(f"    Instruction file not found: {instruction_file}")
            return False
        
        with open(instruction_file, 'r') as f:
            instruction = f.read()
        
        print(f"    Calling API for {round_name} round...")
        
        # Get response with retries
        max_retries = 3
        response = None
        
        for attempt in range(max_retries):
            try:
                if round_name == 'first':
                    # For first round, use generate_first_code (returns tuple of prompt and response)
                    _, response = pe_gpt_generate_simulation.generate_first_code(instruction, model)
                else:
                    # For second/third rounds, need the previous code
                    prev_round = 'first' if round_name == 'second' else 'second'
                    prev_response_file = os.path.join(output_root, model, system, f"{prev_round}_response.txt")
                    
                    if os.path.exists(prev_response_file):
                        with open(prev_response_file, 'r') as f:
                            prev_code = f.read()
                        
                        # Check if previous response is an error
                        if prev_code.startswith("Error"):
                            print(f"    Previous response {prev_round} is an error, cannot proceed")
                            return False
                    else:
                        # Try to get from cleaned response
                        prev_clean_file = os.path.join(output_root, model, system, f"{prev_round}_response.py")
                        if os.path.exists(prev_clean_file):
                            with open(prev_clean_file, 'r') as f:
                                prev_code = f.read()
                        else:
                            print(f"    Previous response not found for {prev_round}")
                            return False
                    
                    # Use generate_second_third_code (returns tuple of prompt and response)
                    _, response = pe_gpt_generate_simulation.generate_second_third_code(instruction, prev_code, model)
                
                if response and not str(response).startswith("Error"):
                    break
                elif response and str(response).startswith("Error"):
                    print(f"    API returned error: {str(response)[:100]}")
                    if attempt < max_retries - 1:
                        print(f"    Retrying in 10 seconds...")
                        time.sleep(10)
                    
            except Exception as e:
                print(f"    Attempt {attempt + 1} exception: {e}")
                if attempt < max_retries - 1:
                    time.sleep(5)
        
        if not response or str(response).startswith("Error"):
            print(f"    Failed to get valid response after {max_retries} attempts")
            return False
        
        # Save the response
        output_dir = os.path.join(output_root, model, system)
        os.makedirs(output_dir, exist_ok=True)
        
        response_file = os.path.join(output_dir, f"{round_name}_response.txt")
        with open(response_file, 'w', encoding='utf-8') as f:
            f.write(response)
        
        print(f"    ✓ Saved to {response_file}")
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
        from p_JLLM_score_gpt41nano import evaluate_and_save_results
        
        # Load files
        prediction_file = f"/home/hongyu/Documents/SimBench/output_llms/{model}/{system}/{round_name}_response.txt"
        
        # For reference, need to use the truth files from demo_data
        round_to_truth = {'first': 'truth1.py', 'second': 'truth2.py', 'third': 'truth3.py'}
        reference_file = f"/home/hongyu/Documents/SimBench/demo_data/{system}/{round_to_truth[round_name]}"
        
        if not os.path.exists(reference_file):
            # Try cleaned_truth files
            round_to_cleaned = {'first': 'cleaned_truth1.py', 'second': 'cleaned_truth2.py', 'third': 'cleaned_truth3.py'}
            reference_file = f"/home/hongyu/Documents/SimBench/demo_data/{system}/{round_to_cleaned[round_name]}"
        
        # API file doesn't exist in this structure, use empty
        api_code = ""
        
        output_dir = f"/home/hongyu/Documents/SimBench/output_llms_{jllm.replace('.', '-')}/{model}/{system}"
        
        if not os.path.exists(prediction_file):
            print(f"    Prediction file not found: {prediction_file}")
            return False
            
        if not os.path.exists(reference_file):
            print(f"    Reference file not found: {reference_file}")
            return False
        
        with open(prediction_file, 'r') as f:
            prediction = f.read()
        with open(reference_file, 'r') as f:
            reference = f.read()
        
        os.makedirs(output_dir, exist_ok=True)
        
        # Call evaluation
        evaluate_and_save_results(round_name, prediction, reference, api_code, output_dir)
        
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
    print("FIXING FAILED SIMULATIONS")
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
        print("These will remain as 'FAILED' in the CSV files.")

if __name__ == "__main__":
    main()