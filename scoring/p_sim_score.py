import evaluate
from codebleu import calc_codebleu
import os
import json
from tqdm import tqdm
import pandas as pd
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path
import logging
import warnings
import re

# Suppress verbose logging and warnings
logging.getLogger("evaluate").setLevel(logging.ERROR)
logging.getLogger("codebleu").setLevel(logging.ERROR)
os.environ["HF_HUB_DISABLE_PROGRESS_BARS"] = "1"
warnings.filterwarnings("ignore", message=".*no reference data-flows extracted.*")
warnings.filterwarnings("ignore", message=".*data-flow match score degenerates to 0.*")

# Define paths
DATA_ROOT = Path("/home/hongyu/Documents/SimBench/")
dataset_path = DATA_ROOT / "demo_data"
output_path = DATA_ROOT / "output_llms"
output_statistic_path = DATA_ROOT / "statistic"

# Auto-discover models from output_llms directory
test_model_list = [d.name for d in output_path.iterdir() if d.is_dir()]
print(f"Discovered {len(test_model_list)} models in {output_path}")

system_list = [
    "art", "beam", "buckling", "cable", "car", "camera",
    "citybus", "curiosity", "feda", "gator", "gear",
    "gps_imu", "handler", "hmmwv", "kraz", "lidar", "m113",
    "man", "mass_spring_damper", "particles", "pendulum",
    "rigid_highway", "rigid_multipatches", "rotor", "scm",
    "scm_hill", "sedan", "sensros", "slider_crank",
    "tablecloth", "turtlebot", "uazbus", "veh_app", "vehros", "viper"
]

system_do_list = system_list

# Track skipped models/systems
skipped_evaluations = []

def is_error_response(content):
    """Check if the content is an error message instead of code."""
    if content is None:
        return True
    error_patterns = [
        "Error code:",
        "404",
        "Not Found",
        "API Error",
        "Internal Server Error",
        "function.*not found"
    ]
    content_lower = content.lower()
    return any(pattern.lower() in content_lower for pattern in error_patterns)

def read_script_safe(file_path):
    """Safely read a script file, return None if not exists or error."""
    if not os.path.exists(file_path):
        return None
    try:
        with open(file_path, "r", encoding="utf-8") as file:
            content = file.read()
            # Check if it's an error response
            if is_error_response(content):
                return None
            return content
    except Exception as e:
        return None

def check_response_files(output_system_path):
    """Check if response files exist and contain valid code."""
    response_files = ["first_response.py", "second_response.py", "third_response.py"]
    txt_response_files = ["first_response.txt", "second_response.txt", "third_response.txt"]
    
    # First check if .py files exist and are valid
    py_valid = True
    for f in response_files:
        content = read_script_safe(os.path.join(output_system_path, f))
        if content is None:
            py_valid = False
            break
    
    if py_valid:
        return "valid"
    
    # Check if .txt files exist and contain errors
    for f in txt_response_files:
        content = read_script_safe(os.path.join(output_system_path, f))
        if content and is_error_response(content):
            return "error"
    
    return "missing"

def evaluate_system(system_folder, model, output_model_path, dataset_path):
    """Evaluate a system for a given model."""
    try:
        rouge = evaluate.load('rouge')
        # Paths based on the current system and model
        system_folder_path = os.path.join(dataset_path, system_folder)
        output_system_path = os.path.join(output_model_path, system_folder)
        
        if not os.path.exists(system_folder_path):
            return None
        
        if not os.path.exists(output_system_path):
            skipped_evaluations.append(f"{model}/{system_folder}: Output directory missing")
            return None
        
        # Check if this model/system has valid response files
        response_status = check_response_files(output_system_path)
        if response_status == "error":
            skipped_evaluations.append(f"{model}/{system_folder}: API error in responses")
            return None
        elif response_status == "missing":
            # Try to read cleaned response files
            pass  # Continue to try reading cleaned files
        
        # Try to read cleaned response files
        predictions = []
        cleaned_files = ["first_cleaned_response.py", "second_cleaned_response.py", "third_cleaned_response.py"]
        
        for cf in cleaned_files:
            content = read_script_safe(os.path.join(output_system_path, cf))
            if content is None:
                # Try reading the non-cleaned version as fallback
                non_cleaned = cf.replace("_cleaned", "")
                content = read_script_safe(os.path.join(output_system_path, non_cleaned))
                if content is None:
                    skipped_evaluations.append(f"{model}/{system_folder}: Missing {cf}")
                    return None
            predictions.append(content)
        
        # Read reference files
        references = []
        ref_files = ['cleaned_truth1.py', 'cleaned_truth2.py', 'cleaned_truth3.py']
        
        for rf in ref_files:
            ref_path = os.path.join(system_folder_path, rf)
            content = read_script_safe(ref_path)
            if content is None:
                # Try non-cleaned version
                non_cleaned = rf.replace("cleaned_", "")
                content = read_script_safe(os.path.join(system_folder_path, non_cleaned))
                if content is None:
                    skipped_evaluations.append(f"{model}/{system_folder}: Missing reference {rf}")
                    return None
            references.append(content)
        
        # Calculate CodeBLEU
        codebleu_scores = []
        for ref, pred in zip(references, predictions):
            try:
                score = calc_codebleu([ref], [pred], lang="python")
                codebleu_scores.append(score)
            except Exception as e:
                # If codebleu fails, use zeros
                codebleu_scores.append({
                    'codebleu': 0,
                    'ngram_match_score': 0,
                    'weighted_ngram_match_score': 0,
                    'syntax_match_score': 0,
                    'dataflow_match_score': 0
                })
        
        # Calculate ROUGE
        rouge_scores = []
        for pred, ref in zip(predictions, references):
            try:
                score = rouge.compute(predictions=[pred], references=[ref])
                rouge_scores.append(score)
            except Exception as e:
                rouge_scores.append({
                    'rouge1': 0,
                    'rouge2': 0,
                    'rougeL': 0,
                    'rougeLsum': 0
                })
        
        # Prepare data for the DataFrame
        data = []
        for i, (codebleu, rouge_score) in enumerate(zip(codebleu_scores, rouge_scores), 1):
            row = {
                'model': model,
                'system': system_folder,
                'round': f'round_{i}',
                'codebleu': codebleu.get('codebleu', 0),
                'ngram_match_score': codebleu.get('ngram_match_score', 0),
                'weighted_ngram_match_score': codebleu.get('weighted_ngram_match_score', 0),
                'syntax_match_score': codebleu.get('syntax_match_score', 0),
                'dataflow_match_score': codebleu.get('dataflow_match_score', 0),
                'rouge1': rouge_score.get('rouge1', 0),
                'rouge2': rouge_score.get('rouge2', 0),
                'rougeL': rouge_score.get('rougeL', 0),
                'rougeLsum': rouge_score.get('rougeLsum', 0)
            }
            data.append(row)
        
        return data
        
    except Exception as e:
        skipped_evaluations.append(f"{model}/{system_folder}: {str(e)}")
        return None

def process_model_system_pair(model, system_folder):
    """Process a single model-system pair."""
    output_model_path = os.path.join(output_path, model)
    os.makedirs(output_model_path, exist_ok=True)
    return evaluate_system(system_folder, model, output_model_path, dataset_path)

if __name__ == '__main__':
    # Initialize an empty list to collect all data
    all_data = []
    
    # Parallel execution using ProcessPoolExecutor
    with ProcessPoolExecutor() as executor:
        futures = []
        for test_model in test_model_list:
            for system_folder in os.listdir(dataset_path):
                if system_folder in system_do_list:
                    futures.append(executor.submit(process_model_system_pair, test_model, system_folder))
        
        # Collecting the results with progress bar
        for future in tqdm(as_completed(futures), total=len(futures), desc="Evaluating"):
            try:
                result = future.result()
                if result is not None:
                    all_data.extend(result)
            except Exception as e:
                # Handle any unexpected errors
                pass
    
    # Convert the collected data into a DataFrame
    if all_data:
        df = pd.DataFrame(all_data)
        
        # Save the DataFrame to a single CSV file
        output_file = os.path.join(output_statistic_path, "evaluation_results.csv")
        os.makedirs(os.path.dirname(output_file), exist_ok=True)
        df.to_csv(output_file, index=False)
        
        print(f"\n✅ Results saved to {output_file}")
        print(f"📊 Successfully evaluated {len(df) // 3} model-system pairs")
    else:
        print("\n⚠️ No successful evaluations completed")
    
    # Print summary of skipped evaluations
    if skipped_evaluations:
        print(f"\n⚠️ Skipped {len(skipped_evaluations)} evaluations:")
        # Group by model
        skipped_by_model = {}
        for skip in skipped_evaluations:
            if "/" in skip:
                model = skip.split("/")[0]
                if model not in skipped_by_model:
                    skipped_by_model[model] = []
                skipped_by_model[model].append(skip)
        
        # Print models with most skips
        sorted_models = sorted(skipped_by_model.items(), key=lambda x: len(x[1]), reverse=True)
        print("\nTop models with skipped evaluations:")
        for model, skips in sorted_models[:5]:
            print(f"  {model}: {len(skips)} skipped")
        
        # Show some examples
        print("\nExample skip reasons:")
        for skip in skipped_evaluations[:5]:
            print(f"  - {skip}")
    
    print("\nFinished")