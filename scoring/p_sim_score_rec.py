import evaluate
from codebleu import calc_codebleu
import os
import json
from tqdm import tqdm
import pandas as pd
from concurrent.futures import ProcessPoolExecutor, as_completed
from datetime import datetime

# Define paths
dataset_path = r'/home/hongyu/Documents/andy_simbench/SimBench/demo_data'
output_path = r'/home/hongyu/Documents/andy_simbench/SimBench/output_llms'
output_statistic_path = r'/home/hongyu/Documents/andy_simbench/SimBench/statistic'

# List of models and systems to evaluate
test_model_list = [
    "deepseek-r1",
    "deepseek-r1-32b",
    "deepseek-r1-8b",
    "gemma-2-27b-it",
    "gemma-2-2b-it",
    "gemma-3-1b-it",
    "gemma-3-27b-it",
    "gpt-4.1-mini",
    "gpt-4.1-nano",
    "gpt-4o-mini",
    "llama-3.1-405b-instruct",
    "llama-3.1-70b-instruct",
    "llama-3.1-8b-instruct",
    "llama-3.3-70b-instruct",
    "llama4_maverick",
    "llama4_scout",
    "mamba-codestral-7b-v0.1",
    "mistral-large-latest",
    "mistral-nemo-12b-instruct",
    "mixtral-8x22b-instruct-v0.1",
    "mixtral-8x7b-instruct-v0.1",
    "nemotron-4-340b-instruct",
    "phi-3-medium-128k-instruct",
    "phi-3-mini-128k-instruct",
    "qwen3-235b-a22b",
    "codestral-22b-instruct-v0.1"
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

system_do_list = system_list

# Track missing implementations
missing_implementations = []

def read_script(file_path):
    """Read script file with error handling."""
    try:
        with open(file_path, "r", encoding="utf-8") as file:
            return file.read()
    except FileNotFoundError:
        return ""
    except Exception as e:
        print(f"Error reading {file_path}: {e}")
        return ""

def evaluate_system(system_folder, model, output_model_path, dataset_path):
    """Evaluate a system for a given model and track missing implementations."""
    rouge = evaluate.load('rouge')
    # Paths based on the current system and model
    system_folder_path = os.path.join(dataset_path, system_folder)
    output_system_path = os.path.join(output_model_path, system_folder)

    if not os.path.exists(system_folder_path):
        print(f"System folder not found: {system_folder_path}")
        return None, None

    # Track missing files for this model-system pair
    missing_files = []
    
    # Check and read predictions
    predictions = []
    prediction_files = [
        ("first_cleaned_response.py", "round_1"),
        ("second_cleaned_response.py", "round_2"),
        ("third_cleaned_response.py", "round_3")
    ]
    
    for filename, round_name in prediction_files:
        file_path = os.path.join(output_system_path, filename)
        content = read_script(file_path)
        predictions.append(content)
        if content == "":
            missing_files.append({
                'model': model,
                'system': system_folder,
                'round': round_name,
                'missing_file': filename,
                'file_path': file_path
            })

    # Read references
    references = [
        read_script(os.path.join(system_folder_path, 'cleaned_truth1.py')),
        read_script(os.path.join(system_folder_path, 'cleaned_truth2.py')),
        read_script(os.path.join(system_folder_path, 'cleaned_truth3.py'))
    ]

    # If any files are missing, record it but still try to process available data
    if missing_files:
        print(f"Missing files for {model}/{system_folder}: {[mf['missing_file'] for mf in missing_files]}")
    
    # If all predictions or references are missing, skip evaluation
    if all(p == "" for p in predictions) or all(r == "" for r in references):
        print(f"Skipping evaluation for {model}/{system_folder} due to all missing files.")
        return None, missing_files

    # Calculate scores only for available pairs
    data = []
    for i, (pred, ref) in enumerate(zip(predictions, references), 1):
        if pred != "" and ref != "":
            try:
                # Calculate CodeBLEU
                codebleu = calc_codebleu([ref], [pred], lang="python")
                
                # Calculate ROUGE
                rouge_score = rouge.compute(predictions=[pred], references=[ref])
                
                row = {
                    'model': model,
                    'system': system_folder,
                    'round': f'round_{i}',
                    'codebleu': codebleu.get('codebleu'),
                    'ngram_match_score': codebleu.get('ngram_match_score'),
                    'weighted_ngram_match_score': codebleu.get('weighted_ngram_match_score'),
                    'syntax_match_score': codebleu.get('syntax_match_score'),
                    'dataflow_match_score': codebleu.get('dataflow_match_score'),
                    'rouge1': rouge_score.get('rouge1'),
                    'rouge2': rouge_score.get('rouge2'),
                    'rougeL': rouge_score.get('rougeL'),
                    'rougeLsum': rouge_score.get('rougeLsum')
                }
                data.append(row)
            except Exception as e:
                print(f"Error calculating scores for {model}/{system_folder} round {i}: {e}")

    return data, missing_files

def process_model_system_pair(model, system_folder):
    """Process a model-system pair and return both evaluation data and missing files."""
    output_model_path = os.path.join(output_path, model)
    os.makedirs(output_model_path, exist_ok=True)
    return evaluate_system(system_folder, model, output_model_path, dataset_path)

if __name__ == '__main__':
    # Initialize lists to collect data
    all_data = []
    all_missing = []

    # Parallel execution using ProcessPoolExecutor
    with ProcessPoolExecutor() as executor:
        futures = []
        for test_model in test_model_list:
            for system_folder in os.listdir(dataset_path):
                if system_folder in system_do_list:
                    futures.append(executor.submit(process_model_system_pair, test_model, system_folder))

        # Collecting the results
        for future in tqdm(as_completed(futures), total=len(futures)):
            try:
                result, missing = future.result()
                if result is not None:
                    all_data.extend(result)
                if missing is not None:
                    all_missing.extend(missing)
            except Exception as e:
                print(f"Error processing future: {e}")

    # Convert evaluation data to DataFrame and save
    if all_data:
        df = pd.DataFrame(all_data)
        output_file = os.path.join(output_statistic_path, "evaluation_results_with_tracking.csv")
        os.makedirs(os.path.dirname(output_file), exist_ok=True)
        df.to_csv(output_file, index=False)
        print(f"Evaluation results saved to {output_file}")
    else:
        print("No evaluation data collected.")

    # Convert missing implementations data to DataFrame and save
    if all_missing:
        missing_df = pd.DataFrame(all_missing)
        
        # Create summary of missing implementations
        summary = missing_df.groupby(['model', 'system']).agg({
            'missing_file': 'count',
            'round': lambda x: ', '.join(x)
        }).rename(columns={'missing_file': 'missing_count', 'round': 'missing_rounds'})
        summary = summary.reset_index()
        
        # Save detailed missing implementations record
        missing_file = os.path.join(output_statistic_path, f"missing_implementations_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
        missing_df.to_csv(missing_file, index=False)
        print(f"Detailed missing implementations saved to {missing_file}")
        
        # Save summary
        summary_file = os.path.join(output_statistic_path, f"missing_implementations_summary_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
        summary.to_csv(summary_file, index=False)
        print(f"Summary of missing implementations saved to {summary_file}")
        
        # Print summary statistics
        print("\n=== Missing Implementations Summary ===")
        print(f"Total missing files: {len(all_missing)}")
        print(f"Models with missing implementations: {missing_df['model'].nunique()}")
        print(f"Systems with missing implementations: {missing_df['system'].nunique()}")
        
        # Print models with most missing implementations
        model_missing_counts = missing_df.groupby('model')['missing_file'].count().sort_values(ascending=False)
        print("\nTop 5 models with most missing implementations:")
        for model, count in model_missing_counts.head().items():
            print(f"  {model}: {count} missing files")
        
        # Print systems with most missing implementations
        system_missing_counts = missing_df.groupby('system')['missing_file'].count().sort_values(ascending=False)
        print("\nTop 5 systems with most missing implementations:")
        for system, count in system_missing_counts.head().items():
            print(f"  {system}: {count} missing files")
    else:
        print("No missing implementations found - all files present!")

    print("\nFinished processing all model-system pairs.")