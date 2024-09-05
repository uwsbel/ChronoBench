import evaluate
from codebleu import calc_codebleu
import os
import json
from tqdm import tqdm
import pandas as pd
from concurrent.futures import ProcessPoolExecutor, as_completed

# Define paths
dataset_path = 'D:/SimBench/demo_data'
output_path = 'D:/SimBench/output'
output_statistic_path = 'D:/SimBench/statistic'

# List of models and systems to evaluate
test_model_list = [
    "gemma-2-2b-it", "gemma-2-9b-it", "gemma-2-27b-it",
    "llama-3.1-405b-instruct", "llama-3.1-70b-instruct",
    "llama-3.1-8b-instruct", "phi-3-mini-128k-instruct", "nemotron-4-340b-instruct",
    "mistral-nemo-12b-instruct", "mixtral-8x22b-instruct-v0.1",
    "codestral-22b-instruct-v0.1", "mixtral-8x7b-instruct-v0.1",
    "mistral-large-latest", "mamba-codestral-7b-v0.1",
    "gpt-4o", "gpt-4o-mini", "claude-3-5-sonnet", "Gemini","phi-3-medium-128k-instruct",
]

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

def read_script(file_path):
    with open(file_path, "r", encoding="utf-8") as file:
        return file.read()

def evaluate_system(system_folder, model, output_model_path, dataset_path):
    rouge = evaluate.load('rouge')
    # Paths based on the current system and model
    system_folder_path = os.path.join(dataset_path, system_folder)
    output_system_path = os.path.join(output_model_path, system_folder)

    if not os.path.exists(system_folder_path):
        print(f"System folder not found: {system_folder_path}")
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
        print(f"Skipping system {output_model_path},{system_folder} due to missing files.")
        #also print the missing files
        if "" in predictions:
            print(f"Missing predictions: {output_system_path}")
        if "" in references:
            print(f"Missing references: {system_folder_path}")
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

        # Collecting the results
        for future in tqdm(as_completed(futures), total=len(futures)):
            result = future.result()
            if result is not None:
                all_data.extend(result)

    # Convert the collected data into a DataFrame
    df = pd.DataFrame(all_data)

    # Save the DataFrame to a single CSV file
    output_file = os.path.join(output_statistic_path, "evaluation_results.csv")
    os.makedirs(os.path.dirname(output_file), exist_ok=True)
    df.to_csv(output_file, index=False)

    print(f"Results saved to {output_file}")
    print("Finished")
