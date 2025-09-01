import os
import sys
from concurrent.futures import ThreadPoolExecutor, as_completed
from tqdm import tqdm
import evaluate
from openai import OpenAI
import json
import re
import logging
import csv
import io
import subprocess
import time
import random
from datetime import datetime

# Only OpenAI API needed for judge model
# The evaluated models' outputs are already generated and saved

# ============================================
# ENHANCED LOGGING CONFIGURATION
# ============================================

# Set up the evaluated model
evaluated_model = "gpt-4o-mini"

# Create output directory for this judge model
OUTPUT_DIR = f"/home/hongyu/Documents/SimBench/scoring/out_diff_models/out_{evaluated_model.replace('.', '-')}"
os.makedirs(OUTPUT_DIR, exist_ok=True)

# Set up comprehensive logging
LOG_FILE = os.path.join(OUTPUT_DIR, "jllm_score_log.txt")

# Create a custom logger
logger = logging.getLogger('JLLM_Score')
logger.setLevel(logging.DEBUG)

# Create file handler for detailed logging
file_handler = logging.FileHandler(LOG_FILE, mode='a', encoding='utf-8')
file_handler.setLevel(logging.DEBUG)

# Create console handler for basic output
console_handler = logging.StreamHandler(sys.stdout)
console_handler.setLevel(logging.INFO)

# Create formatter
formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s', datefmt='%Y-%m-%d %H:%M:%S')
file_handler.setFormatter(formatter)
console_formatter = logging.Formatter('%(message)s')
console_handler.setFormatter(console_formatter)

# Add handlers to logger
logger.addHandler(file_handler)
logger.addHandler(console_handler)

# ============================================
# API KEY CONFIGURATION
# ============================================

# Load API key specifically for gpt-4o-mini judge model
# Priority order: Model-specific key > Numbered key > Default key
openai_api_key = os.getenv("OPENAI_API_KEY_GPT4OMINI") or os.getenv("OPENAI_API_KEY_1")
if not openai_api_key:
    # Fallback to default if specific key not found
    openai_api_key = os.getenv("OPENAI_API_KEY")
    logger.warning("OPENAI_API_KEY_GPT4OMINI and OPENAI_API_KEY_1 not found, using default OPENAI_API_KEY")
else:
    key_source = "OPENAI_API_KEY_GPT4OMINI" if os.getenv("OPENAI_API_KEY_GPT4OMINI") else "OPENAI_API_KEY_1"
    logger.info(f"Using {key_source} for gpt-4o-mini evaluations")

# No other API keys needed - only using OpenAI for judging
# All model outputs are already generated in output_llms directory

# Log startup information
logger.info("="*60)
logger.info(f"JLLM Scoring Script Started - Judge Model: {evaluated_model}")
logger.info(f"Log file: {LOG_FILE}")
logger.info(f"Output directory: {OUTPUT_DIR}")
logger.info("="*60)

rouge = evaluate.load('rouge')

# ============================================
# PROGRESS TRACKING
# ============================================

class ProgressTracker:
    def __init__(self, output_dir):
        self.progress_file = os.path.join(output_dir, "progress.json")
        self.load_progress()
    
    def load_progress(self):
        if os.path.exists(self.progress_file):
            with open(self.progress_file, 'r') as f:
                self.progress = json.load(f)
        else:
            self.progress = {
                "completed": [],
                "failed": [],
                "in_progress": None,
                "start_time": datetime.now().isoformat(),
                "last_update": datetime.now().isoformat()
            }
    
    def save_progress(self):
        self.progress["last_update"] = datetime.now().isoformat()
        with open(self.progress_file, 'w') as f:
            json.dump(self.progress, f, indent=2)
    
    def mark_completed(self, model, system):
        key = f"{model}/{system}"
        if key not in self.progress["completed"]:
            self.progress["completed"].append(key)
            logger.info(f"✓ Completed: {key} ({len(self.progress['completed'])} total)")
            self.save_progress()
    
    def mark_failed(self, model, system, error):
        key = f"{model}/{system}"
        self.progress["failed"].append({"key": key, "error": str(error), "time": datetime.now().isoformat()})
        logger.error(f"✗ Failed: {key} - {error}")
        self.save_progress()
    
    def is_completed(self, model, system):
        return f"{model}/{system}" in self.progress["completed"]
    
    def set_in_progress(self, model, system):
        self.progress["in_progress"] = f"{model}/{system}"
        self.save_progress()

progress_tracker = ProgressTracker(OUTPUT_DIR)

def get_provider_for_model(model_name):
    """For JLLM scoring, only OpenAI models are used as judges."""
    # All judge models (gpt-4o-mini, gpt-4.1-mini, gpt-4.1-nano) are OpenAI models
    return "openai"

def get_llm_response(prompt, model_name, temperature=0.2, top_p=0.7, max_tokens=12000):
    """Get response from OpenAI judge model for evaluation.
    Note: All test model outputs are already generated and saved in output_llms.
    This function is only used to get judge scores from OpenAI models.
    """
    try:
        if not openai_api_key:
            raise Exception("OpenAI API key not set")
            
        client = OpenAI(api_key=openai_api_key)
            
        completion = client.chat.completions.create(
            model=model_name,
            messages=[{"role": "user", "content": prompt}],
            temperature=temperature,
            top_p=top_p,
            max_tokens=max_tokens,
            stream=False
        )
        return completion.choices[0].message.content
        
    except Exception as e:
        logger.error(f"Error with OpenAI API for judge model {model_name}: {str(e)}")
        raise

def retry_with_exponential_backoff(max_retries=None, base_wait=2, max_wait=300):
    """
    Decorator to retry API calls with exponential backoff.
    Enhanced with better logging for rate limits.
    """
    def decorator(func):
        def wrapper(*args, **kwargs):
            retries = 0
            while True:
                try:
                    return func(*args, **kwargs)
                except Exception as e:
                    error_message = str(e).lower()
                    
                    # Check if it's a rate limit or usage limit error
                    is_rate_limit = any(keyword in error_message for keyword in [
                        "429", "rate_limit", "rate limit", "quota", "insufficient_quota",
                        "usage limit", "limit exceeded", "too many requests"
                    ])
                    
                    if is_rate_limit:
                        # Try to extract specific wait time from error message
                        wait_time = None
                        error_str = str(e)
                        
                        patterns = [
                            r'try again in ([\d.]+)\s*s',
                            r'Please try again in ([\d.]+)',
                            r'retry after ([\d.]+)',
                            r'wait ([\d.]+) second',
                            r'available in ([\d.]+) second',
                        ]
                        
                        for pattern in patterns:
                            wait_match = re.search(pattern, error_str, re.IGNORECASE)
                            if wait_match:
                                wait_time = float(wait_match.group(1)) + 1.0
                                logger.info(f"API provided wait time: {wait_time-1:.0f} seconds")
                                break
                        
                        if wait_time is None:
                            wait_time = base_wait * (2 ** min(retries, 10)) + random.uniform(0, 1)
                            wait_time = min(wait_time, max_wait)
                            logger.info(f"No wait time in error, using exponential backoff")
                        
                        logger.warning(f"⏳ Rate/usage limit hit. Will retry indefinitely...")
                        logger.warning(f"  Waiting {wait_time:.1f} seconds before retry #{retries + 1}")
                        logger.debug(f"  Error: {str(e)[:150]}...")
                        
                        # Show countdown for long waits
                        if wait_time > 10:
                            for remaining in range(int(wait_time), 0, -1):
                                print(f"\r  Time remaining: {remaining} seconds...", end="", flush=True)
                                time.sleep(1)
                            print("\r  Retrying now...                    ", flush=True)
                        else:
                            time.sleep(wait_time)
                        
                        retries += 1
                        continue
                    else:
                        # For non-rate-limit errors, respect max_retries
                        if max_retries and retries >= max_retries:
                            logger.error(f"Max retries ({max_retries}) reached for non-rate-limit error.")
                            raise e
                        elif retries < 3:
                            wait_time = base_wait * (2 ** retries)
                            logger.warning(f"Error occurred. Waiting {wait_time:.1f} seconds before retry {retries + 1}/3...")
                            time.sleep(wait_time)
                            retries += 1
                            continue
                        else:
                            raise e
            
            return None
        return wrapper
    return decorator

def read_script(file_path):
    with open(file_path, "r", encoding="utf-8") as file:
        return file.read()

@retry_with_exponential_backoff(max_retries=None, base_wait=2, max_wait=300)
def evaluate_pychrono_code_against_reference_document(code, reference_code, api_documentation_link, model_link):
    prompt = f"""
    You are a PyChrono expert tasked with evaluating a simulation script by comparing it against a reference script generated by experts. Your evaluation should consider both the accuracy of the script compared to the reference and adherence to best practices as outlined in the PyChrono API documentation.

    Here is the PyChrono code you need to evaluate:
    [The Start of Assistant's Answer]
    {code}
    [The End of Assistant's Answer]

    Here is the expert-generated reference code:
    [The Start of Reference Answer]
    {reference_code}
    [The End of Reference Answer]

    Use the following evaluation criteria and point deduction guidelines:

    1. **Completeness (40 points total)**
       - Compare the provided code to the reference script. Deduct **15 points** for each missing essential component (e.g., system initialization, body creation, visualization) that is present in the reference script.
       - Deduct **10 points** if a component is present but lacks important details or is incorrectly configured compared to the reference.
       - Deduct **5 points** for minor omissions or slight deviations from the reference script.

    2. **Correctness (30 points total)**
       - Compare the code to the reference script. Deduct **15 points** for each incorrect use of a PyChrono API that could lead to a significant change in simulation behavior.
       - Deduct **10 points** for logical errors in the code, such as incorrect joint initialization or wrong setting of body properties, especially if the reference script does it correctly.
       - Deduct **5 points** for minor inaccuracies or unnecessary API calls that deviate from the reference script.

    3. **Code Quality (10 points total)**
       - Evaluate the readability, structure, and documentation of the code against the reference script. Deduct **5 to 10 points** for poor readability, structure, or lack of meaningful variable names and formatting.
       - Deduct **5 points** for insufficient comments or failure to follow documentation best practices, especially if the reference script provides better documentation.

    4. **Efficiency (10 points total)**
       - Evaluate the efficiency of the code compared to the reference script. Deduct **5 points** for each instance of unnecessary calculations, redundant code, or inefficient use of APIs that is optimized in the reference script.
       - Deduct **3 points** for missing obvious optimization opportunities that the reference script implements.

    5. **Error Handling and Robustness (5 points total)**
       - Assess the error handling and robustness of the code. Deduct **5 points** for lack of basic error handling or failure to account for common issues that the reference script handles.
       - Deduct **3 points** for inadequate handling of edge cases compared to the reference script.

    6. **Use of Visualization Tools (5 points total)**
       - Compare the use of visualization tools in the provided code to the reference script. Deduct **3 to 5 points** for incorrect or inadequate visualization setup as per the reference script.
       - Deduct **2 points** for minor visualization issues, such as suboptimal lighting or incomplete setup of visual elements, compared to the reference.

    Avoid position biases and ensure that the order in which the responses are presented does not influence your decision. Do not allow the length of the responses to influence your evaluation. Do not favor certain names of the assistants. Be as objective as possible.

    After providing your explanation, output the final score using the following format: "[[x]]" where x is the score assigned to the assistant's answer.

    Reference the PyChrono API documentation provided here: {api_documentation_link}

    Provide the evaluated score and a brief explanation of the deductions below:
    """
    response = get_llm_response(prompt, model_link)
    return response, prompt

@retry_with_exponential_backoff(max_retries=None, base_wait=2, max_wait=300)
def evaluate_pychrono_code_against_reference(code, reference_code, model_link):
    prompt = f"""
    You are a PyChrono expert tasked with evaluating a simulation script by comparing it against a reference script generated by experts.

    Here is the PyChrono code you need to evaluate:
    [The Start of Assistant's Answer]
    {code}
    [The End of Assistant's Answer]

    Here is the expert-generated reference code:
    [The Start of Reference Answer]
    {reference_code}
    [The End of Reference Answer]

    Use the following evaluation criteria and point deduction guidelines:

    1. **Completeness (40 points total)**
       - Compare the provided code to the reference script. Deduct **15 points** for each missing essential component (e.g., system initialization, body creation, visualization) that is present in the reference script.
       - Deduct **10 points** if a component is present but lacks important details or is incorrectly configured compared to the reference.
       - Deduct **5 points** for minor omissions or slight deviations from the reference script.

    2. **Correctness (30 points total)**
       - Compare the code to the reference script. Deduct **15 points** for each incorrect use of a PyChrono API that could lead to a significant change in simulation behavior.
       - Deduct **10 points** for logical errors in the code, such as incorrect joint initialization or wrong setting of body properties, especially if the reference script does it correctly.
       - Deduct **5 points** for minor inaccuracies or unnecessary API calls that deviate from the reference script.

    3. **Code Quality (10 points total)**
       - Evaluate the readability, structure, and documentation of the code against the reference script. Deduct **5 to 10 points** for poor readability, structure, or lack of meaningful variable names and formatting.
       - Deduct **5 points** for insufficient comments or failure to follow documentation best practices, especially if the reference script provides better documentation.

    4. **Efficiency (10 points total)**
       - Evaluate the efficiency of the code compared to the reference script. Deduct **5 points** for each instance of unnecessary calculations, redundant code, or inefficient use of APIs that is optimized in the reference script.
       - Deduct **3 points** for missing obvious optimization opportunities that the reference script implements.

    5. **Error Handling and Robustness (5 points total)**
       - Assess the error handling and robustness of the code. Deduct **5 points** for lack of basic error handling or failure to account for common issues that the reference script handles.
       - Deduct **3 points** for inadequate handling of edge cases compared to the reference script.

    6. **Use of Visualization Tools (5 points total)**
       - Compare the use of visualization tools in the provided code to the reference script. Deduct **3 to 5 points** for incorrect or inadequate visualization setup as per the reference script.
       - Deduct **2 points** for minor visualization issues, such as suboptimal lighting or incomplete setup of visual elements, compared to the reference.

    Avoid position biases and ensure that the order in which the responses are presented does not influence your decision. Do not allow the length of the responses to influence your evaluation. Do not favor certain names of the assistants. Be as objective as possible.

    After providing your explanation, output the final score using the following format: "[[x]]" where x is the score assigned to the assistant's answer.

    Provide the evaluated score and a brief explanation of the deductions below:
    """
    response = get_llm_response(prompt, model_link)
    return response, prompt

@retry_with_exponential_backoff(max_retries=None, base_wait=2, max_wait=300)
def evaluate_pychrono_code_against_document(code, api_documentation_link, model_link):
    prompt = f"""
        You are a PyChrono expert tasked with evaluating a simulation script by comparing it against the PyChrono API documentation. While the API documentation provides guidelines, it may not cover all aspects due to length constraints. Therefore, your evaluation should also be based on your knowledge of best practices in Python coding and general simulation principles.

        Here is the PyChrono code you need to evaluate:
        [The Start of Assistant's Answer]
        {code}
        [The End of Assistant's Answer]

        Use the following evaluation criteria and point deduction guidelines:

        1. **Completeness (40 points total)**
           - Deduct **15 points** for each missing essential component (e.g., system initialization, body creation, visualization) as outlined in the PyChrono API documentation or generally expected in a simulation setup.
           - Deduct **10 points** if a component is present but lacks important details or is incorrectly configured according to the API documentation or general simulation best practices.
           - Deduct **5 points** for minor omissions or slight deviations from best practices mentioned in the API documentation or common Python coding practices.

        2. **Correctness (30 points total)**
           - Deduct **15 points** for each incorrect use of a PyChrono API that could lead to a significant change in simulation behavior, as indicated by the documentation or your expert knowledge.
           - Deduct **10 points** for logical errors in the code, such as incorrect joint initialization or wrong setting of body properties, based on the API documentation or standard simulation principles.
           - Deduct **5 points** for minor inaccuracies or unnecessary API calls that deviate from the API guidelines or standard coding practices.

        3. **Code Quality (10 points total)**
           - Evaluate the readability, structure, and documentation of the code. Deduct **5 to 10 points** for poor readability, structure, or lack of meaningful variable names and formatting, based on your Python expertise.
           - Deduct **5 points** for insufficient comments or failure to follow documentation best practices, whether outlined in the API documentation or based on general coding standards.

        4. **Efficiency (10 points total)**
           - Deduct **5 points** for each instance of unnecessary calculations, redundant code, or inefficient use of APIs that could be optimized according to the API documentation or your understanding of efficient coding practices.
           - Deduct **3 points** for missing obvious optimization opportunities as suggested by the API documentation or standard programming practices.

        5. **Error Handling and Robustness (5 points total)**
           - Deduct **5 points** for lack of basic error handling or failure to account for common issues, as recommended by the API documentation or best practices in Python coding.
           - Deduct **3 points** for inadequate handling of edge cases, considering both the API documentation and typical robustness requirements in coding.

        6. **Use of Visualization Tools (5 points total)**
           - Deduct **3 to 5 points** for incorrect or inadequate visualization setup according to the API documentation or general expectations for visualizing simulations.
           - Deduct **2 points** for minor visualization issues, such as suboptimal lighting or incomplete setup of visual elements, based on both the API documentation and your understanding of effective simulation visualization.

        Avoid position biases and ensure that the order in which the responses are presented does not influence your decision. Do not allow the length of the responses to influence your evaluation. Do not favor certain names of the assistants. Be as objective as possible.

        Reference the PyChrono API documentation provided here: {api_documentation_link}

        After providing your explanation, output the final score using the following format: "[[x]]" where x is the score assigned to the assistant's answer.

        Provide the evaluated score and a brief explanation of the deductions below:
        """
    response = get_llm_response(prompt, model_link)
    return response, prompt

def evaluate_and_save_results(round_name, prediction, reference_code, api_path, output_system_path):
    """Enhanced with better logging and error handling"""
    logger.debug(f"Starting evaluation for {round_name}")
    
    # Run all three evaluation methods in parallel
    from concurrent.futures import ThreadPoolExecutor
    
    def eval_document():
        try:
            score, prompt = evaluate_pychrono_code_against_document(prediction, api_path, evaluated_model)
            logger.info(f"✓ {round_name} - Document evaluation complete")
            return score, prompt
        except Exception as e:
            logger.error(f"✗ {round_name} - Document evaluation failed: {e}")
            return f"Error: {str(e)}", ""
    
    def eval_reference():
        try:
            score, prompt = evaluate_pychrono_code_against_reference(prediction, reference_code, evaluated_model)
            logger.info(f"✓ {round_name} - Reference evaluation complete")
            return score, prompt
        except Exception as e:
            logger.error(f"✗ {round_name} - Reference evaluation failed: {e}")
            return f"Error: {str(e)}", ""
    
    def eval_reference_document():
        try:
            score, prompt = evaluate_pychrono_code_against_reference_document(prediction, reference_code, api_path, evaluated_model)
            logger.info(f"✓ {round_name} - Reference+Document evaluation complete")
            return score, prompt
        except Exception as e:
            logger.error(f"✗ {round_name} - Reference+Document evaluation failed: {e}")
            return f"Error: {str(e)}", ""
    
    # Execute evaluations sequentially for OpenAI models to avoid rate limits
    score_document, prompt_document = eval_document()
    score_reference, prompt_reference = eval_reference()
    score_reference_document, prompt_reference_document = eval_reference_document()

    # Save scores to files
    score_document_path = os.path.join(output_system_path, f"{round_name}_score_document.txt")
    score_reference_path = os.path.join(output_system_path, f"{round_name}_score_reference.txt")
    score_reference_document_path = os.path.join(output_system_path, f"{round_name}_score_reference_document.txt")

    with open(score_document_path, 'w', encoding="utf-8") as file:
        file.write(score_document)

    with open(score_reference_path, 'w', encoding="utf-8") as file:
        file.write(score_reference)

    with open(score_reference_document_path, 'w', encoding="utf-8") as file:
        file.write(score_reference_document)

    # Prepare and save evaluation data as JSON
    evaluation_data = {
        "round_name": round_name,
        "prediction": prediction,
        "reference_code": reference_code,
        "api_path": api_path,
        "output_system_path": output_system_path,
        "scores": {
            "score_document": score_document,
            "score_reference": score_reference,
            "score_reference_document": score_reference_document
        },
        "prompts": {
            "prompt_document": prompt_document,
            "prompt_reference": prompt_reference,
            "prompt_reference_document": prompt_reference_document
        }
    }

    json_output_path = os.path.join(output_system_path, f"{round_name}_evaluation.json")
    with open(json_output_path, 'w', encoding="utf-8") as json_file:
        json.dump(evaluation_data, json_file, indent=4, ensure_ascii=False)

def merge_csv_files(output_path, combined_csv_filename="combined_evaluation_scores.csv"):
    """Merges all small CSV files from different models and systems into a single large CSV file."""
    combined_csv_data = []

    for model_dir in os.listdir(output_path):
        model_path = os.path.join(output_path, model_dir)
        if os.path.isdir(model_path):
            for system_dir in os.listdir(model_path):
                system_path = os.path.join(model_path, system_dir)
                if os.path.isdir(system_path):
                    small_csv_path = os.path.join(system_path, "evaluation_scores.csv")
                    if os.path.exists(small_csv_path):
                        with open(small_csv_path, 'r', encoding="utf-8") as csvfile:
                            reader = csv.reader(csvfile)
                            headers = next(reader)
                            if not combined_csv_data:
                                combined_csv_data.append(headers)
                            combined_csv_data.extend(list(reader))

    combined_csv_path = os.path.join(output_path, combined_csv_filename)
    with open(combined_csv_path, 'w', newline='', encoding="utf-8") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerows(combined_csv_data)

    logger.info(f"Combined CSV file saved to {combined_csv_path}")

def extract_scores_from_txt(file_path):
    """Extracts the numerical score from a text file."""
    with open(file_path, 'r', encoding="utf-8") as file:
        content = file.read()

    if content.startswith("Error:"):
        logger.warning(f"Warning: {file_path} contains an error instead of a score")
        return 0
    
    match = re.search(r"\[\[(\d+)\]\]", content)
    if match:
        return int(match.group(1))
    
    # Handle format like "[[x]] 70" where score is after [[x]]
    match_x = re.search(r"\[\[x\]\]\s*(\d+)", content)
    if match_x:
        score = int(match_x.group(1))
        logger.info(f"Found score in [[x]] format: {score} in {file_path}")
        return score
    
    logger.warning(f"Warning: No valid score found in {file_path}, using default score 0")
    return 0

def save_scores_to_csv_with_metadata(output_system_path, test_model, system_folder,
                                    csv_filename="evaluation_scores.csv", evaluated_model="gpt-4o-mini"):
    """Extracts scores and saves them into a CSV with metadata."""
    csv_data = [["Test Model", "System", "Round", "Score Document", "Score Reference", "Score Reference Document"]]

    rounds = ["first", "second", "third"]
    csv_output_path = os.path.join(output_system_path, csv_filename)

    for round_name in rounds:
        try:
            score_document_path = os.path.join(output_system_path, f"{round_name}_score_document.txt")
            score_reference_path = os.path.join(output_system_path, f"{round_name}_score_reference.txt")
            score_reference_document_path = os.path.join(output_system_path, f"{round_name}_score_reference_document.txt")

            score_document = extract_scores_from_txt(score_document_path)
            score_reference = extract_scores_from_txt(score_reference_path)
            score_reference_document = extract_scores_from_txt(score_reference_document_path)

            csv_data.append([test_model, system_folder, round_name, score_document, score_reference, score_reference_document])

        except Exception as e:
            logger.error(f"Error processing {round_name} in system {system_folder} for model {test_model}: {e}")

    with open(csv_output_path, 'w', newline='', encoding="utf-8") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerows(csv_data)

    logger.info(f"Scores saved to {csv_output_path}")

# Paths
dataset_path = r"/home/hongyu/Documents/SimBench/demo_data"
# FIXED: Use judge-specific output directory to avoid overwriting between judges
Output_path = f"/home/hongyu/Documents/SimBench/output_llms_{evaluated_model.replace('.', '-')}"
Output_conversation_path = r"/home/hongyu/Documents/SimBench/output_conversion"
Output_statistic_path = r"/home/hongyu/Documents/SimBench/statistic"

# Model and system lists
test_model_list = [
    # DeepSeek Models
    "deepseek-r1",
    "deepseek-r1-8b",
    "deepseek-r1-32b",
    
    # Meta/Llama Models
    "llama-3.1-405b-instruct",
    "llama-3.1-70b-instruct",
    "llama-3.1-8b-instruct",
    "llama-3.3-70b-instruct",
    "llama4_maverick",
    "llama4_scout",
    "llama3.1-8b-f2",
    "llama3.3-70b-sft1",
    "llama3.1-8b-lora1",
    "llama4-109b-lora1",
    "llama3.3-70b-lora1",
    
    # NVIDIA Models
    "nemotron-4-340b-instruct",
    
    # Microsoft Phi Models
    "phi-3-mini-128k-instruct",
    "phi-3-medium-128k-instruct",
    
    # Qwen Model
    "qwen3-235b-a22b"
]

system_list = ["art", "beam", "buckling", "cable", "camera", "citybus", "curiosity", "feda", "gator", 
               "gear", "gps_imu", "handler", "hmmwv", "kraz", "lidar", "m113", "man", "mass_spring_damper", 
               "particles", "pendulum", "rigid_highway", "rigid_multipatches", "rotor", "scm", "scm_hill", 
               "sedan", "sensros", "slider_crank", "tablecloth", "turtlebot", "uazbus", "veh_app", "vehros", "viper"]

system_do_list = system_list

def is_already_evaluated(output_path, test_model, system_folder):
    """Check if all score files exist for a model/system combination"""
    required_files = [
        'first_score_document.txt',
        'second_score_document.txt', 
        'third_score_document.txt',
        'evaluation_scores.csv'
    ]
    
    system_path = os.path.join(output_path, test_model, system_folder)
    
    if not os.path.exists(system_path):
        return False
    
    for file in required_files:
        file_path = os.path.join(system_path, file)
        if not os.path.exists(file_path) or os.path.getsize(file_path) == 0:
            return False
    
    return True

def process_model_system(test_model, system_folder, dataset_path, Output_path, Output_conversation_path, Output_statistic_path):
    """Process a single model/system combination with enhanced logging"""
    
    # Check if already evaluated using progress tracker
    if progress_tracker.is_completed(test_model, system_folder):
        logger.info(f"⏭️  Skipping: {test_model}/{system_folder} (already evaluated)")
        return f"Skipped: {system_folder} for model {test_model}"
    
    # Also check file system for safety
    if is_already_evaluated(Output_path, test_model, system_folder):
        progress_tracker.mark_completed(test_model, system_folder)
        logger.info(f"⏭️  Skipping: {test_model}/{system_folder} (files exist)")
        return f"Skipped: {system_folder} for model {test_model}"
    
    progress_tracker.set_in_progress(test_model, system_folder)
    
    # Add delay to avoid rate limits
    delay = random.uniform(2.0, 4.0)
    time.sleep(delay)
    
    system_folder_path = os.path.join(dataset_path, system_folder)
    output_system_path = os.path.join(Output_path, test_model, system_folder)
    os.makedirs(output_system_path, exist_ok=True)

    if system_folder in system_do_list:
        logger.info(f'Processing model {test_model} on system {system_folder}')
        
        try:
            # Read response files
            first_response_path = os.path.join(output_system_path, "first_response.py")
            second_response_path = os.path.join(output_system_path, "second_response.py")
            third_response_path = os.path.join(output_system_path, "third_response.py")

            first_prediction = read_script(first_response_path)
            second_prediction = read_script(second_response_path)
            third_prediction = read_script(third_response_path)

            # Read reference files
            first_reference_path = os.path.join(system_folder_path, 'truth1.py')
            second_reference_path = os.path.join(system_folder_path, 'truth2.py')
            third_reference_path = os.path.join(system_folder_path, 'truth3.py')

            first_reference = read_script(first_reference_path)
            second_reference = read_script(second_reference_path)
            third_reference = read_script(third_reference_path)

            api_path = read_script(os.path.join(r'/home/hongyu/Documents/SimBench/api', 'api.txt'))

            # Evaluate all rounds
            evaluate_and_save_results("first", first_prediction, first_reference, api_path, output_system_path)
            evaluate_and_save_results("second", second_prediction, second_reference, api_path, output_system_path)
            evaluate_and_save_results("third", third_prediction, third_reference, api_path, output_system_path)

            # Save scores to CSV
            save_scores_to_csv_with_metadata(output_system_path, test_model, system_folder, "evaluation_scores.csv")
            
            progress_tracker.mark_completed(test_model, system_folder)
            
        except Exception as e:
            logger.error(f"Failed to process {test_model}/{system_folder}: {e}")
            progress_tracker.mark_failed(test_model, system_folder, str(e))
            return f"Failed: {system_folder} for model {test_model}"

    return f"Completed {system_folder} for model {test_model}"

# Main execution
logger.info("="*60)
logger.info(f"Starting evaluation with judge model: {evaluated_model}")
logger.info(f"Processing {len(test_model_list)} models × {len(system_do_list)} systems = {len(test_model_list) * len(system_do_list)} combinations")
logger.info("="*60)

# Use low concurrency for OpenAI models
max_workers = 2
logger.info(f"Using reduced concurrency (max_workers={max_workers}) for OpenAI judge model: {evaluated_model}")

with ThreadPoolExecutor(max_workers=max_workers) as executor:
    futures = []
    total_tasks = 0
    
    for test_model in test_model_list:
        output_model_path = os.path.join(Output_path, test_model)
        os.makedirs(output_model_path, exist_ok=True)

        for system_folder in system_do_list:
            futures.append(
                executor.submit(
                    process_model_system,
                    test_model,
                    system_folder,
                    dataset_path,
                    Output_path,
                    Output_conversation_path,
                    Output_statistic_path
                )
            )
            total_tasks += 1

    # Use tqdm with enhanced logging
    completed = 0
    for future in tqdm(as_completed(futures), total=len(futures), desc=f"Evaluating with {evaluated_model}"):
        result = future.result()
        completed += 1
        if completed % 10 == 0:
            logger.info(f"Progress: {completed}/{total_tasks} tasks completed ({100*completed/total_tasks:.1f}%)")

logger.info("="*60)
logger.info("Finished processing all models and systems")
logger.info(f"Completed: {len(progress_tracker.progress['completed'])} evaluations")
logger.info(f"Failed: {len(progress_tracker.progress['failed'])} evaluations")
logger.info("="*60)

# Merge CSV files
merge_csv_files(Output_path)
logger.info("All evaluations complete!")