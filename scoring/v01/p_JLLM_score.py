import os
from concurrent.futures import ThreadPoolExecutor, as_completed
from tqdm import tqdm
import evaluate
from openai import OpenAI
import json
import re
import sys
import logging
import csv
import io
import subprocess
import time
import random
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

# Import additional API clients
try:
    from anthropic import Anthropic
except ImportError:
    Anthropic = None
    print("Warning: Anthropic library not installed. Claude models will not work.")

try:
    import google.generativeai as genai
except ImportError:
    genai = None
    print("Warning: Google GenerativeAI library not installed. Gemini models will not work.")

# Load API keys from environment (now loaded from .env)
openai_api_key = os.getenv("OPENAI_API_KEY")
anthropic_api_key = os.getenv("ANTHROPIC_API_KEY")
google_api_key = os.getenv("GOOGLE_API_KEY") or os.getenv("GEMINI_API_KEY")
mistral_api_key = os.getenv("MISTRAL_API_KEY")
nvidia_api_key = openai_api_key  # Keep for backward compatibility

rouge = evaluate.load('rouge')

def get_provider_for_model(model_name):
    """Determine the API provider for a given model."""
    model_lower = model_name.lower()
    
    if "claude" in model_lower:
        return "anthropic"
    elif "gemini" in model_lower or "gemma" in model_lower:
        return "google"
    elif "gpt" in model_lower or "o3" in model_lower or "o4" in model_lower:
        return "openai"
    elif "mistral" in model_lower or "mixtral" in model_lower or "codestral" in model_lower or "mamba" in model_lower:
        return "mistral"
    elif "llama" in model_lower or "nemotron" in model_lower or "phi" in model_lower:
        return "nvidia"  # These often use NVIDIA endpoints
    elif "deepseek" in model_lower:
        return "deepseek"
    elif "qwen" in model_lower:
        return "qwen"
    else:
        return "openai"  # Default fallback

def get_llm_response(prompt, model_name, temperature=0.2, top_p=0.7, max_tokens=12000):
    """Unified function to get LLM response from any provider."""
    provider = get_provider_for_model(model_name)
    
    try:
        if provider == "anthropic":
            if not Anthropic or not anthropic_api_key:
                raise Exception("Anthropic API not available or API key not set")
            
            client = Anthropic(api_key=anthropic_api_key)
            
            # Map model names to Anthropic's naming convention
            anthropic_model_map = {
                "claude-3-5-sonnet": "claude-3-5-sonnet-20241022",
                "claude-3-7-sonnet-20250219": "claude-3-opus-20240229",  # Map to available model
                "claude-4-sonnet-20250514": "claude-3-5-sonnet-20241022",  # Map to available model
            }
            
            actual_model = anthropic_model_map.get(model_name, model_name)
            
            response = client.messages.create(
                model=actual_model,
                max_tokens=max_tokens,
                temperature=temperature,
                messages=[{"role": "user", "content": prompt}]
            )
            return response.content[0].text
            
        elif provider == "google":
            if not genai or not google_api_key:
                raise Exception("Google API not available or API key not set")
            
            genai.configure(api_key=google_api_key)
            
            # Map model names to Google's naming convention
            google_model_map = {
                "Gemini-1.5-pro": "gemini-1.5-pro",
                "Gemini-2.5-pro": "gemini-1.5-pro",  # Use 1.5 if 2.5 not available
                "gemma-2-2b-it": "gemini-1.5-flash",
                "gemma-2-9b-it": "gemini-1.5-flash",
                "gemma-2-27b-it": "gemini-1.5-pro",
                "gemma-3-1b-it": "gemini-1.5-flash",
            }
            
            actual_model = google_model_map.get(model_name, "gemini-1.5-pro")
            model = genai.GenerativeModel(actual_model)
            
            generation_config = genai.types.GenerationConfig(
                temperature=temperature,
                top_p=top_p,
                max_output_tokens=max_tokens,
            )
            
            response = model.generate_content(prompt, generation_config=generation_config)
            return response.text
            
        elif provider == "mistral":
            if not mistral_api_key:
                # Fall back to OpenAI client with Mistral endpoint
                if not openai_api_key:
                    raise Exception("Mistral API key not set and no OpenAI fallback")
                    
            from mistralai.client import MistralClient
            from mistralai.models.chat_completion import ChatMessage
            
            client = MistralClient(api_key=mistral_api_key)
            
            # Map model names
            mistral_model_map = {
                "mistral-nemo-12b-instruct": "open-mistral-nemo",
                "mixtral-8x22b-instruct-v0.1": "open-mixtral-8x22b",
                "mixtral-8x7b-instruct-v0.1": "open-mixtral-8x7b",
                "mistral-large-latest": "mistral-large-latest",
                "codestral-22b-instruct-v0.1": "codestral-latest",
                "mamba-codestral-7b-v0.1": "open-codestral-mamba",
            }
            
            actual_model = mistral_model_map.get(model_name, model_name)
            
            response = client.chat(
                model=actual_model,
                messages=[ChatMessage(role="user", content=prompt)],
                temperature=temperature,
                top_p=top_p,
                max_tokens=max_tokens,
            )
            return response.choices[0].message.content
            
        else:
            # Default to OpenAI API (works for OpenAI, NVIDIA, and compatible endpoints)
            if not openai_api_key:
                raise Exception(f"OpenAI API key not set for provider {provider}")
                
            client = OpenAI(api_key=openai_api_key)
            
            # For NVIDIA and other providers, might need endpoint adjustment
            if provider == "nvidia":
                # Some NVIDIA models might need special handling
                pass
                
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
        print(f"Error with {provider} API for model {model_name}: {str(e)}")
        raise

def retry_with_exponential_backoff(max_retries=None, base_wait=2, max_wait=300):
    """
    Decorator to retry API calls with exponential backoff.
    For rate limits/usage limits, retries indefinitely.
    
    Args:
        max_retries: Maximum number of retry attempts (None = infinite for rate limits)
        base_wait: Base wait time in seconds (will be exponentially increased)
        max_wait: Maximum wait time in seconds (increased from 60 to 300)
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
                        # Try to extract specific wait time from error message FIRST
                        import re
                        wait_time = None
                        error_str = str(e)
                        
                        # Multiple patterns to match different API error formats
                        patterns = [
                            r'try again in ([\d.]+)\s*s',  # OpenAI format
                            r'Please try again in ([\d.]+)',  # Alternative format
                            r'retry after ([\d.]+)',  # Standard HTTP format
                            r'wait ([\d.]+) second',  # Generic format
                            r'available in ([\d.]+) second',  # Quota format
                        ]
                        
                        for pattern in patterns:
                            wait_match = re.search(pattern, error_str, re.IGNORECASE)
                            if wait_match:
                                wait_time = float(wait_match.group(1)) + 1.0  # Add 1 second buffer
                                print(f"  API provided wait time: {wait_time-1:.0f} seconds")
                                break
                        
                        # If no wait time found in error message, use exponential backoff
                        if wait_time is None:
                            # For rate limits without specific time, use exponential backoff with cap
                            wait_time = base_wait * (2 ** min(retries, 10)) + random.uniform(0, 1)
                            wait_time = min(wait_time, max_wait)  # Only cap the exponential backoff
                            print(f"  No wait time in error, using exponential backoff")
                        
                        print(f"\n⏳ Rate/usage limit hit. Will retry indefinitely...")
                        print(f"  Waiting {wait_time:.1f} seconds before retry #{retries + 1}")
                        print(f"  Error: {str(e)[:150]}...")
                        print(f"  This is normal - the script will continue automatically after waiting.")
                        
                        # Show countdown for long waits
                        if wait_time > 10:
                            for remaining in range(int(wait_time), 0, -1):
                                print(f"\r  Time remaining: {remaining} seconds...", end="", flush=True)
                                time.sleep(1)
                            print("\r  Retrying now...                    ", flush=True)
                        else:
                            time.sleep(wait_time)
                        
                        retries += 1
                        # No limit on retries for rate limit errors
                        continue
                    else:
                        # For non-rate-limit errors, respect max_retries if set
                        if max_retries and retries >= max_retries:
                            print(f"Max retries ({max_retries}) reached for non-rate-limit error.")
                            raise e
                        elif retries < 3:  # Try up to 3 times for other errors
                            wait_time = base_wait * (2 ** retries)
                            print(f"Error occurred. Waiting {wait_time:.1f} seconds before retry {retries + 1}/3...")
                            time.sleep(wait_time)
                            retries += 1
                            continue
                        else:
                            # If it's not a rate limit error and we've tried enough, raise
                            raise e
            
            return None  # Should not reach here
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
    [The Start of Assistant’s Answer]
    {code}
    [The End of Assistant’s Answer]

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

    After providing your explanation, output the final score using the following format: "[[x]]" where x is the score assigned to the assistant’s answer.

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
    [The Start of Assistant’s Answer]
    {code}
    [The End of Assistant’s Answer]

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

    After providing your explanation, output the final score using the following format: "[[x]]" where x is the score assigned to the assistant’s answer.

    Provide the evaluated score and a brief explanation of the deductions below:
    """
    response = get_llm_response(prompt, model_link)
    return response, prompt

@retry_with_exponential_backoff(max_retries=None, base_wait=2, max_wait=300)
def evaluate_pychrono_code_against_document(code, api_documentation_link, model_link):
    prompt = f"""
        You are a PyChrono expert tasked with evaluating a simulation script by comparing it against the PyChrono API documentation. While the API documentation provides guidelines, it may not cover all aspects due to length constraints. Therefore, your evaluation should also be based on your knowledge of best practices in Python coding and general simulation principles.

        Here is the PyChrono code you need to evaluate:
        [The Start of Assistant’s Answer]
        {code}
        [The End of Assistant’s Answer]

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

        After providing your explanation, output the final score using the following format: "[[x]]" where x is the score assigned to the assistant’s answer.

        Provide the evaluated score and a brief explanation of the deductions below:
        """
    response = get_llm_response(prompt, model_link)
    return response, prompt

def evaluate_and_save_results(round_name, prediction, reference_code, api_path, output_system_path):
    # Run all three evaluation methods in parallel
    from concurrent.futures import ThreadPoolExecutor
    
    def eval_document():
        try:
            score, prompt = evaluate_pychrono_code_against_document(prediction, api_path, evaluated_model)
            print(f"✓ {round_name} - Document evaluation complete")
            return score, prompt
        except Exception as e:
            print(f"✗ {round_name} - Document evaluation failed: {e}")
            return f"Error: {str(e)}", ""
    
    def eval_reference():
        try:
            score, prompt = evaluate_pychrono_code_against_reference(prediction, reference_code, evaluated_model)
            print(f"✓ {round_name} - Reference evaluation complete")
            return score, prompt
        except Exception as e:
            print(f"✗ {round_name} - Reference evaluation failed: {e}")
            return f"Error: {str(e)}", ""
    
    def eval_reference_document():
        try:
            score, prompt = evaluate_pychrono_code_against_reference_document(prediction, reference_code, api_path, evaluated_model)
            print(f"✓ {round_name} - Reference+Document evaluation complete")
            return score, prompt
        except Exception as e:
            print(f"✗ {round_name} - Reference+Document evaluation failed: {e}")
            return f"Error: {str(e)}", ""
    
    # Execute evaluations with model-specific concurrency
    # For OpenAI models, run sequentially to avoid rate limits
    if evaluated_model in ["gpt-4o", "gpt-4o-mini", "gpt-4.1", "gpt-4.1-mini", "gpt-4.1-nano", "o3", "o4-mini"]:
        # Run sequentially for OpenAI models
        score_document, prompt_document = eval_document()
        score_reference, prompt_reference = eval_reference()
        score_reference_document, prompt_reference_document = eval_reference_document()
    else:
        # Run in parallel for other models
        with ThreadPoolExecutor(max_workers=3) as executor:
            future_document = executor.submit(eval_document)
            future_reference = executor.submit(eval_reference)
            future_ref_doc = executor.submit(eval_reference_document)
        
        # Wait for all results
        score_document, prompt_document = future_document.result()
        score_reference, prompt_reference = future_reference.result()
        score_reference_document, prompt_reference_document = future_ref_doc.result()

    # Define paths for saving scores
    score_document_path = os.path.join(output_system_path, f"{round_name}_score_document.txt")
    score_reference_path = os.path.join(output_system_path, f"{round_name}_score_reference.txt")
    score_reference_document_path = os.path.join(output_system_path, f"{round_name}_score_reference_document.txt")

    # Save scores to files
    with open(score_document_path, 'w', encoding="utf-8") as file:
        file.write(score_document)

    with open(score_reference_path, 'w', encoding="utf-8") as file:
        file.write(score_reference)

    with open(score_reference_document_path, 'w', encoding="utf-8") as file:
        file.write(score_reference_document)

        # Prepare data to save as JSON
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

        # Define path for JSON output
        json_output_path = os.path.join(output_system_path, f"{round_name}_evaluation.json")

        # Save evaluation data to JSON file
        with open(json_output_path, 'w', encoding="utf-8") as json_file:
            json.dump(evaluation_data, json_file, indent=4, ensure_ascii=False)


def merge_csv_files(output_path, combined_csv_filename="combined_evaluation_scores.csv"):
    """
    Merges all small CSV files from different models and systems into a single large CSV file.

    :param output_path: The root directory where all the model-specific directories are stored.
    :param combined_csv_filename: The name of the resulting combined CSV file.
    """
    combined_csv_data = []

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
                            combined_csv_data.extend(list(reader))
                    else:
                        print(f"No CSV file found at {small_csv_path}")

    # Save the combined data into a large CSV file
    combined_csv_path = os.path.join(output_path, combined_csv_filename)
    with open(combined_csv_path, 'w', newline='', encoding="utf-8") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerows(combined_csv_data)

    print(f"Combined CSV file saved to {combined_csv_path}")


def extract_scores_from_txt(file_path):
    """
    Extracts the numerical score from a text file.
    Assumes that the score is in the format [[x]] where x is the number.
    """
    with open(file_path, 'r', encoding="utf-8") as file:
        content = file.read()

    # Check if the content is an error message
    if content.startswith("Error:"):
        print(f"Warning: {file_path} contains an error instead of a score")
        return 0  # Return 0 score for failed evaluations
    
    # Use regex to find the score in the format [[x]]
    match = re.search(r"\[\[(\d+)\]\]", content)
    if match:
        return int(match.group(1))
    else:
        # Try to handle partial scores or errors more gracefully
        print(f"Warning: No valid score found in {file_path}, using default score 0")
        return 0


def save_scores_to_csv_with_metadata(output_system_path, test_model, system_folder,
                                    csv_filename="evaluation_scores.csv", evaluated_model="gpt-4o-mini"):
    """
    Extracts scores from text files for different evaluation rounds and saves them into a CSV,
    including metadata like the LLM model, testing model, and system.

    :param output_system_path: The directory containing the score text files.
    :param test_model: The name of the LLM being evaluated.
    :param system_folder: The name of the dynamical system being tested.
    :param csv_filename: The name of the CSV file to save the scores.
    :param evaluated_model: The judge model used for evaluation (not currently used in function body).
    """
    # Prepare a list to hold CSV rows
    csv_data = [["Test Model", "System", "Round", "Score Document", "Score Reference", "Score Reference Document"]]

    rounds = ["first", "second", "third"]
    csv_output_path = os.path.join(output_system_path, csv_filename)

    for round_name in rounds:
        try:
            # Define paths to the score text files for each method
            score_document_path = os.path.join(output_system_path, f"{round_name}_score_document.txt")
            score_reference_path = os.path.join(output_system_path, f"{round_name}_score_reference.txt")
            score_reference_document_path = os.path.join(output_system_path,
                                                         f"{round_name}_score_reference_document.txt")

            # Extract scores from the text files
            score_document = extract_scores_from_txt(score_document_path)
            score_reference = extract_scores_from_txt(score_reference_path)
            score_reference_document = extract_scores_from_txt(score_reference_document_path)

            # Append the scores along with metadata to the CSV data list
            csv_data.append(
                [test_model, system_folder, round_name, score_document, score_reference, score_reference_document])

        except Exception as e:
            print(f"Error processing {round_name} in system {system_folder} for model {test_model}: {e}")

        # Overwrite the CSV file with new data
    with open(csv_output_path, 'w', newline='', encoding="utf-8") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerows(csv_data)

    print(f"Scores saved to {csv_output_path}")
# data set path
dataset_path = r"/home/hongyu/Documents/SimBench/demo_data"
Output_path = r"/home/hongyu/Documents/SimBench/output_llms"
Output_conversation_path =  r"/home/hongyu/Documents/SimBench/output_conversion"
Output_statistic_path = r"/home/hongyu/Documents/SimBench/statistic"
merge_csv_files(Output_path)
all_model_list= ["gemma-2-2b-it", "gemma-2-9b-it", "gemma-2-27b-it", "llama-3.1-405b-instruct", "llama-3.1-70b-instruct",
"llama-3.1-8b-instruct", "phi-3-mini-128k-instruct", "phi-3-medium-128k-instruct",
 "nemotron-4-340b-instruct", "mistral-nemo-12b-instruct", "mixtral-8x22b-instruct-v0.1", "codestral-22b-instruct-v0.1",
 "mixtral-8x7b-instruct-v0.1", "mistral-large-latest", "mamba-codestral-7b-v0.1",
 "gpt-4o", "gpt-4o-mini", "claude-3-5-sonnet", "Gemini-1.5-pro",
"llama4_maverick","llama4_scout", "llama-3.3-70b-instruct","o3","deepseek-r1-8b",
"deepseek-r1-32b", "deepseek-r1","gemma-3-1b-it","qwen3-235b-a22b","claude-3-7-sonnet-20250219",
"claude-4-sonnet-20250514","Gemini-2.5-pro","gpt-4.1-mini", "gpt-4.1-nano",
"gpt-4.1","o4-mini","llama3.1-8b-f2","llama3.3-70b-sft1","llama3.1-8b-lora1","llama4-109b-lora1","llama3.3-70b-lora1"]

# The evaluated_model is set by run_multiple_judges.sh script
evaluated_model = "claude-3-5-sonnet"
test_model_list = all_model_list

system_list = ["art", "beam", "buckling", "cable",  "camera", "citybus", "curiosity", "feda", "gator", "gear", "gps_imu", "handler", "hmmwv", "kraz", "lidar", "m113", "man", "mass_spring_damper", "particles", "pendulum",
               "rigid_highway", "rigid_multipatches", "rotor", "scm", "scm_hill", "sedan", "sensros", "slider_crank", "tablecloth", "turtlebot", "uazbus", "veh_app","vehros","viper"]
#system_do_list= ["rotor", "scm", "scm_hill", "sedan", "sensros", "slider_crank", "tablecloth", "turtlebot", "uazbus", "veh_app","vehros","viper"]
system_do_list=system_list
def process_model_system(test_model, system_folder, dataset_path, Output_path, Output_conversation_path,
                         Output_statistic_path):
    # Add model-specific delays to avoid hitting API rate limits
    # OpenAI models need longer delays due to strict rate limits
    if evaluated_model in ["gpt-4o", "gpt-4o-mini", "gpt-4.1", "gpt-4.1-mini", "gpt-4.1-nano", "o3", "o4-mini"]:
        delay = random.uniform(2.0, 4.0)  # Longer delay for OpenAI models
    elif evaluated_model in ["claude-3-5-sonnet", "claude-3-7-sonnet-20250219", "claude-4-sonnet-20250514"]:
        delay = random.uniform(1.0, 2.0)  # Moderate delay for Anthropic models
    else:
        delay = random.uniform(0.5, 1.5)  # Standard delay for other models
    
    time.sleep(delay)
    
    system_folder_path = os.path.join(dataset_path, system_folder)
    output_system_path = os.path.join(Output_path, test_model, system_folder)
    os.makedirs(output_system_path, exist_ok=True)

    if system_folder in system_do_list:
        print(f'Processing model {test_model} on system {system_folder}')

        # Read the three response Python files
        first_response_path = os.path.join(output_system_path, "first_response.py")
        second_response_path = os.path.join(output_system_path, "second_response.py")
        third_response_path = os.path.join(output_system_path, "third_response.py")

        first_prediction = read_script(first_response_path)
        second_prediction = read_script(second_response_path)
        third_prediction = read_script(third_response_path)

        first_reference_path = os.path.join(system_folder_path, 'truth1.py')
        second_reference_path = os.path.join(system_folder_path, 'truth2.py')
        third_reference_path = os.path.join(system_folder_path, 'truth3.py')

        first_reference = read_script(first_reference_path)
        second_reference = read_script(second_reference_path)
        third_reference = read_script(third_reference_path)

        api_path = read_script(os.path.join(r'/home/hongyu/Documents/SimBench/api', 'api.txt'))

        # Example usage for first, second, and third rounds
        evaluate_and_save_results("first", first_prediction, first_reference, api_path, output_system_path)
        evaluate_and_save_results("second", second_prediction, second_reference, api_path, output_system_path)
        evaluate_and_save_results("third", third_prediction, third_reference, api_path, output_system_path)

        # Save the scores and metadata to CSV
        save_scores_to_csv_with_metadata(output_system_path, test_model, system_folder, "evaluation_scores.csv")

    return f"Completed {system_folder} for model {test_model}"


# Parallel processing for all models and systems
# Adjust max_workers based on the judge model to avoid rate limits
# OpenAI models need lower concurrency due to strict rate limits
if evaluated_model in ["gpt-4o", "gpt-4o-mini", "gpt-4.1", "gpt-4.1-mini", "gpt-4.1-nano", "o3", "o4-mini"]:
    max_workers = 2  # Low concurrency for OpenAI models
    print(f"Using reduced concurrency (max_workers={max_workers}) for OpenAI judge model: {evaluated_model}")
elif evaluated_model in ["claude-3-5-sonnet", "claude-3-7-sonnet-20250219", "claude-4-sonnet-20250514"]:
    max_workers = 3  # Moderate concurrency for Anthropic models
    print(f"Using moderate concurrency (max_workers={max_workers}) for Anthropic judge model: {evaluated_model}")
else:
    max_workers = 5  # Conservative concurrency for other models
    print(f"Using conservative concurrency (max_workers={max_workers}) for judge model: {evaluated_model}")

with ThreadPoolExecutor(max_workers=max_workers) as executor:
    futures = []
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

    # Use tqdm to show the progress bar for all futures
    for future in tqdm(as_completed(futures), total=len(futures)):
        print(future.result())

print("Finished processing all models and systems.")
