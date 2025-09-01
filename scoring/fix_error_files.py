#!/usr/bin/env python3
"""
Re-evaluate files that failed with API errors using gpt-4.1-nano.
This script identifies error files and re-runs the evaluation for them.
"""

import os
import re
import time
import json
import subprocess
from pathlib import Path
import logging

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Read API key from .env file manually
def get_api_key():
    env_file = "/home/hongyu/Documents/SimBench/.env"
    if os.path.exists(env_file):
        with open(env_file, 'r') as f:
            for line in f:
                if line.startswith("OPENAI_API_KEY"):
                    # Extract the key value
                    parts = line.strip().split('=', 1)
                    if len(parts) == 2:
                        key = parts[1].strip().strip('"').strip("'")
                        if "GPT41NANO" in parts[0] or "_3" in parts[0]:
                            return key
                        # Store as fallback
                        fallback_key = key
        return fallback_key if 'fallback_key' in locals() else None
    return None

# API Configuration
api_key = get_api_key()
if not api_key:
    logger.error("No OpenAI API key found in .env file")
    exit(1)

JUDGE_MODEL = "gpt-4.1-nano"

def find_error_files():
    """Find all files that contain error messages."""
    output_path = "/home/hongyu/Documents/SimBench/output_llms_gpt-4-1-nano"
    error_files = []
    
    for root, dirs, files in os.walk(output_path):
        for file in files:
            if ('_score_document.txt' in file or 
                '_score_reference.txt' in file or 
                '_score_reference_document.txt' in file):
                
                file_path = os.path.join(root, file)
                try:
                    with open(file_path, 'r', encoding='utf-8') as f:
                        content = f.read()
                    if content.startswith("FAILED:") or content.startswith("Error:"):
                        error_files.append(file_path)
                except:
                    pass
    
    return error_files

def get_corresponding_files(score_file_path):
    """Get the prediction and reference files for a score file."""
    # Parse the score file path
    dir_path = os.path.dirname(score_file_path)
    file_name = os.path.basename(score_file_path)
    
    # Determine the round and type
    if 'first_score' in file_name:
        round_name = 'first'
    elif 'second_score' in file_name:
        round_name = 'second'
    elif 'third_score' in file_name:
        round_name = 'third'
    else:
        return None, None
    
    # Determine evaluation type
    if 'reference_document' in file_name:
        eval_type = 'reference_document'
    elif 'reference' in file_name:
        eval_type = 'reference'
    else:
        eval_type = 'document'
    
    # Get prediction file
    pred_file = os.path.join(dir_path, f"{round_name}_response.py")
    if not os.path.exists(pred_file):
        pred_file = os.path.join(dir_path, f"{round_name}_response.txt")
    
    # Get reference file
    # Need to find the system path from the directory structure
    parts = dir_path.split('/')
    model_idx = -1
    for i, part in enumerate(parts):
        if 'output_llms' in part:
            model_idx = i + 1
            break
    
    if model_idx == -1 or model_idx >= len(parts):
        return None, None
    
    system_name = parts[model_idx + 1]  # Get system name
    
    # Reference files are in the original demo directory
    ref_base_path = f"/home/hongyu/Documents/SimBench/demo_data/{system_name}"
    
    if eval_type == 'reference' or eval_type == 'reference_document':
        ref_file = os.path.join(ref_base_path, f"truth{round_name[0] if round_name == 'first' else ('2' if round_name == 'second' else '3')}.py")
    else:
        ref_file = None  # Document evaluation doesn't need reference
    
    # Get API documentation if needed
    api_file = "/home/hongyu/Documents/SimBench/api/api.txt" if eval_type == 'document' or eval_type == 'reference_document' else None
    
    return pred_file, ref_file, api_file

def call_openai_api(prompt, api_key, model="gpt-4.1-nano"):
    """Call OpenAI API using curl command."""
    import json
    import subprocess
    
    headers = {
        "Content-Type": "application/json",
        "Authorization": f"Bearer {api_key}"
    }
    
    data = {
        "model": model,
        "messages": [
            {"role": "system", "content": "You are an expert in PyChrono physics simulation library."},
            {"role": "user", "content": prompt}
        ],
        "temperature": 0.1,
        "max_tokens": 4000
    }
    
    # Create curl command
    curl_cmd = [
        "curl", "-s", "-X", "POST",
        "https://api.openai.com/v1/chat/completions",
        "-H", f"Authorization: Bearer {api_key}",
        "-H", "Content-Type: application/json",
        "-d", json.dumps(data)
    ]
    
    try:
        result = subprocess.run(curl_cmd, capture_output=True, text=True, timeout=60)
        if result.returncode == 0:
            response = json.loads(result.stdout)
            if "choices" in response and len(response["choices"]) > 0:
                return response["choices"][0]["message"]["content"]
            elif "error" in response:
                logger.error(f"API error: {response['error']}")
                return None
        else:
            logger.error(f"Curl failed: {result.stderr}")
            return None
    except Exception as e:
        logger.error(f"API call failed: {e}")
        return None

def evaluate_with_jllm(prediction_code, reference_code=None, api_doc=None, eval_type='document'):
    """Evaluate code using the JLLM."""
    
    # Prepare the prompt based on evaluation type
    if eval_type == 'document':
        prompt = f"""
        You are a PyChrono expert tasked with evaluating a simulation script by comparing it against the PyChrono API documentation. While the API documentation provides guidelines, it may not cover all aspects due to length constraints. Therefore, your evaluation should also be based on your knowledge of best practices in Python coding and general simulation principles.

        Here is the PyChrono code you need to evaluate:
        {prediction_code}

        And here is the PyChrono API documentation:
        {api_doc}

        Evaluate the code based on these criteria:
        1. Completeness (40 points): Are all essential components present?
        2. Correctness (30 points): Is the API used correctly? Are there logical errors?
        3. Code Quality (10 points): Is the code well-structured, readable, and documented?
        4. Efficiency (10 points): Is the code efficient without unnecessary computations?
        5. Error Handling and Robustness (5 points): Does it handle edge cases and errors?
        6. Use of Visualization Tools (5 points): Are visualization tools properly utilized?

        Provide a detailed evaluation with specific deductions for each criterion.
        End your evaluation with a final score in the format: [[score]]
        """
    
    elif eval_type == 'reference':
        prompt = f"""
        You are tasked with evaluating a PyChrono simulation script by comparing it to an expert-written reference script.

        Here is the code to evaluate:
        {prediction_code}

        Here is the reference script:
        {reference_code}

        Evaluate based on:
        1. Completeness (40 points): Are all components from the reference present?
        2. Correctness (30 points): Does it match the reference's logic and API usage?
        3. Code Quality (10 points): Is it as well-structured as the reference?
        4. Efficiency (10 points): Is it as efficient as the reference?
        5. Error Handling (5 points): Does it handle errors like the reference?
        6. Visualization (5 points): Does it match the reference's visualization setup?

        Provide a detailed evaluation with specific deductions.
        End with a final score in the format: [[score]]
        """
    
    else:  # reference_document
        prompt = f"""
        You are evaluating a PyChrono simulation script using both a reference implementation and API documentation.

        Code to evaluate:
        {prediction_code}

        Reference implementation:
        {reference_code}

        API documentation:
        {api_doc}

        Evaluate the code by comparing it to the reference while considering the API documentation for best practices.
        
        Criteria:
        1. Completeness (40 points): Are all essential components from the reference present?
        2. Correctness (30 points): Does it correctly implement the reference's functionality according to the API?
        3. Code Quality (10 points): Is the code well-structured and readable?
        4. Efficiency (10 points): Is the implementation efficient?
        5. Error Handling (5 points): Does it handle potential errors?
        6. Visualization (5 points): Is visualization properly implemented?

        Provide detailed evaluation with specific deductions.
        End with a final score in the format: [[score]]
        """
    
    # Use the curl-based API call
    return call_openai_api(prompt, api_key, JUDGE_MODEL)

def process_error_file(error_file_path):
    """Process a single error file and re-evaluate it."""
    logger.info(f"Processing: {error_file_path}")
    
    # Determine evaluation type from filename
    if 'reference_document' in error_file_path:
        eval_type = 'reference_document'
    elif 'reference' in error_file_path:
        eval_type = 'reference'
    else:
        eval_type = 'document'
    
    # Get corresponding files
    pred_file, ref_file, api_file = get_corresponding_files(error_file_path)
    
    if not pred_file or not os.path.exists(pred_file):
        logger.warning(f"  Prediction file not found for {error_file_path}")
        return False
    
    # Read files
    try:
        with open(pred_file, 'r', encoding='utf-8') as f:
            prediction_code = f.read()
        
        reference_code = None
        if ref_file and os.path.exists(ref_file):
            with open(ref_file, 'r', encoding='utf-8') as f:
                reference_code = f.read()
        
        api_doc = None
        if api_file and os.path.exists(api_file):
            with open(api_file, 'r', encoding='utf-8') as f:
                api_doc = f.read()[:20000]  # Limit API doc size
        
    except Exception as e:
        logger.error(f"  Error reading files: {e}")
        return False
    
    # Re-evaluate
    result = evaluate_with_jllm(prediction_code, reference_code, api_doc, eval_type)
    
    if result:
        # Save the new evaluation
        try:
            with open(error_file_path, 'w', encoding='utf-8') as f:
                f.write(result)
            logger.info(f"  ✅ Successfully re-evaluated")
            return True
        except Exception as e:
            logger.error(f"  Error saving result: {e}")
            return False
    else:
        logger.error(f"  ❌ Evaluation failed")
        return False

def main():
    """Main function to fix all error files."""
    
    logger.info("Finding error files...")
    error_files = find_error_files()
    logger.info(f"Found {len(error_files)} error files to fix\n")
    
    if not error_files:
        logger.info("No error files found!")
        return
    
    # Group by model to process systematically
    by_model = {}
    for file_path in error_files:
        parts = file_path.split('/')
        for i, part in enumerate(parts):
            if 'output_llms' in part and i + 1 < len(parts):
                model = parts[i + 1]
                if model not in by_model:
                    by_model[model] = []
                by_model[model].append(file_path)
                break
    
    # Process each model's error files
    total_fixed = 0
    total_failed = 0
    
    for model, files in by_model.items():
        logger.info(f"\nProcessing {model}: {len(files)} files")
        
        for i, file_path in enumerate(files, 1):
            logger.info(f"  [{i}/{len(files)}] {os.path.basename(os.path.dirname(file_path))}/{os.path.basename(file_path)}")
            
            if process_error_file(file_path):
                total_fixed += 1
            else:
                total_failed += 1
            
            # Rate limiting
            time.sleep(1)  # Wait 1 second between API calls
    
    # Summary
    logger.info("\n" + "="*50)
    logger.info("SUMMARY")
    logger.info("="*50)
    logger.info(f"✅ Successfully fixed: {total_fixed} files")
    logger.info(f"❌ Failed to fix: {total_failed} files")
    logger.info(f"📊 Total processed: {len(error_files)} files")

if __name__ == "__main__":
    main()