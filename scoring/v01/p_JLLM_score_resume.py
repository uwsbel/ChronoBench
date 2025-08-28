#!/usr/bin/env python3
"""
Modified version of p_JLLM_score.py with resume capability.
Skips already evaluated model/system combinations.
"""

import os
import sys
import json
from pathlib import Path

# Import everything from the original script
sys.path.insert(0, '/home/hongyu/Documents/SimBench/scoring/v01')

# We'll dynamically import and modify the behavior
original_script = '/home/hongyu/Documents/SimBench/scoring/v01/p_JLLM_score.py'

# Read and modify the original script
with open(original_script, 'r') as f:
    script_content = f.read()

# Function to check if a model/system has been evaluated
def is_already_evaluated(model_path, system_folder):
    """Check if all score files exist for a model/system combination"""
    required_files = [
        'first_score_document.txt',
        'first_score_reference.txt',
        'first_score_reference_document.txt',
        'second_score_document.txt',
        'second_score_reference.txt', 
        'second_score_reference_document.txt',
        'third_score_document.txt',
        'third_score_reference.txt',
        'third_score_reference_document.txt',
        'evaluation_scores.csv'
    ]
    
    system_path = os.path.join(model_path, system_folder)
    
    # If directory doesn't exist, not evaluated
    if not os.path.exists(system_path):
        return False
    
    # Check if all required files exist
    for file in required_files:
        if not os.path.exists(os.path.join(system_path, file)):
            return False
    
    return True

# Inject resume logic into the script
resume_code = '''
# Resume capability injection
import os
from pathlib import Path

def is_already_evaluated(output_path, test_model, system_folder):
    """Check if all score files exist for a model/system combination"""
    required_files = [
        'first_score_document.txt',
        'first_score_reference.txt', 
        'first_score_reference_document.txt',
        'second_score_document.txt',
        'second_score_reference.txt',
        'second_score_reference_document.txt',
        'third_score_document.txt',
        'third_score_reference.txt',
        'third_score_reference_document.txt',
        'evaluation_scores.csv'
    ]
    
    system_path = os.path.join(output_path, test_model, system_folder)
    
    # If directory doesn't exist, not evaluated
    if not os.path.exists(system_path):
        return False
    
    # Check if all required files exist
    for file in required_files:
        file_path = os.path.join(system_path, file)
        if not os.path.exists(file_path):
            return False
    
    # Additional check: ensure files are not empty
    for file in required_files:
        file_path = os.path.join(system_path, file)
        if os.path.getsize(file_path) == 0:
            return False
            
    return True

# Modify the process_model_system function to check for existing evaluations
_original_process_model_system = process_model_system

def process_model_system_with_resume(test_model, system_folder, dataset_path, Output_path, Output_conversation_path, Output_statistic_path):
    # Check if already evaluated
    if is_already_evaluated(Output_path, test_model, system_folder):
        print(f"⏭️ Skipping already evaluated: {test_model}/{system_folder}")
        return f"Skipped (already evaluated): {system_folder} for model {test_model}"
    
    # Call original function
    return _original_process_model_system(test_model, system_folder, dataset_path, Output_path, Output_conversation_path, Output_statistic_path)

# Replace the function
process_model_system = process_model_system_with_resume

print("\\n" + "="*60)
print("RESUME MODE ENABLED")
print("Will skip already evaluated model/system combinations")
print("="*60 + "\\n")

# Count and display what will be evaluated
total_combinations = len(test_model_list) * len(system_do_list)
to_evaluate = 0
skipped = 0

for test_model in test_model_list:
    for system_folder in system_do_list:
        if is_already_evaluated(Output_path, test_model, system_folder):
            skipped += 1
        else:
            to_evaluate += 1

print(f"Total combinations: {total_combinations}")
print(f"Already evaluated (will skip): {skipped}")
print(f"To be evaluated: {to_evaluate}")
print("="*60 + "\\n")
'''

# Find where to inject the resume code (after the process_model_system function definition)
import_pos = script_content.find('def process_model_system(')
if import_pos != -1:
    # Find the end of the function
    func_end = script_content.find('\ndef ', import_pos + 1)
    if func_end == -1:
        func_end = script_content.find('\n# Parallel processing', import_pos)
    
    if func_end != -1:
        # Insert the resume code right before parallel processing section
        modified_script = script_content[:func_end] + '\n\n' + resume_code + '\n' + script_content[func_end:]
    else:
        modified_script = script_content
else:
    modified_script = script_content

# Execute the modified script
exec(modified_script)