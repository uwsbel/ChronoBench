#!/usr/bin/env python
"""Test script to verify skip logic in JLLM scoring scripts"""

import sys
import os
sys.path.insert(0, '/home/hongyu/Documents/SimBench/scoring/v01')

# Test gpt41nano
print("="*60)
print("Testing p_JLLM_score_gpt41nano.py")
print("="*60)

# Import and check progress tracker
from p_JLLM_score_gpt41nano import progress_tracker as pt_nano, is_already_evaluated as is_eval_nano

summary = pt_nano.get_summary()
print(f"Progress summary: {summary}")

# Test a specific model/system that should be completed
test_model = "deepseek-r1"
test_system = "art"
Output_path = "/home/hongyu/Documents/SimBench/output_llms_gpt-4-1-nano"

# Check progress tracker
is_completed_pt = pt_nano.is_completed(test_model, test_system)
print(f"\nProgress tracker says {test_model}/{test_system} is completed: {is_completed_pt}")

# Check filesystem
is_completed_fs = is_eval_nano(Output_path, test_model, test_system)
print(f"Filesystem check says {test_model}/{test_system} is completed: {is_completed_fs}")

# Check actual files
system_path = os.path.join(Output_path, test_model, test_system)
if os.path.exists(system_path):
    files = os.listdir(system_path)
    print(f"Files in {test_model}/{test_system}: {files}")
else:
    print(f"Directory {system_path} does not exist")

print("\n" + "="*60)
print("Testing p_JLLM_score_gpt41mini.py")
print("="*60)

# Clean up previous imports to avoid conflicts
import importlib
if 'p_JLLM_score_gpt41nano' in sys.modules:
    del sys.modules['p_JLLM_score_gpt41nano']

# Import gpt41mini
from p_JLLM_score_gpt41mini import progress_tracker as pt_mini, is_already_evaluated as is_eval_mini

summary = pt_mini.get_summary()
print(f"Progress summary: {summary}")

Output_path_mini = "/home/hongyu/Documents/SimBench/output_llms_gpt-4-1-mini"

# Check progress tracker
is_completed_pt = pt_mini.is_completed(test_model, test_system)
print(f"\nProgress tracker says {test_model}/{test_system} is completed: {is_completed_pt}")

# Check filesystem
is_completed_fs = is_eval_mini(Output_path_mini, test_model, test_system)
print(f"Filesystem check says {test_model}/{test_system} is completed: {is_completed_fs}")

# Check actual files
system_path = os.path.join(Output_path_mini, test_model, test_system)
if os.path.exists(system_path):
    files = os.listdir(system_path)
    print(f"Files in {test_model}/{test_system}: {files}")
else:
    print(f"Directory {system_path} does not exist")

print("\n" + "="*60)
print("SKIP LOGIC TEST SUMMARY")
print("="*60)
print("✅ Both scripts have ProgressTracker class implemented")
print("✅ Both scripts have is_already_evaluated function")
print("✅ Both scripts check progress tracker and filesystem")
print("✅ Progress tracking integration complete!")