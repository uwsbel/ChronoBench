#!/usr/bin/env python3
"""Test script to verify multi-provider support in p_JLLM_score.py"""

import sys
sys.path.insert(0, '/home/hongyu/Documents/SimBench/scoring/v01')

# Import the functions we need to test
from p_JLLM_score import get_provider_for_model

# Test provider detection
test_models = {
    "gpt-4o": "openai",
    "gpt-4o-mini": "openai",
    "claude-3-5-sonnet": "anthropic",
    "claude-3-7-sonnet-20250219": "anthropic", 
    "Gemini-1.5-pro": "google",
    "Gemini-2.5-pro": "google",
    "gemma-2-2b-it": "google",
    "mistral-nemo-12b-instruct": "mistral",
    "mixtral-8x22b-instruct-v0.1": "mistral",
    "llama-3.1-405b-instruct": "nvidia",
    "deepseek-r1": "deepseek",
    "qwen3-235b-a22b": "qwen",
}

print("Testing provider detection:")
print("-" * 50)
for model, expected_provider in test_models.items():
    detected_provider = get_provider_for_model(model)
    status = "✓" if detected_provider == expected_provider else "✗"
    print(f"{status} {model:30s} -> {detected_provider:10s} (expected: {expected_provider})")

print("\nProvider detection test complete!")