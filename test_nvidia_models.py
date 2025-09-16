#!/usr/bin/env python3
"""Test NVIDIA MODEL_REGISTRY to verify all model IDs are valid."""

import os
from openai import OpenAI
import time

# Get API key
nvidia_api_key = os.getenv("NVIDIA_API_KEY")
if not nvidia_api_key:
    print("❌ NVIDIA_API_KEY not set")
    exit(1)

# Model registry from full_pipeline_parallel.py (with fixes for 404 errors)
MODEL_REGISTRY = {
    # NVIDIA NIM API models (using NVIDIA API for all OSS models)
    "qwen3-235b-a22b":               ("nvidia", "qwen/qwen3-235b-a22b"),
    "gemma-2-27b-it":                ("nvidia", "google/gemma-2-27b-it"),
    "gemma-2-9b-it":                 ("nvidia", "google/gemma-2-9b-it"),
    "gemma-2-2b-it":                 ("nvidia", "google/gemma-2-2b-it"),
    "gemma-3-1b-it":                 ("nvidia", "google/gemma-3-1b-it"),
    "gemma-3-27b-it":                ("nvidia", "google/gemma-3-27b-it"),  # FIXED: Removed nvdev/
    "llama4_maverick":               ("nvidia", "meta/llama-4-maverick-17b-128e-instruct"),  # FIXED: Use meta/ instead of nvdev/meta/
    "llama4_scout":                  ("nvidia", "meta/llama-4-scout-17b-16e-instruct"),  # FIXED: Use meta/ instead of nvdev/meta/
    "llama-3.3-70b-instruct":        ("nvidia", "meta/llama-3.3-70b-instruct"),  # FIXED: Removed nvdev/
    "llama-3.1-405b-instruct":       ("nvidia", "meta/llama-3.1-405b-instruct"),
    "llama-3.1-70b-instruct":        ("nvidia", "meta/llama-3.1-70b-instruct"),
    "llama-3.1-8b-instruct":         ("nvidia", "meta/llama-3.1-8b-instruct"),
    "mixtral-8x22b-instruct-v0.1":   ("nvidia", "mistralai/mixtral-8x22b-instruct-v0.1"),  # FIXED: Added -v0.1
    "mixtral-8x7b-instruct-v0.1":    ("nvidia", "mistralai/mixtral-8x7b-instruct-v0.1"),  # FIXED: Added -v0.1
    "codestral-22b-instruct-v0.1":   ("nvidia", "mistralai/codestral-22b-instruct-v0.1"),  # FIXED: Added -v0.1
    "mistral-nemo-12b-instruct":     ("nvidia", "nv-mistralai/mistral-nemo-12b-instruct"),
    "mamba-codestral-7b-v0.1":       ("nvidia", "mistralai/mamba-codestral-7b-v0.1"),
    "deepseek-r1-8b":                ("nvidia", "deepseek-ai/deepseek-r1-distill-llama-8b"),
    "deepseek-r1-32b":               ("nvidia", "deepseek-ai/deepseek-r1-distill-qwen-32b"),
    "deepseek-r1":                   ("nvidia", "deepseek-ai/deepseek-r1-0528"),
    "phi-3-mini-128k-instruct":      ("nvidia", "microsoft/phi-3-mini-128k-instruct"),
    "phi-3-medium-128k-instruct":    ("nvidia", "microsoft/phi-3-medium-128k-instruct"),  # FIXED: Lowercase phi

    # Additional models from p_NIM.py
    "mistral-small-3.1-24b-instruct-2503": ("nvidia", "mistralai/mistral-small-3.1-24b-instruct-2503"),
    "mistral-medium-3-instruct":     ("nvidia", "mistralai/mistral-medium-3-instruct"),
    "qwq-32b":                        ("nvidia", "qwen/qwq-32b"),
    "qwen3-7b-instuct":               ("nvidia", "qwen/qwen2-7b-instruct"),
    "phi-4-mini-instruct":            ("nvidia", "microsoft/phi-4-mini-instruct"),
}

def test_model(model_name, model_path):
    """Test a single model with a simple prompt."""
    client = OpenAI(
        base_url="https://integrate.api.nvidia.com/v1",
        api_key=nvidia_api_key,
        timeout=10.0
    )

    try:
        # Simple test prompt
        completion = client.chat.completions.create(
            model=model_path,
            messages=[{"role": "user", "content": "Say 'OK' if you work"}],
            temperature=0.1,
            max_tokens=10,
            stream=False
        )
        # Check if completion has expected structure
        if completion and completion.choices and len(completion.choices) > 0:
            if completion.choices[0].message and completion.choices[0].message.content:
                return True, completion.choices[0].message.content[:20]
            else:
                return False, "Empty response from model"
        else:
            return False, "Invalid response structure"
    except Exception as e:
        error_msg = str(e)
        if "404" in error_msg:
            return False, "404 Not Found - Invalid model ID"
        elif "503" in error_msg:
            return False, "503 Service Unavailable"
        elif "429" in error_msg:
            return False, "429 Rate Limit"
        else:
            return False, f"Error: {error_msg[:50]}"

def main():
    print("Testing NVIDIA MODEL_REGISTRY entries...")
    print("=" * 60)

    results = []
    nvidia_models = [(name, path) for name, (provider, path) in MODEL_REGISTRY.items() if provider == "nvidia"]

    for i, (model_name, model_path) in enumerate(nvidia_models, 1):
        print(f"\n[{i}/{len(nvidia_models)}] Testing: {model_name}")
        print(f"    Path: {model_path}")

        success, response = test_model(model_name, model_path)

        if success:
            print(f"    ✅ SUCCESS: {response}")
            results.append((model_name, "✅ Working"))
        else:
            print(f"    ❌ FAILED: {response}")
            results.append((model_name, f"❌ {response}"))

        # Rate limit protection
        if i < len(nvidia_models):
            time.sleep(2)

    print("\n" + "=" * 60)
    print("SUMMARY:")
    print("=" * 60)

    working = sum(1 for _, status in results if "✅" in status)
    failed = sum(1 for _, status in results if "❌" in status)

    print(f"Total models tested: {len(results)}")
    print(f"✅ Working: {working}")
    print(f"❌ Failed: {failed}")

    if failed > 0:
        print("\nFailed models:")
        for name, status in results:
            if "❌" in status:
                print(f"  - {name}: {status}")

    print("\nAll models:")
    for name, status in results:
        print(f"  {name}: {status}")

if __name__ == "__main__":
    main()