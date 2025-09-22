#!/usr/bin/env python3
"""Test NVIDIA MODEL_REGISTRY to verify all model IDs are valid."""

import os
from openai import OpenAI
import time
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

# Get API key
nvidia_api_key = os.getenv("NVIDIA_API_KEY")
if not nvidia_api_key:
    print("❌ NVIDIA_API_KEY not set in .env file or environment")
    print("Please create a .env file with: NVIDIA_API_KEY=your_key_here")
    exit(1)
else:
    print(f"✅ NVIDIA_API_KEY loaded from .env (length: {len(nvidia_api_key)})")

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
    "deepseek-r1":                   ("nvidia", "deepseek-ai/deepseek-r1"),
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

def test_token_limit(model_name, model_path):
    """Test token limits for the model - both 4096 and 16384."""
    client = OpenAI(
        base_url="https://integrate.api.nvidia.com/v1",
        api_key=nvidia_api_key,
        timeout=10.0
    )

    # First try with 16384 tokens (4096*4)
    try:
        completion = client.chat.completions.create(
            model=model_path,
            messages=[{"role": "user", "content": "Say 'OK'"}],
            temperature=0.1,
            max_tokens=16384,  # 4096*4
            stream=False
        )
        return "16384", "Supports 16384 tokens (4096*4)"
    except Exception as e:
        error_msg = str(e)

    # If 16384 failed, try with 4096 tokens
    try:
        completion = client.chat.completions.create(
            model=model_path,
            messages=[{"role": "user", "content": "Say 'OK'"}],
            temperature=0.1,
            max_tokens=4096,
            stream=False
        )
        return "4096", "Limited to 4096 tokens"
    except Exception as e:
        error_msg = str(e)
        if "422" in error_msg:
            # Extract the actual limit from error message if possible
            import re
            match = re.search(r'less than or equal to (\d+)', error_msg)
            if match:
                limit = match.group(1)
                return limit, f"Limited to {limit} tokens (from error)"
            return "unknown", f"422 Error: {error_msg[:50]}"
        else:
            return "error", f"Error: {error_msg[:50]}"

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

    # Test token limits
    print("\n" + "=" * 60)
    print("TOKEN LIMIT TESTING:")
    print("=" * 60)
    print("Testing token limits (4096 and 16384) for all working models...")

    token_limits = []
    for i, (model_name, model_path) in enumerate(nvidia_models, 1):
        # Only test working models
        if any(model_name in str(r) and "✅" in str(r) for r in results):
            print(f"\n[{i}/{len(nvidia_models)}] Testing token limit: {model_name}")

            limit, limit_msg = test_token_limit(model_name, model_path)

            if limit == "16384":
                print(f"    ✅ {limit_msg}")
            elif limit == "4096":
                print(f"    ⚠️  {limit_msg}")
            else:
                print(f"    ❌ {limit_msg}")

            token_limits.append((model_name, model_path, limit))
            time.sleep(2)  # Rate limit protection

    # Summary of models by token limit
    print("\n" + "=" * 60)
    print("TOKEN LIMIT SUMMARY:")
    print("=" * 60)

    # Models with 4096 limit
    limited_4096 = [(m, p) for m, p, limit in token_limits if limit == "4096"]
    if limited_4096:
        print("\nMODELS WITH 4096 TOKEN LIMIT:")
        for model, _ in limited_4096:
            print(f"  - {model}")

        print("\nAdd these to MODELS_WITH_4096_LIMIT in full_pipeline_parallel.py:")
        for _, path in limited_4096:
            print(f'    "{path}",')

    # Models with 16384 limit
    supports_16384 = [(m, p) for m, p, limit in token_limits if limit == "16384"]
    if supports_16384:
        print("\nMODELS SUPPORTING 16384 TOKENS (4096*4):")
        for model, _ in supports_16384:
            print(f"  - {model}")

    # Models with other/unknown limits
    other_limits = [(m, p, limit) for m, p, limit in token_limits if limit not in ["4096", "16384"]]
    if other_limits:
        print("\nMODELS WITH OTHER/UNKNOWN LIMITS:")
        for model, _, limit in other_limits:
            print(f"  - {model}: {limit}")

if __name__ == "__main__":
    main()