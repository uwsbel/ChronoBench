#!/usr/bin/env python3
"""Test qwen3-235b-a22b model to debug invalid response issue."""

import os
from openai import OpenAI
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

# Get API key
nvidia_api_key = os.getenv("NVIDIA_API_KEY")
if not nvidia_api_key:
    print("❌ NVIDIA_API_KEY not set in .env file")
    exit(1)

print("Testing qwen3-235b-a22b model...")
print("=" * 60)

# Test 1: Basic non-streaming request
print("\nTest 1: Non-streaming request")
try:
    client = OpenAI(
        base_url="https://integrate.api.nvidia.com/v1",
        api_key=nvidia_api_key,
        timeout=30.0
    )

    resp = client.chat.completions.create(
        model="qwen/qwen3-235b-a22b",
        messages=[{"role": "user", "content": "Say 'OK' if you work"}],
        max_tokens=10,
        stream=False
    )

    if resp and resp.choices and len(resp.choices) > 0:
        content = resp.choices[0].message.content
        print(f"✅ Success: {content}")
    else:
        print(f"❌ Invalid response structure: {resp}")

except Exception as e:
    print(f"❌ Error: {e}")

# Test 2: Streaming request
print("\nTest 2: Streaming request")
try:
    client = OpenAI(
        base_url="https://integrate.api.nvidia.com/v1",
        api_key=nvidia_api_key,
        timeout=30.0
    )

    resp = client.chat.completions.create(
        model="qwen/qwen3-235b-a22b",
        messages=[{"role": "user", "content": "Say 'OK' if you work"}],
        max_tokens=10,
        stream=True
    )

    print("Response chunks: ", end="")
    chunk_count = 0
    none_count = 0
    content = ""

    for chunk in resp:
        chunk_count += 1
        if chunk.choices and len(chunk.choices) > 0:
            delta = chunk.choices[0].delta
            if hasattr(delta, 'content') and delta.content is not None:
                content += delta.content
                print(f"[{delta.content}]", end="")
            else:
                none_count += 1
                print("[None]", end="")
        else:
            print("[Empty]", end="")

    print(f"\n✅ Received {chunk_count} chunks, {none_count} were None")
    print(f"Final content: '{content}'")

except Exception as e:
    print(f"❌ Error: {e}")

# Test 3: Token limit test
print("\nTest 3: Testing token limits")
for max_tokens in [4096, 8192, 16384]:
    print(f"\n  Testing max_tokens={max_tokens}...")
    try:
        client = OpenAI(
            base_url="https://integrate.api.nvidia.com/v1",
            api_key=nvidia_api_key,
            timeout=10.0
        )

        resp = client.chat.completions.create(
            model="qwen/qwen3-235b-a22b",
            messages=[{"role": "user", "content": "Say 'OK'"}],
            max_tokens=max_tokens,
            stream=False
        )
        print(f"    ✅ Supports max_tokens={max_tokens}")
    except Exception as e:
        error_msg = str(e)
        if "422" in error_msg:
            print(f"    ❌ 422 Error at max_tokens={max_tokens}")
        else:
            print(f"    ❌ Other error: {error_msg[:50]}")

print("\n" + "=" * 60)
print("Testing complete!")