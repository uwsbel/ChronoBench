#!/bin/bash

# SimBench LLM Model Availability Checker
# This script tests which LLM models are available with current API keys
# without running full simulations

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}LLM Model Availability Checker${NC}"
echo -e "${BLUE}========================================${NC}\n"

# Get the directory of this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Source API keys from run_pipeline.sh
if [ -f "${SCRIPT_DIR}/run_pipeline.sh" ]; then
    # Extract just the API key exports without running the whole script
    eval $(grep "^export.*API_KEY=" "${SCRIPT_DIR}/run_pipeline.sh")
    echo -e "${GREEN}✓ API keys loaded from run_pipeline.sh${NC}\n"
else
    echo -e "${RED}✗ run_pipeline.sh not found${NC}"
    exit 1
fi

# Activate conda environment
echo -e "${BLUE}Activating conda environment...${NC}"
source ~/anaconda3/etc/profile.d/conda.sh 2>/dev/null || source ~/miniconda3/etc/profile.d/conda.sh 2>/dev/null
conda activate chrono
echo -e "${GREEN}✓ Conda environment activated${NC}\n"

# Initialize report file
REPORT_FILE="${SCRIPT_DIR}/llm_availability_report.txt"
echo "=== LLM Model Availability Report ===" > "$REPORT_FILE"
echo "Generated: $(date)" >> "$REPORT_FILE"
echo "========================================" >> "$REPORT_FILE"
echo "" >> "$REPORT_FILE"

# Function to test models
test_models() {
    local provider="$1"
    echo -e "${BLUE}Testing $provider models...${NC}"
    echo "[$provider Models]" >> "$REPORT_FILE"
    shift
    "$@" >> "$REPORT_FILE" 2>&1
    echo "" >> "$REPORT_FILE"
}

# Test OpenAI models
if [ -n "$OPENAI_API_KEY" ]; then
    test_models "OpenAI" python -c "
from openai import OpenAI
import os

models = [
    'gpt-4.1-mini', 
    'gpt-4o-mini-f1', 
    'gpt-4.1', 
    'gpt-4o-mini',
    'gpt-4.1-nano', 
    'o4-mini', 
    'o3', 
    'gpt-4o-mini-f3',
    'gpt-4o'
]

api_key = os.environ.get('OPENAI_API_KEY', '')
if not api_key:
    print('⚠ No OpenAI API key found')
else:
    client = OpenAI(api_key=api_key)
    for model in models:
        try:
            response = client.chat.completions.create(
                model=model,
                messages=[{'role': 'user', 'content': 'test'}],
                max_tokens=1,
                temperature=0
            )
            print(f'✓ {model}: Available')
        except Exception as e:
            error = str(e).replace('\\n', ' ')[:100]
            print(f'✗ {model}: {error}')
"
else
    echo "⚠ OpenAI API key not configured" >> "$REPORT_FILE"
    echo "" >> "$REPORT_FILE"
fi

# Test Claude models
if [ -n "$ANTHROPIC_API_KEY" ]; then
    test_models "Anthropic/Claude" python -c "
import anthropic
import os

models = [
    'claude-3-7-sonnet-20250219',
    'claude-3-5-sonnet',
    'claude-3-5-sonnet-20241022',
    'claude-4-sonnet-20250514',
    'claude-3-opus-20240229',
    'claude-3-haiku-20240307'
]

api_key = os.environ.get('ANTHROPIC_API_KEY', '')
if not api_key:
    print('⚠ No Anthropic API key found')
else:
    client = anthropic.Anthropic(api_key=api_key)
    for model in models:
        try:
            response = client.messages.create(
                model=model,
                messages=[{'role': 'user', 'content': 'test'}],
                max_tokens=1,
                temperature=0
            )
            print(f'✓ {model}: Available')
        except Exception as e:
            error = str(e).replace('\\n', ' ')[:100]
            print(f'✗ {model}: {error}')
"
else
    echo "⚠ Anthropic API key not configured" >> "$REPORT_FILE"
    echo "" >> "$REPORT_FILE"
fi

# Test Google models
if [ -n "$GOOGLE_API_KEY" ]; then
    test_models "Google/Gemini" python -c "
import google.generativeai as genai
import os

models = [
    'gemini-1.5-pro',
    'gemini-1.5-pro-latest',
    'gemini-1.5-flash',
    'gemini-1.5-flash-latest',
    'gemini-2.0-flash-exp',
    'gemini-pro',
    'Gemini-1.5-pro-1.5-pro',  # From script
    'Gemini-1.5-pro-2.5-pro'   # From script
]

api_key = os.environ.get('GOOGLE_API_KEY', '')
if not api_key:
    print('⚠ No Google API key found')
else:
    genai.configure(api_key=api_key)
    for model_name in models:
        try:
            model = genai.GenerativeModel(model_name)
            response = model.generate_content('test', 
                generation_config={'max_output_tokens': 1, 'temperature': 0})
            print(f'✓ {model_name}: Available')
        except Exception as e:
            error = str(e).replace('\\n', ' ')[:100]
            print(f'✗ {model_name}: {error}')
"
else
    echo "⚠ Google API key not configured" >> "$REPORT_FILE"
    echo "" >> "$REPORT_FILE"
fi

# Test Mistral models
if [ -n "$MISTRAL_API_KEY" ]; then
    test_models "Mistral" python -c "
from mistralai import Mistral
import os

models = [
    'mistral-large-latest',
    'mistral-large-2411',
    'mistral-medium-latest',
    'mistral-small-latest',
    'open-mistral-7b',
    'open-mixtral-8x7b',
    'open-mixtral-8x22b',
    'codestral-latest'
]

api_key = os.environ.get('MISTRAL_API_KEY', '')
if not api_key:
    print('⚠ No Mistral API key found')
else:
    client = Mistral(api_key=api_key)
    for model in models:
        try:
            response = client.chat.complete(
                model=model,
                messages=[{'role': 'user', 'content': 'test'}],
                max_tokens=1,
                temperature=0
            )
            print(f'✓ {model}: Available')
        except Exception as e:
            error = str(e).replace('\\n', ' ')[:100]
            print(f'✗ {model}: {error}')
"
else
    echo "⚠ Mistral API key not configured" >> "$REPORT_FILE"
    echo "" >> "$REPORT_FILE"
fi

# Test NVIDIA/DeepInfra models (from Open_generate_simulation.py)
echo -e "${BLUE}Testing NVIDIA/DeepInfra models...${NC}"
echo "[NVIDIA/DeepInfra Models]" >> "$REPORT_FILE"
python -c "
from openai import OpenAI
import os

# Extract NVIDIA API key from Open_generate_simulation.py
nvidia_api_key = os.getenv('NVIDIA_API_KEY', '')

models = {
    'gemma-2-9b-it': 'google/gemma-2-9b-it',
    'gemma-2-27b-it': 'google/gemma-2-27b-it',
    'llama-3.1-405b': 'meta/llama-3.1-405b-instruct',
    'llama-3.1-70b': 'meta/llama-3.1-70b-instruct',
    'llama-3.1-8b': 'meta/llama-3.1-8b-instruct',
    'phi-3-mini': 'microsoft/phi-3-mini-128k-instruct',
    'nemotron-340b': 'nvidia/nemotron-4-340b-instruct',
    'mistral-nemo': 'nv-mistralai/mistral-nemo-12b-instruct',
    'mixtral-8x22b': 'mistralai/mixtral-8x22b-instruct-v0.1',
    'codestral-22b': 'mistralai/codestral-22b-instruct-v0.1'
}

# Test with DeepInfra endpoint
print('Testing with DeepInfra endpoint:')
client = OpenAI(
    base_url='https://api.deepinfra.com/v1/openai',
    api_key=nvidia_api_key
)
for name, model in list(models.items())[:3]:  # Test first 3 to save credits
    try:
        response = client.chat.completions.create(
            model=model,
            messages=[{'role': 'user', 'content': 'test'}],
            max_tokens=1,
            temperature=0
        )
        print(f'✓ {name}: Available on DeepInfra')
    except Exception as e:
        error = str(e).replace('\\n', ' ')[:80]
        print(f'✗ {name}: {error}')

# Test with NVIDIA NIM endpoint
print('\\nTesting with NVIDIA NIM endpoint:')
client = OpenAI(
    base_url='https://integrate.api.nvidia.com/v1',
    api_key=nvidia_api_key
)
for name, model in list(models.items())[:3]:  # Test first 3
    try:
        response = client.chat.completions.create(
            model=model,
            messages=[{'role': 'user', 'content': 'test'}],
            max_tokens=1,
            temperature=0
        )
        print(f'✓ {name}: Available on NVIDIA NIM')
    except Exception as e:
        error = str(e).replace('\\n', ' ')[:80]
        print(f'✗ {name}: {error}')
" >> "$REPORT_FILE" 2>&1
echo "" >> "$REPORT_FILE"

# Generate summary
echo -e "\n${BLUE}========================================${NC}"
echo -e "${BLUE}Summary${NC}"
echo -e "${BLUE}========================================${NC}"

echo "=== SUMMARY ===" >> "$REPORT_FILE"
echo -n "Total models tested: " >> "$REPORT_FILE"
grep -c "^[✓✗]" "$REPORT_FILE" >> "$REPORT_FILE" || echo "0" >> "$REPORT_FILE"
echo -n "Available models: " >> "$REPORT_FILE"
grep -c "^✓" "$REPORT_FILE" >> "$REPORT_FILE" || echo "0" >> "$REPORT_FILE"
echo -n "Unavailable models: " >> "$REPORT_FILE"
grep -c "^✗" "$REPORT_FILE" >> "$REPORT_FILE" || echo "0" >> "$REPORT_FILE"

# Display report
echo -e "\n${GREEN}Report saved to: $REPORT_FILE${NC}\n"
echo "Contents:"
echo "----------------------------------------"
cat "$REPORT_FILE"
echo "----------------------------------------"

echo -e "\n${GREEN}✓ Availability check complete!${NC}"