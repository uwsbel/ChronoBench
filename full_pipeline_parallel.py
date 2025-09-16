# -*- coding: utf-8 -*-
"""
SimBench full pipeline: generate -> extract/clean -> execute -> LLM scoring -> CSVs.
- Automatically loads API keys from .env file (OpenAI / Anthropic / Google / Mistral / NVIDIA NIM / OpenRouter).
- Runs all models you confirmed from the screenshot.
- Writes per-model CSVs and one combined CSV at the end.

Dependencies (install as needed):
  pip install openai anthropic google-generativeai mistralai requests evaluate tqdm python-dotenv

Note: Paths default to your Windows layout; override with env vars if desired:
  SIMBENCH_DATASET, SIMBENCH_OUTPUT, SIMBENCH_CONV, SIMBENCH_STAT
"""

import os, re, csv, json, time, logging, subprocess, io, requests
from datetime import datetime
from typing import Dict, Tuple, Optional, Any, List, Union
from tqdm import tqdm
import concurrent.futures
from concurrent.futures import ThreadPoolExecutor, as_completed  # <-- added for parallel judging
from dotenv import load_dotenv  # Added for .env file support
from time import sleep  # Added for retry logic

from rankings import RankingSystem
from functools import wraps

# -----------------------------
# Continuous retry decorator for API calls
# -----------------------------
def retry_until_success(func):
    """Decorator that retries indefinitely until successful response."""
    @wraps(func)
    def wrapper(*args, **kwargs):
        attempt = 0
        delay = 2.0  # Initial delay in seconds

        while True:
            attempt += 1
            try:
                response = func(*args, **kwargs)
                # Validate response is not an error
                if response and not str(response).startswith("[ERROR]"):
                    if attempt > 1:
                        print(f"Success on attempt {attempt}")
                    return response
                else:
                    print(f"Invalid response on attempt {attempt}, retrying...")
            except Exception as e:
                error_str = str(e)
                # Log the specific error type
                if '504' in error_str:
                    print(f"504 Gateway Timeout on attempt {attempt}, retrying in {delay:.1f}s...", flush=True)
                elif '503' in error_str:
                    print(f"503 Service Unavailable on attempt {attempt}, retrying in {delay:.1f}s...", flush=True)
                elif '502' in error_str:
                    print(f"502 Bad Gateway on attempt {attempt}, retrying in {delay:.1f}s...", flush=True)
                else:
                    print(f"Error on attempt {attempt}: {error_str[:100]}, retrying in {delay:.1f}s...", flush=True)

                sleep(delay)
                # Exponential backoff with cap at 60 seconds
                delay = min(delay * 1.5, 60.0)

                # Reset delay after many attempts to handle long outages
                if attempt % 10 == 0:
                    print(f"{attempt} attempts made, continuing...")
                    delay = 2.0  # Reset delay to avoid too long waits
    return wrapper

# -----------------------------
# Configuration flags for skip logic
# -----------------------------
SKIP_EXISTING_SIMULATIONS = False  # Set to False to regenerate simulations even if they exist
SKIP_EXISTING_SCORING = False      # Set to False to re-score even if scores exist

# Parallel processing configuration
# For NVIDIA: Large models self-rate-limit, small models need throttling
# For OpenAI: Can use more workers due to higher rate limits
MAX_GENERATION_WORKERS = 1   # Default, will be overridden per provider/model
MAX_SCORING_WORKERS = 20     # Aggressive for OpenAI's 5000 RPM limit

def get_nvidia_workers(model_id: str) -> int:
    """Determine optimal worker count for NVIDIA models based on size.
    Large models take longer and naturally stay under 40 RPM.
    Small models complete quickly and need throttling.
    """
    model_lower = model_id.lower()

    # Mega models (>300B): Very slow, can use many workers
    # Check for deepseek-r1 (not the distilled versions)
    if "deepseek-r1" in model_lower and "distill" not in model_lower:
        return 10  # DeepSeek-R1 671B: ~0.5 RPM per worker = 5 RPM total
    elif "405b" in model_lower:
        return 10  # Llama-405B: ~0.5 RPM per worker = 5 RPM total
    elif "340b" in model_lower:
        return 10  # Nemotron-340B: ~0.5 RPM per worker = 5 RPM total

    # Large models (70-200B): Slow, can use several workers
    elif "qwen3-235b" in model_lower:
        return 10  # Qwen3-235B: ~0.8 RPM per worker = 6-7 RPM total
    elif "mixtral-8x22b" in model_lower:
        return 10  # Mixtral-8x22B (141B MoE): ~1 RPM per worker = 8 RPM total
    elif "mistral-large" in model_lower:
        return 10  # Mistral-Large (123B): ~1 RPM per worker = 8 RPM total
    elif "70b" in model_lower:
        return 10  # 70B models: ~2 RPM per worker = 12 RPM total
    elif "mixtral-8x7b" in model_lower:
        return 10  # Mixtral-8x7B (47B MoE): ~2 RPM per worker = 10 RPM total

    # Medium models (20-50B): Moderate speed
    elif "32b" in model_lower:
        return 10  # 32B models: ~3 RPM per worker = 12 RPM total
    elif "27b" in model_lower:
        return 10  # 27B models: ~3 RPM per worker = 12 RPM total
    elif "22b" in model_lower or "codestral-22b" in model_lower:
        return 10  # 22B models: ~3 RPM per worker = 12 RPM total

    # Small-medium models (10-20B): Faster
    elif "17b" in model_lower or "maverick" in model_lower or "scout" in model_lower:
        return 10  # 17B models: ~4 RPM per worker = 12 RPM total
    elif "14b" in model_lower or "phi-3-medium" in model_lower:
        return 10  # 14B models: ~4 RPM per worker = 12 RPM total
    elif "12b" in model_lower or "nemo" in model_lower:
        return 10  # 12B models: ~4 RPM per worker = 12 RPM total

    # Small models (<10B): Fast, need throttling
    elif "8b" in model_lower or "9b" in model_lower:
        return 10  # 8-9B models: ~8 RPM per worker = 16 RPM total
    elif "7b" in model_lower or "mamba" in model_lower:
        return 10  # 7B models: ~8 RPM per worker = 16 RPM total
    elif "3.8b" in model_lower or "phi-3-mini" in model_lower:
        return 10  # 3.8B Phi-mini: ~10 RPM per worker = 20 RPM total
    else:
        return 10  # 1-2B models: ~10 RPM per worker = 20 RPM total

# -----------------------------
# 1) Paths (edit if needed)
# -----------------------------
DATASET_PATH = os.environ.get("SIMBENCH_DATASET", r"/home/hongyu/Documents/andy_simbench/SimBench/demo_data")
OUTPUT_PATH  = os.environ.get("SIMBENCH_OUTPUT",  r"/home/hongyu/Documents/andy_simbench/SimBench/output_llms")
CONV_PATH    = os.environ.get("SIMBENCH_CONV",    r"/home/hongyu/Documents/andy_simbench/SimBench/output_conversion")
OUTPUT_SIM_PATH = OUTPUT_PATH  # Simulations are stored under the main output path
OUTPUT_SCORE_PATH = OUTPUT_PATH  # Scores are also stored under the main output path
STAT_PATH    = os.environ.get("SIMBENCH_STAT",    r"/home/hongyu/Documents/andy_simbench/SimBench/statistic")

os.makedirs(OUTPUT_PATH, exist_ok=True)
os.makedirs(CONV_PATH, exist_ok=True)
os.makedirs(STAT_PATH, exist_ok=True)

# -----------------------------
# 2) Load API keys from .env file
# -----------------------------
# Load environment variables from .env file
load_dotenv()

print("\n=== Loading API keys from .env file ===")
OPENAI_API_KEY = os.environ.get("OPENAI_API_KEY", "")
NVIDIA_API_KEY = os.environ.get("NVIDIA_API_KEY", "")

# Print status of loaded keys
print(f"OpenAI API key ............: {'Loaded' if OPENAI_API_KEY else '✗ Not found'}")
print(f"NVIDIA NIM API key ........: {'Loaded' if NVIDIA_API_KEY else '✗ Not found'}")

# Export so sub-libs can see them
if OPENAI_API_KEY: os.environ["OPENAI_API_KEY"] = OPENAI_API_KEY

# -----------------------------
# 3) Provider clients (lazy)
# -----------------------------
openai_client = None
nvidia_openai_client = None  # OpenAI-compatible for NIM

def get_openai():
    global openai_client
    if openai_client is None:
        from openai import OpenAI
        openai_client = OpenAI(api_key=OPENAI_API_KEY)
    return openai_client

def get_nvidia_openai():
    global nvidia_openai_client
    if nvidia_openai_client is None:
        from openai import OpenAI
        nvidia_openai_client = OpenAI(
            api_key=NVIDIA_API_KEY,
            base_url="https://integrate.api.nvidia.com/v1",
            timeout=1200.0  # Add 2-minute timeout for large models
        )
    return nvidia_openai_client

# -----------------------------
# 4) Model registry (provider + provider-specific ID)
#    Edit IDs here if any mismatch with your accounts.
# -----------------------------
MODEL_REGISTRY: Dict[str, Tuple[str, str]] = {
    # OpenAI
    "gpt-4.1-nano": ("openai", "gpt-4.1-nano"),
    "gpt-4.1-mini": ("openai", "gpt-4.1-mini"),
    "gpt-4.1":      ("openai", "gpt-4.1"),
    "gpt-4o-mini":  ("openai", "gpt-4o-mini"),
    "gpt-4o":       ("openai", "gpt-4o"),
    "o3":           ("openai", "o3"),
    "o4-mini":      ("openai", "o4-mini"),

    # Anthropic
    "claude-4-sonnet-20250514":   ("anthropic", "claude-4-sonnet-20250514"),
    "claude-3.7-sonnet-20250219": ("anthropic", "claude-3-7-sonnet-20250219"),
    "claude-3.5-sonnet":          ("anthropic", "claude-3-5-sonnet-20240620"),

    # Google AI (Gemini)
    "Gemini-2.5-pro": ("google", "gemini-2.5-pro"),
    "Gemini-1.5-pro": ("google", "gemini-1.5-pro"),

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

# === Student LLMs to be scored by the 3 judge LLMs ===
ALL_MODELS = [
    # DeepSeek Models (3)
    "deepseek-r1",
    "deepseek-r1-8b",
    "deepseek-r1-32b",

    # Meta/Llama Models (6)
    "llama-3.1-405b-instruct",
    "llama-3.1-70b-instruct",
    "llama-3.1-8b-instruct",
    "llama-3.3-70b-instruct",
    "llama4_maverick",
    "llama4_scout",

    # Microsoft Phi Models (4)
    "phi-3-mini-128k-instruct",
    "phi-3-medium-128k-instruct",
    "phi-4-mini-instruct",

    # Google Gemma Models (5)
    "gemma-2-9b-it",
    "gemma-2-27b-it",
    "gemma-2-2b-it",
    "gemma-3-1b-it",
    "gemma-3-27b-it",

    # Mistral Models (8)
    "mistral-nemo-12b-instruct",
    "mixtral-8x22b-instruct-v0.1",
    "mixtral-8x7b-instruct-v0.1",
    "codestral-22b-instruct-v0.1",
    "mamba-codestral-7b-v0.1",
    "mistral-small-3.1-24b-instruct-2503",
    "mistral-medium-3-instruct",

    # Qwen Models (2)
    # "qwen3-235b-a22b",  # Temporarily disabled - returns invalid response structure
    "qwq-32b",
    "qwen3-7b-instuct",
]

# -----------------------------
# 5) Generation helpers
# -----------------------------
SYSTEM_PREAMBLE = (
    "You are a PyChrono expert. Generate or fix a simulation script. "
    "Follow instructions carefully and return Python code inside triple backticks."
)

@retry_until_success
def call_provider(provider: str, model_id: str, prompt: str, max_tokens: int = 4096) -> str:
    """
    Normalized call for chat-style models across providers.
    Returns the generated text (not just code).
    Now with continuous retry until successful response.
    """
    if provider == "openai":
        if not OPENAI_API_KEY:
            raise RuntimeError("Missing OPENAI_API_KEY for model: " + model_id)
        client = get_openai()
        resp = client.chat.completions.create(
            model=model_id,
            messages=[{"role":"user","content": f"{SYSTEM_PREAMBLE}\n\n{prompt}"}],
            max_completion_tokens=max_tokens,
            temperature=0.6,
            top_p=0.9,
        )
        return resp.choices[0].message.content

    elif provider == "nvidia":
        if not NVIDIA_API_KEY:
            raise RuntimeError("Missing NVIDIA_API_KEY for model: " + model_id)
        client = get_nvidia_openai()
        # Use streaming to prevent 504 timeouts
        resp = client.chat.completions.create(
            model=model_id,
            messages=[{"role":"user","content": f"{SYSTEM_PREAMBLE}\n\n{prompt}"}],
            temperature=0.6,
            top_p=0.9,
            max_tokens=max_tokens,
            stream=True  # Enable streaming for NVIDIA
        )
        # Collect streamed response
        response_content = ""
        for chunk in resp:
            if chunk.choices[0].delta.content is not None:
                response_content += chunk.choices[0].delta.content
        return response_content

    else:
        raise ValueError(f"Unknown provider: {provider}")

def read_text(path: str) -> str:
    with open(path, "r", encoding="utf-8") as f:
        return f.read()

def write_text(path: str, text: str):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write(text)

# -----------------------------
# 6) Prompt builders (same as your originals)
# -----------------------------
def prompt_first(first_prompt: str) -> str:
    return f"""
You are a PyChrono expert tasked with generating a simulation script based on the following instructions. Make sure to:
1. Initialize the PyChrono environment and core components.
2. Add the required physical systems and objects as specified.
3. Set necessary default parameters such as positions, forces, and interactions.

Instructions:
\"\"\"\n{first_prompt}\n\"\"\"\n"""

def prompt_second_third(instr_prompt: str, base_code: str) -> str:
    return f"""
You are a PyChrono expert tasked with generating a simulation script based on the following instructions and a given PyChrono script, which may contain errors. Your task has two parts: identify the potential errors in the script and correct them if exist, also follow the instructions to modify the script to meet the requirements.

Here is the PyChrono code you need to modify:
{base_code}

Please modify the given code based on the following instructions:
\"\"\"\n{instr_prompt}\n\"\"\"\n
To complete the task, follow these steps:

Review the given PyChrono script and identify any errors, including syntax errors, logical errors, incorrect method names, and parameter issues.
Correct the identified errors in the script to ensure it runs correctly.
Modify the script based on the provided instructions to ensure it meets the specified requirements.

Provide the corrected and modified script below:
"""

# -----------------------------
# 7) Extract & clean helpers
# -----------------------------
def extract_python_blocks(text: str) -> str:
    """Return concatenated python blocks if present, else fall back to whole text."""
    blocks = re.findall(r"```python(.*?)```", text, flags=re.DOTALL|re.IGNORECASE)
    if blocks:
        return "\n\n".join([b.strip() for b in blocks])
    # Fallback: any fenced code
    blocks2 = re.findall(r"```(.*?)```", text, flags=re.DOTALL)
    if blocks2:
        return "\n\n".join([b.strip() for b in blocks2])
    return text.strip()

def strip_comments(py_code: str) -> str:
    code = re.sub(r'#.*', '', py_code)
    code = re.sub(r'("""[\s\S]*?"""|\'\'\'[\s\S]*?\'\'\')', '', code)
    return code.strip()

def extract_and_save(txt_path: str, py_out: str, py_out_clean: str, log_prefix: str):
    logging.basicConfig(filename=os.path.join(os.path.dirname(py_out), f"{log_prefix}_extract.log"),
                        level=logging.INFO,
                        format='%(asctime)s - %(levelname)s - %(message)s')
    try:
        text = read_text(txt_path)
        code = extract_python_blocks(text)
        write_text(py_out, code)
        cleaned = strip_comments(code)
        write_text(py_out_clean, cleaned)
        logging.info(f"Saved: {py_out}, {py_out_clean}")
    except Exception as e:
        logging.exception(f"Extraction failed: {e}")

# -----------------------------
# 8) LLM scoring (uses OpenAI by default)
# -----------------------------
from openai import OpenAI as _OpenAI  # use same package
from functools import wraps

def get_eval_client():
    if not OPENAI_API_KEY:
        raise RuntimeError("Evaluation requires OPENAI_API_KEY (used for scoring prompts).")
    return _OpenAI(api_key=OPENAI_API_KEY)

def retry_with_backoff(max_retries=3, initial_delay=1.0):
    """Decorator for retrying functions with exponential backoff."""
    def decorator(func):
        @wraps(func)
        def wrapper(*args, **kwargs):
            delay = initial_delay
            
            for attempt in range(max_retries):
                try:
                    return func(*args, **kwargs)
                except Exception as e:
                    if attempt < max_retries - 1:
                        print(f"[Retry {attempt + 1}/{max_retries}] {func.__name__} failed: {str(e)[:100]}. Retrying in {delay}s...")
                        sleep(delay)
                        delay *= 2  # Exponential backoff
                    else:
                        error_msg = f"ERROR: {func.__name__} failed after {max_retries} attempts: {str(e)[:200]}"
                        print(error_msg)
                        return error_msg
            
            # This should never be reached, but just in case
            return f"ERROR: {func.__name__} failed: Unknown error"
        return wrapper
    return decorator

_score_num_re = re.compile(r"\[\[(\d+(?:\.\d+)?)\]\]")

def extract_score(text: str) -> Optional[Union[float, str]]:
    """Extract numeric score or return error message if present."""
    if not text:
        return None
    # If it's an error message, return it as-is
    if text.startswith("ERROR:"):
        return text
    # Try to extract numeric score (now supports decimals)
    m = _score_num_re.search(text)
    return float(m.group(1)) if m else None

# === ADDED: multi-judge support (keeps the originals above untouched) ===
JUDGE_MODELS = ["gpt-4o-mini", "gpt-4.1-mini", "gpt-4.1-nano"]

@retry_with_backoff(max_retries=3, initial_delay=1.0)
def score_against_doc_with_model(code: str, api_doc: str, eval_model: str) -> str:
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

Reference the PyChrono API documentation provided here:
--- DOC START ---
{api_doc}
--- DOC END ---

After providing your explanation, output the final score using the following format: "[[x]]" where x is the score assigned to the assistant's answer.

Provide the evaluated score and a brief explanation of the deductions below:
"""
    client = get_eval_client()
    resp = client.chat.completions.create(
        model=eval_model,
        messages=[{"role":"user","content": prompt}],
        temperature=0.2, top_p=0.7, max_completion_tokens=4000
    )
    return resp.choices[0].message.content

@retry_with_backoff(max_retries=3, initial_delay=1.0)
def score_against_ref_with_model(code: str, ref: str, eval_model: str) -> str:
    prompt = f"""
You are a PyChrono expert tasked with evaluating a simulation script by comparing it against a reference script generated by experts.

Here is the PyChrono code you need to evaluate:
[The Start of Assistant's Answer]
{code}
[The End of Assistant's Answer]

Here is the expert-generated reference code:
[The Start of Reference Answer]
{ref}
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
    client = get_eval_client()
    resp = client.chat.completions.create(
        model=eval_model,
        messages=[{"role":"user","content": prompt}],
        temperature=0.2, top_p=0.7, max_completion_tokens=4000
    )
    return resp.choices[0].message.content

@retry_with_backoff(max_retries=3, initial_delay=1.0)
def score_against_ref_and_doc_with_model(code: str, ref: str, api_doc: str, eval_model: str) -> str:
    prompt = f"""
You are a PyChrono expert tasked with evaluating a simulation script by comparing it against a reference script generated by experts. Your evaluation should consider both the accuracy of the script compared to the reference and adherence to best practices as outlined in the PyChrono API documentation.

Here is the PyChrono code you need to evaluate:
[The Start of Assistant's Answer]
{code}
[The End of Assistant's Answer]

Here is the expert-generated reference code:
[The Start of Reference Answer]
{ref}
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

Reference the PyChrono API documentation provided here:
--- DOC START ---
{api_doc}
--- DOC END ---

Provide the evaluated score and a brief explanation of the deductions below:
"""
    client = get_eval_client()
    resp = client.chat.completions.create(
        model=eval_model,
        messages=[{"role":"user","content": prompt}],
        temperature=0.2, top_p=0.7, max_completion_tokens=4000
    )
    return resp.choices[0].message.content

def _run_triplet_for_judge(judge_model: str, code: str, ref: str, api_doc: str):
    """Run doc/ref/ref+doc scoring sequentially for a single judge model."""
    # The retry decorator already handles errors, but we keep try-catch for any unexpected issues
    try:
        s_doc_txt  = score_against_doc_with_model(code, api_doc, judge_model)
    except Exception as e:
        s_doc_txt = f"ERROR: Unexpected error in score_against_doc_with_model: {str(e)[:200]}"
    try:
        s_ref_txt  = score_against_ref_with_model(code, ref, judge_model)
    except Exception as e:
        s_ref_txt = f"ERROR: Unexpected error in score_against_ref_with_model: {str(e)[:200]}"
    try:
        s_both_txt = score_against_ref_and_doc_with_model(code, ref, api_doc, judge_model)
    except Exception as e:
        s_both_txt = f"ERROR: Unexpected error in score_against_ref_and_doc_with_model: {str(e)[:200]}"

    s_doc  = extract_score(s_doc_txt)
    s_ref  = extract_score(s_ref_txt)
    s_both = extract_score(s_both_txt)
    return judge_model, (s_doc_txt, s_ref_txt, s_both_txt), (s_doc, s_ref, s_both)

# -----------------------------
# 9) Systems list
# -----------------------------
SYSTEMS = ["art", "beam", "buckling", "cable", "car", "camera", "citybus", "curiosity", "feda", "gator", "gear", "gps_imu", "handler", "hmmwv", "kraz", "lidar", "m113", "man", "mass_spring_damper", "particles", "pendulum",
               "rigid_highway", "rigid_multipatches", "rotor", "scm", "scm_hill", "sedan", "sensros", "slider_crank", "tablecloth", "turtlebot", "uazbus", "veh_app","vehros","viper"]

# -----------------------------
# 10) Process single system helper
# -----------------------------
def process_single_system(
    model_name: str,
    provider: str,
    model_id: str,
    system: str,
    dataset_path: str = DATASET_PATH,
    output_path: str = OUTPUT_PATH,
    conv_path: str = CONV_PATH,
    output_sim_path: str = OUTPUT_SIM_PATH,
    output_score_path: str = OUTPUT_SCORE_PATH,
    skip_existing_simulations: bool = SKIP_EXISTING_SIMULATIONS,
    skip_existing_scoring: bool = SKIP_EXISTING_SCORING,
    judge_models: list = JUDGE_MODELS,
    max_scoring_workers: int = MAX_SCORING_WORKERS
):
    """Process one system: generate/load, extract, and score with parallel judges."""
    sys_in_dir = os.path.join(dataset_path, system)
    if not os.path.isdir(sys_in_dir):
        print(f"[warn] Missing dataset folder: {sys_in_dir} — skipping system")
        return None

    model_out_root = os.path.join(output_path, model_name)
    sys_out_dir = os.path.join(model_out_root, system)
    os.makedirs(sys_out_dir, exist_ok=True)

    # Check if simulation results already exist
    first_resp_path = os.path.join(sys_out_dir, "first_response.txt")
    second_resp_path = os.path.join(sys_out_dir, "second_response.txt")
    third_resp_path = os.path.join(sys_out_dir, "third_response.txt")

    if skip_existing_simulations and os.path.exists(first_resp_path) and os.path.exists(second_resp_path) and os.path.exists(third_resp_path):
        # Results exist, skip generation and load existing responses
        print(f"  [Skip] Results exist for {system}, loading existing responses...")
        r1 = read_text(first_resp_path)
        r2 = read_text(second_resp_path)
        r3 = read_text(third_resp_path)

        # Load prompts for conversation JSON (still needed for context)
        in1_txt = os.path.join(sys_in_dir, "input1.txt")
        in2_txt = os.path.join(sys_in_dir, "input2.txt")
        in3_txt = os.path.join(sys_in_dir, "input3.txt")
        in2_py = os.path.join(sys_in_dir, "pyinput2.py")
        in3_py = os.path.join(sys_in_dir, "pyinput3.py")

        if os.path.exists(in1_txt) and os.path.exists(in2_txt) and os.path.exists(in3_txt):
            p1 = prompt_first(read_text(in1_txt))
            p2 = prompt_second_third(read_text(in2_txt), read_text(in2_py))
            p3 = prompt_second_third(read_text(in3_txt), read_text(in3_py))
        else:
            p1 = p2 = p3 = "[Prompts not available]"
    else:
        # Generate new responses (or force regeneration if skip_existing_simulations is False)
        if not skip_existing_simulations and os.path.exists(first_resp_path):
            print(f"  [Regenerate] Force regenerating responses for {system}...", flush=True)
        else:
            print(f"  [Generate] Creating new responses for {system}...", flush=True)

        # ---- Generate 1st round
        in1_txt = os.path.join(sys_in_dir, "input1.txt")
        if not os.path.exists(in1_txt):
            print(f"[warn] {in1_txt} missing")
            return None
        print(f"    → {system}: Starting round 1/3...", flush=True)
        p1 = prompt_first(read_text(in1_txt))
        # Will retry until success with the @retry_until_success decorator
        r1 = call_provider(provider, model_id, p1, max_tokens=4096*4)
        write_text(first_resp_path, r1)

        # ---- Generate 2nd round
        in2_txt = os.path.join(sys_in_dir, "input2.txt")
        in2_py = os.path.join(sys_in_dir, "pyinput2.py")
        print(f"    → {system}: Starting round 2/3...", flush=True)
        p2 = prompt_second_third(read_text(in2_txt), read_text(in2_py))
        # Will retry until success with the @retry_until_success decorator
        r2 = call_provider(provider, model_id, p2, max_tokens=4096*4)
        write_text(second_resp_path, r2)

        # ---- Generate 3rd round
        in3_txt = os.path.join(sys_in_dir, "input3.txt")
        in3_py = os.path.join(sys_in_dir, "pyinput3.py")
        print(f"    → {system}: Starting round 3/3...", flush=True)
        p3 = prompt_second_third(read_text(in3_txt), read_text(in3_py))
        # Will retry until success with the @retry_until_success decorator
        r3 = call_provider(provider, model_id, p3, max_tokens=4096*4)
        write_text(third_resp_path, r3)

        # Save convo JSON
        conv_json = [{
            "instruction": p3, "input": "", "output": r3,
            "system": "You are a PyChrono expert tasked with generating a simulation script based on the following instructions.",
            "history": [[p1, r1],[p2, r2]]
        }]
        write_text(os.path.join(conv_path, f"{model_name}_{system}_conversation.json"), json.dumps(conv_json, indent=2))

    # ---- Extract + clean code for all 3 rounds
    extract_and_save(os.path.join(sys_out_dir, "first_response.txt"),
                     os.path.join(sys_out_dir, "first_response.py"),
                     os.path.join(sys_out_dir, "first_cleaned_response.py"),
                     log_prefix="first")
    extract_and_save(os.path.join(sys_out_dir, "second_response.txt"),
                     os.path.join(sys_out_dir, "second_response.py"),
                     os.path.join(sys_out_dir, "second_cleaned_response.py"),
                     log_prefix="second")
    extract_and_save(os.path.join(sys_out_dir, "third_response.txt"),
                     os.path.join(sys_out_dir, "third_response.py"),
                     os.path.join(sys_out_dir, "third_cleaned_response.py"),
                     log_prefix="third")

    # ---- Scoring vs reference & docs (uses OpenAI key)
    try:
        api_doc = read_text(os.path.join(os.path.dirname(DATASET_PATH), "api", "api.txt")) \
                    if os.path.exists(os.path.join(os.path.dirname(DATASET_PATH), "api", "api.txt")) else "N/A"
    except:
        api_doc = "N/A"

    truth1 = os.path.join(sys_in_dir, "truth1.py")
    truth2 = os.path.join(sys_in_dir, "truth2.py")
    truth3 = os.path.join(sys_in_dir, "truth3.py")

    # === Run MULTIPLE judges in parallel ===
    # Check if multi-judge scores should be skipped
    def check_multi_judge_scores_exist(sys_out_dir: str) -> bool:
        """Check if multi-judge score files exist for this system"""
        for round_name in ["first", "second", "third"]:
            for jm in judge_models:
                safe_jm = jm.replace("/", "_").replace(":", "_")
                files_to_check = [
                    f"{round_name}_score_document__{safe_jm}.txt",
                    f"{round_name}_score_reference__{safe_jm}.txt",
                    f"{round_name}_score_reference_document__{safe_jm}.txt"
                ]
                if not all(os.path.exists(os.path.join(sys_out_dir, f)) for f in files_to_check):
                    return False
        return True

    def _score_round_multi_judges(round_name: str, py_clean_name: str, truth_path: str):
        code = read_text(os.path.join(sys_out_dir, py_clean_name)) if os.path.exists(os.path.join(sys_out_dir, py_clean_name)) else ""
        ref = read_text(truth_path) if os.path.exists(truth_path) else ""
        results = {}
        # Use high parallelism for OpenAI scoring
        with ThreadPoolExecutor(max_workers=max_scoring_workers) as ex:
            futs = {ex.submit(_run_triplet_for_judge, jm, code, ref, api_doc): jm for jm in judge_models}
            for fut in as_completed(futs):
                jm, (s_doc_txt, s_ref_txt, s_both_txt), (s_doc, s_ref, s_both) = fut.result()
                # Save judge-specific raw outputs
                safe_jm = jm.replace("/", "_").replace(":", "_")
                write_text(os.path.join(sys_out_dir, f"{round_name}_score_document__{safe_jm}.txt"), s_doc_txt)
                write_text(os.path.join(sys_out_dir, f"{round_name}_score_reference__{safe_jm}.txt"), s_ref_txt)
                write_text(os.path.join(sys_out_dir, f"{round_name}_score_reference_document__{safe_jm}.txt"), s_both_txt)
                results[jm] = (s_doc, s_ref, s_both)
        return results

    # Score all three rounds with multi-judges in parallel
    if skip_existing_scoring and check_multi_judge_scores_exist(sys_out_dir):
        print(f"  [Skip] Multi-judge scores already exist for {system}, loading existing scores...")
        # Load existing multi-judge scores
        j1, j2, j3 = {}, {}, {}
        for jm in judge_models:
            safe_jm = jm.replace("/", "_").replace(":", "_")
            j1[jm] = (
                extract_score(read_text(os.path.join(sys_out_dir, f"first_score_document__{safe_jm}.txt"))),
                extract_score(read_text(os.path.join(sys_out_dir, f"first_score_reference__{safe_jm}.txt"))),
                extract_score(read_text(os.path.join(sys_out_dir, f"first_score_reference_document__{safe_jm}.txt")))
            )
            j2[jm] = (
                extract_score(read_text(os.path.join(sys_out_dir, f"second_score_document__{safe_jm}.txt"))),
                extract_score(read_text(os.path.join(sys_out_dir, f"second_score_reference__{safe_jm}.txt"))),
                extract_score(read_text(os.path.join(sys_out_dir, f"second_score_reference_document__{safe_jm}.txt")))
            )
            j3[jm] = (
                extract_score(read_text(os.path.join(sys_out_dir, f"third_score_document__{safe_jm}.txt"))),
                extract_score(read_text(os.path.join(sys_out_dir, f"third_score_reference__{safe_jm}.txt"))),
                extract_score(read_text(os.path.join(sys_out_dir, f"third_score_reference_document__{safe_jm}.txt")))
            )
    else:
        if not skip_existing_scoring and check_multi_judge_scores_exist(sys_out_dir):
            print(f"  [Regenerate] Force re-running multi-judge scoring for {system}...")
        else:
            print(f"  [Score] Running multi-judge scoring for {system}...")

        # Score all three rounds in parallel with high concurrency
        with ThreadPoolExecutor(max_workers=MAX_SCORING_WORKERS) as executor:
            future_j1 = executor.submit(_score_round_multi_judges, "first", "first_cleaned_response.py", truth1)
            future_j2 = executor.submit(_score_round_multi_judges, "second", "second_cleaned_response.py", truth2)
            future_j3 = executor.submit(_score_round_multi_judges, "third", "third_cleaned_response.py", truth3)

            j1 = future_j1.result()
            j2 = future_j2.result()
            j3 = future_j3.result()

    # Calculate average scores from all judges
    def calculate_average_scores(judge_results):
        """Calculate average scores from multiple judges, handling None and error values."""
        doc_scores = []
        ref_scores = []
        both_scores = []

        for judge_model, (s_doc, s_ref, s_both) in judge_results.items():
            # Only include numeric scores in average (skip None and ERROR strings)
            if isinstance(s_doc, (int, float)) and s_doc is not None:
                doc_scores.append(s_doc)
            if isinstance(s_ref, (int, float)) and s_ref is not None:
                ref_scores.append(s_ref)
            if isinstance(s_both, (int, float)) and s_both is not None:
                both_scores.append(s_both)

        # Calculate averages or return error message if no valid scores
        avg_doc = round(sum(doc_scores) / len(doc_scores), 2) if doc_scores else "ERROR: No valid scores from judges"
        avg_ref = round(sum(ref_scores) / len(ref_scores), 2) if ref_scores else "ERROR: No valid scores from judges"
        avg_both = round(sum(both_scores) / len(both_scores), 2) if both_scores else "ERROR: No valid scores from judges"

        return avg_doc, avg_ref, avg_both

    # Calculate averaged scores for each round
    avg1 = calculate_average_scores(j1)
    avg2 = calculate_average_scores(j2)
    avg3 = calculate_average_scores(j3)

    # Update the main CSV to use averaged scores
    csv_name = f"evaluation_scores_{model_name}_averaged.csv"
    csv_path_avg = os.path.join(sys_out_dir, csv_name)
    rows_avg = [["Model","System","Round","Avg Score Document","Avg Score Reference","Avg Score Ref+Doc","Num Judges"]]
    rows_avg += [
        [model_name, system, "first",  avg1[0], avg1[1], avg1[2], len([1 for jm in j1.values() if isinstance(jm[0], (int, float))])],
        [model_name, system, "second", avg2[0], avg2[1], avg2[2], len([1 for jm in j2.values() if isinstance(jm[0], (int, float))])],
        [model_name, system, "third",  avg3[0], avg3[1], avg3[2], len([1 for jm in j3.values() if isinstance(jm[0], (int, float))])],
    ]
    with open(csv_path_avg, "w", newline="", encoding="utf-8") as f:
        csv.writer(f).writerows(rows_avg)

    # Write a separate CSV per judge
    for jm in JUDGE_MODELS:
        safe_jm = jm.replace("/", "_").replace(":", "_")
        judge_csv = os.path.join(sys_out_dir, f"evaluation_scores_{model_name}__judged_by_{safe_jm}.csv")
        rows_j = [["Model","System","Judge","Round","Score Document","Score Reference","Score Ref+Doc"]]
        rows_j += [
            [model_name, system, jm, "first",  *(j1.get(jm, (None,None,None)))],
            [model_name, system, jm, "second", *(j2.get(jm, (None,None,None)))],
            [model_name, system, jm, "third",  *(j3.get(jm, (None,None,None)))],
        ]
        with open(judge_csv, "w", newline="", encoding="utf-8") as f:
            csv.writer(f).writerows(rows_j)

    # Print completion message
    print(f"  ✓ Completed {system} - all 3 rounds processed successfully", flush=True)
    return system

# -----------------------------
# 11) Main pipeline
# -----------------------------
def run_model(model_name: str):
    if model_name not in MODEL_REGISTRY:
        print(f"[!] Model not in registry: {model_name} — skipping")
        return

    provider, model_id = MODEL_REGISTRY[model_name]
    # Check key present for provider
    if provider == "openai" and not OPENAI_API_KEY:
        print(f"[!] Missing OpenAI key for {model_name}; skip."); return
    if provider == "nvidia" and not NVIDIA_API_KEY:
        print(f"[!] Missing NVIDIA key for {model_name}; skip."); return
    # Skip models from unsupported providers
    if provider not in ["openai", "nvidia"]:
        print(f"[!] Provider {provider} not supported (only OpenAI and NVIDIA); skipping {model_name}"); return

    # Determine worker count based on provider and model
    if provider == "nvidia":
        generation_workers = get_nvidia_workers(model_id)
    elif provider == "openai":
        generation_workers = 5  # OpenAI has higher rate limits
    else:
        generation_workers = MAX_GENERATION_WORKERS

    print(f"\n=== Running model: {model_name} ({provider}:{model_id}) ===")
    print(f"    Processing {len(SYSTEMS)} systems with {generation_workers} parallel workers")
    model_out_root = os.path.join(OUTPUT_PATH, model_name)
    os.makedirs(model_out_root, exist_ok=True)

    # Process systems in parallel with provider-specific worker count
    with concurrent.futures.ThreadPoolExecutor(max_workers=generation_workers) as executor:
        # Submit all system processing tasks with small delay for NVIDIA to prevent overload
        futures = {}
        for i, system in enumerate(SYSTEMS):
            future = executor.submit(
                process_single_system,
                model_name,
                provider,
                model_id,
                system
            )
            futures[future] = system

            # Add small delay between NVIDIA submissions to prevent gateway overload
            if provider == "nvidia" and i < len(SYSTEMS) - 1:
                sleep(0.5)  # 0.5 second delay between NVIDIA requests

        # Track progress with tqdm
        for future in tqdm(
            concurrent.futures.as_completed(futures),
            total=len(futures),
            desc=f"{model_name} systems"
        ):
            system = futures[future]
            try:
                future.result()
            except Exception as e:
                print(f"[ERROR] Failed to process system {system}: {e}")

    # All processing now happens via process_single_system
    print(f"=== Completed: {model_name} ===")

def merge_all_csvs():
    """Merge all averaged evaluation scores into a single CSV."""
    combined = []
    header_written = False
    for model_dir in os.listdir(OUTPUT_PATH):
        model_root = os.path.join(OUTPUT_PATH, model_dir)
        if not os.path.isdir(model_root): 
            continue
        for system in os.listdir(model_root):
            sys_dir = os.path.join(model_root, system)
            if not os.path.isdir(sys_dir):
                continue
            # Look specifically for averaged CSV files
            for fn in os.listdir(sys_dir):
                if fn.startswith("evaluation_scores_") and fn.endswith("_averaged.csv"):
                    with open(os.path.join(sys_dir, fn), "r", encoding="utf-8") as f:
                        rows = list(csv.reader(f))
                        if not header_written and rows:
                            combined.append(rows[0]); header_written = True
                        combined.extend(rows[1:])
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_csv = os.path.join(OUTPUT_PATH, f"combined_evaluation_scores_averaged_{ts}.csv")
    with open(out_csv, "w", newline="", encoding="utf-8") as f:
        csv.writer(f).writerows(combined)
    print(f"\nCombined averaged scores CSV written to: {out_csv}")

# -----------------------------
# 11) Run all models
# -----------------------------
if __name__ == "__main__":
    print("\nDatasets:", DATASET_PATH)
    print("Output  :", OUTPUT_PATH)
    print("Convos  :", CONV_PATH)

    for model in ALL_MODELS:
        run_model(model)

    merge_all_csvs()
    
    # Generate comprehensive rankings with all metrics
    print("\nGenerating comprehensive rankings...")
    try:
        ranker = RankingSystem()
        rankings = ranker.run()
    except Exception as e:
        print(f"Warning: Failed to generate comprehensive rankings: {e}")
    
    print("\nAll done.")
