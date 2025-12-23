from openai import OpenAI
import os
import json
import time
import concurrent.futures
from tqdm import tqdm

# -----------------------------------------------------------------------------
# Auto-detect project root based on script location
# -----------------------------------------------------------------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
# Script is at: <PROJECT_ROOT>/scoring/v01/pe_gpt_generate_simulation.py, so go up 2 levels
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))

# -----------------------------------------------------------------------------
# Configuration
# -----------------------------------------------------------------------------
CONTEXT_FILE = os.path.join(PROJECT_ROOT, "api", "api.txt")

try:
    with open(CONTEXT_FILE, "r", encoding="utf-8") as f:
        EXTRA_CONTEXT = f.read().strip()
except FileNotFoundError:
    EXTRA_CONTEXT = ""
    print(f"[!] Warning: context file not found at {CONTEXT_FILE!r}. Continuing without extra context.")

OPENAI_API_KEY  = os.getenv("OPENAI_API_KEY")
NVIDIA_API_KEY  = os.getenv("NVIDIA_API_KEY")

if not (OPENAI_API_KEY or NVIDIA_API_KEY):
    raise RuntimeError("Please set at least one of OPENAI_API_KEY or NVIDIA_API_KEY in your environment.")

# Reasonable defaults
TEMPERATURE = 0.6
TOP_P       = 0.9
MAX_TOKENS  = 4096 * 4
REQUEST_TIMEOUT = 120  # seconds
RETRIES = 3
BACKOFF_SECS = 2.0

# -----------------------------------------------------------------------------
# Helper utilities
# -----------------------------------------------------------------------------
def read_script(path: str) -> str:
    """Return the full UTF-8 text of *path*."""
    with open(path, "r", encoding="utf-8") as fp:
        return fp.read()

# -----------------------------------------------------------------------------
# Prompt builders (PE stays the same as yours)
# -----------------------------------------------------------------------------
def _make_first_prompt(user_instruction: str) -> str:
    """Compose the Round-1 prompt, injecting extra PE context."""
    return (
        "Here is the API doc for PyChrono."
        f"{EXTRA_CONTEXT}\n\n"
        "You are a PyChrono expert tasked with generating a simulation script based on the following instructions. Make sure to:\n"
        "1. Initialize the PyChrono environment and core components.\n"
        "2. Add the required physical systems and objects as specified.\n"
        "3. Set necessary default parameters such as positions, forces, and interactions.\n\n"
        "Instructions:\n"
        "\"\"\"\n"
        f"{user_instruction}\n"
        "\"\"\"\n"
    )

def _make_second_third_prompt(instructions: str, code: str) -> str:
    """Compose the Round-2/3 prompt, including code to patch and PE context."""
    return (
        "Here is the API doc for PyChrono."
        f"{EXTRA_CONTEXT}\n\n"
        "You are a PyChrono expert tasked with generating a simulation script based on the following instructions and a given PyChrono script, which may contain errors. Your task has two parts: identify the potential errors in the script and correct them if they exist, then follow the instructions to modify the script to meet the requirements.\n\n"
        "Here is the PyChrono code you need to modify:\n"
        "```python\n"
        f"{code}\n"
        "```\n\n"
        "Instructions:\n"
        "\"\"\"\n"
        f"{instructions}\n"
        "\"\"\"\n\n"
        "To complete the task, follow these steps:\n"
        "1. Review the given PyChrono script and identify any errors, including syntax errors, logical errors, incorrect method names, and parameter issues.\n"
        "2. Correct the identified errors in the script to ensure it runs correctly.\n"
        "3. Modify the script based on the provided instructions to ensure it meets the specified requirements.\n\n"
        "Provide the corrected and modified script below:"
    )

# -----------------------------------------------------------------------------
# Provider routing
# -----------------------------------------------------------------------------
# Define where each model lives and the id to send to the provider.
# - provider: "nvidia" (NV NIM OpenAI-compatible endpoint) or "openai" (OpenAI public API)
# - model_id: string to pass to client.chat.completions.create(model=...)
MODEL_REGISTRY = {
    # ---------- OpenAI examples ----------
    "gpt-4.1":               {"provider": "openai", "model_id": "gpt-4.1"},
    "gpt-4.1-mini":          {"provider": "openai", "model_id": "gpt-4.1-mini"},
    "o4-mini":               {"provider": "openai", "model_id": "o4-mini"},
    "o3":                    {"provider": "openai", "model_id": "o3"},
    "gpt-4o":                {"provider": "openai", "model_id": "gpt-4o"},
    "gpt-4o-mini":           {"provider": "openai", "model_id": "gpt-4o-mini"},

    # ---------- NVIDIA NIM examples (same as you had) ----------
    "gemma-2-9b-it":         {"provider": "nvidia", "model_id": "google/gemma-2-9b-it"},
    "gemma-2-27b-it":        {"provider": "nvidia", "model_id": "google/gemma-2-27b-it"},
    "gemma-2-2b-it":         {"provider": "nvidia", "model_id": "google/gemma-2-2b-it"},
    "llama-3.1-405b-instruct":{"provider": "nvidia", "model_id": "meta/llama-3.1-405b-instruct"},
    "llama-3.1-70b-instruct":{"provider": "nvidia", "model_id": "meta/llama-3.1-70b-instruct"},
    "llama-3.1-8b-instruct": {"provider": "nvidia", "model_id": "meta/llama-3.1-8b-instruct"},
    "phi-3-mini-128k-instruct":{"provider": "nvidia", "model_id": "microsoft/phi-3-mini-128k-instruct"},
    "phi-3-medium-128k-instruct":{"provider": "nvidia", "model_id": "microsoft/Phi-3-medium-128k-instruct"},
    "nemotron-4-340b-instruct":{"provider": "nvidia", "model_id": "nvidia/nemotron-4-340b-instruct"},
    "mistral-nemo-12b-instruct":{"provider": "nvidia", "model_id": "nv-mistralai/mistral-nemo-12b-instruct"},
    "mixtral-8x22b-instruct-v0.1":{"provider": "nvidia", "model_id": "mistralai/mixtral-8x22b-instruct-v0.1"},
    "codestral-22b-instruct-v0.1":{"provider": "nvidia", "model_id": "mistralai/codestral-22b-instruct-v0.1"},
    "mixtral-8x7b-instruct-v0.1":{"provider": "nvidia", "model_id": "mistralai/mixtral-8x7b-instruct-v0.1"},
    "mistral-large-latest":  {"provider": "nvidia", "model_id": "mistralai/mistral-large"},
    "mistral-small-3.1-24b-instruct-2503":{"provider": "nvidia","model_id":"mistralai/mistral-small-3.1-24b-instruct-2503"},
    "mistral-medium-3-instruct":{"provider": "nvidia","model_id":"mistralai/mistral-medium-3-instruct"},
    "mamba-codestral-7b-v0.1":{"provider": "nvidia", "model_id": "mistralai/mamba-codestral-7b-v0.1"},
    "llama4_maverick":       {"provider": "nvidia", "model_id": "nvdev/meta/llama-4-maverick-17b-128e-instruct"},
    "llama4_scout":          {"provider": "nvidia", "model_id": "nvdev/meta/llama-4-scout-17b-16e-instruct"},
    "llama-3.3-70b-instruct":{"provider": "nvidia", "model_id": "nvdev/meta/llama-3.3-70b-instruct"},
    "deepseek-r1-8b":        {"provider": "nvidia", "model_id": "deepseek-ai/deepseek-r1-distill-llama-8b"},
    "deepseek-r1-32b":       {"provider": "nvidia", "model_id": "deepseek-ai/deepseek-r1-distill-qwen-32b"},
    "deepseek-r1":           {"provider": "nvidia", "model_id": "deepseek-ai/deepseek-r1-0528"},
    "gemma-3-27b-it":        {"provider": "nvidia", "model_id": "nvdev/google/gemma-3-27b-it"},
    "gemma-3-1b-it":         {"provider": "nvidia", "model_id": "google/gemma-3-1b-it"},
    "qwen3-235b-a22b":       {"provider": "nvidia", "model_id": "qwen/qwen3-235b-a22b"},
    "qwq-32b":               {"provider": "nvidia", "model_id": "qwen/qwq-32b"},
    "qwen3-7b-instruct":     {"provider": "nvidia", "model_id": "qwen/qwen2-7b-instruct"},
    "phi-4-mini-instruct":   {"provider": "nvidia", "model_id": "microsoft/phi-4-mini-instruct"},
}

def _make_client(provider: str) -> OpenAI:
    """Return an OpenAI SDK client configured for the given provider."""
    if provider == "openai":
        if not OPENAI_API_KEY:
            raise RuntimeError("OPENAI_API_KEY not set for OpenAI provider.")
        # Default base_url is OpenAI's; no override needed.
        return OpenAI(api_key=OPENAI_API_KEY, timeout=REQUEST_TIMEOUT)
    elif provider == "nvidia":
        if not NVIDIA_API_KEY:
            raise RuntimeError("NVIDIA_API_KEY not set for NVIDIA provider.")
        # NV NIM OpenAI-compatible endpoint
        return OpenAI(base_url="https://integrate.api.nvidia.com/v1",
                      api_key=NVIDIA_API_KEY,
                      timeout=REQUEST_TIMEOUT)
    else:
        raise ValueError(f"Unknown provider: {provider}")

# -----------------------------------------------------------------------------
# Chat-completion wrapper with simple retry/backoff
# -----------------------------------------------------------------------------
def _chat_completion(model_name: str, prompt: str):
    if model_name not in MODEL_REGISTRY:
        raise KeyError(f"Model '{model_name}' not found in MODEL_REGISTRY.")
    provider   = MODEL_REGISTRY[model_name]["provider"]
    model_id   = MODEL_REGISTRY[model_name]["model_id"]

    client = _make_client(provider)

    last_err = None
    for attempt in range(1, RETRIES + 1):
        try:
            resp = client.chat.completions.create(
                model=model_id,
                messages=[{"role": "user", "content": prompt}],
                temperature=TEMPERATURE,
                top_p=TOP_P,
                max_tokens=MAX_TOKENS,
                stream=False,
            )
            return resp.choices[0].message.content, prompt
        except Exception as exc:
            last_err = exc
            if attempt < RETRIES:
                time.sleep(BACKOFF_SECS * attempt)
            else:
                print("[!] Completion error:", exc)
    # Return the exception text as output (keeps your pipeline stable)
    return f"[Completion error] {last_err}", prompt

def generate_first_code(instruction: str, model_name: str):
    return _chat_completion(model_name, _make_first_prompt(instruction))

def generate_second_third_code(instr: str, code: str, model_name: str):
    return _chat_completion(model_name, _make_second_third_prompt(instr, code))

# -----------------------------------------------------------------------------
# Conversation persistence
# -----------------------------------------------------------------------------
def save_conversation_json(path_out: str, p1: str, r1: str, p2: str, r2: str, p3: str, r3: str):
    os.makedirs(os.path.dirname(path_out), exist_ok=True)
    data = [{
        "instruction": p3,
        "input": "",
        "output": r3,
        "system": "You are a PyChrono expert tasked with generating a simulation script based on the following instructions.",
        "history": [[p1, r1], [p2, r2]],
    }]
    with open(path_out, "w", encoding="utf-8") as fp:
        json.dump(data, fp, indent=4)

# -----------------------------------------------------------------------------
# Per-system workflow (Rounds 1-3)
# -----------------------------------------------------------------------------
def process_system(test_model: str, system_folder: str, dataset_root: str, out_model_dir: str, out_conv_dir: str):
    folder_in  = os.path.join(dataset_root, system_folder)
    folder_out = os.path.join(out_model_dir, system_folder)
    os.makedirs(folder_out, exist_ok=True)

    # Round 1
    r1_text, p1 = generate_first_code(read_script(os.path.join(folder_in, "input1.txt")), test_model)
    with open(os.path.join(folder_out, "first_response.txt"), "w", encoding="utf-8") as fp:
        fp.write(r1_text)

    # Round 2
    r2_text, p2 = generate_second_third_code(
        read_script(os.path.join(folder_in, "input2.txt")),
        read_script(os.path.join(folder_in, "pyinput2.py")),
        test_model,
    )
    with open(os.path.join(folder_out, "second_response.txt"), "w", encoding="utf-8") as fp:
        fp.write(r2_text)

    # Round 3
    r3_text, p3 = generate_second_third_code(
        read_script(os.path.join(folder_in, "input3.txt")),
        read_script(os.path.join(folder_in, "pyinput3.py")),
        test_model,
    )
    with open(os.path.join(folder_out, "third_response.txt"), "w", encoding="utf-8") as fp:
        fp.write(r3_text)

    # Save conversation JSON
    save_conversation_json(
        os.path.join(out_conv_dir, f"{test_model}_{system_folder}_conversation.json"),
        p1, r1_text, p2, r2_text, p3, r3_text,
    )

# -----------------------------------------------------------------------------
# Main orchestration
# -----------------------------------------------------------------------------
def main():
    # Systems list (kept as-is; note you have 'sensros' and 'vehros' spelled that way in your data)
    systems_all = [
        "art", "beam", "buckling", "cable", "car", "camera", "citybus", "curiosity", "feda", "gator", "gear",
        "gps_imu", "handler", "hmmwv", "kraz", "lidar", "m113", "man", "mass_spring_damper", "particles",
        "pendulum", "rigid_highway", "rigid_multipatches", "rotor", "scm", "scm_hill", "sedan", "sensros",
        "slider_crank", "tablecloth", "turtlebot", "uazbus", "veh_app", "vehros", "viper",
    ]

    # Auto-detected paths based on project root
    dataset_root = os.path.join(PROJECT_ROOT, "demo_data")
    output_root = os.path.join(PROJECT_ROOT, "output_llms")
    conv_root = os.path.join(PROJECT_ROOT, "output_conversion")

    # Mix OpenAI + NVIDIA models freely here
    test_models = [
        # OpenAI
        #"gpt-4.1-mini", "o4-mini", 
        "gpt-4o-mini",
        # NVIDIA NIM
        #"llama4_maverick", "llama4_scout",
        #"gemma-2-9b-it", "llama-3.1-8b-instruct",
    ]

    MAX_WORKERS = 4

    systems = [d for d in os.listdir(dataset_root) if d in systems_all]

    for model_name in test_models:
        out_model_dir = os.path.join(output_root, f"pe_{model_name}")
        os.makedirs(out_model_dir, exist_ok=True)

        with concurrent.futures.ThreadPoolExecutor(max_workers=MAX_WORKERS) as exe:
            futs = {
                exe.submit(
                    process_system,
                    model_name,
                    sys_folder,
                    dataset_root,
                    out_model_dir,
                    conv_root,
                ): sys_folder
                for sys_folder in systems
            }

            for fut in tqdm(concurrent.futures.as_completed(futs), total=len(futs), desc=f"⟳ {model_name}"):
                folder = futs[fut]
                try:
                    fut.result()
                except Exception as e:
                    print(f"[!] Error in {folder}: {e}")

    print("All done.")

if __name__ == "__main__":
    main()
