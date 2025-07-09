from openai import OpenAI
import os
import json
import concurrent.futures
from tqdm import tqdm

# -----------------------------------------------------------------------------
# Configuration
# -----------------------------------------------------------------------------
CONTEXT_FILE = r"C:\Users\jingquanw\SimBench\api\api.txt"

try:
    with open(CONTEXT_FILE, "r", encoding="utf-8") as f:
        EXTRA_CONTEXT = f.read().strip()
except FileNotFoundError:
    EXTRA_CONTEXT = ""
    print(f"[!] Warning: context file not found at {CONTEXT_FILE!r}. Continuing without extra context.")

nvidia_api_key = os.getenv("NVIDIA_API_KEY")
if not nvidia_api_key:
    raise RuntimeError("Please set the NVIDIA_API_KEY environment variable!")

# -----------------------------------------------------------------------------
# Helper utilities
# -----------------------------------------------------------------------------

def read_script(path: str) -> str:
    """Return the full UTF-8 text of *path*."""
    with open(path, "r", encoding="utf-8") as fp:
        return fp.read()

# -----------------------------------------------------------------------------
# Prompt builders
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
# Chat-completion wrapper
# -----------------------------------------------------------------------------

def _chat_completion(model_link: str, prompt: str):
    try:
        client = OpenAI(base_url="https://integrate.api.nvidia.com/v1", api_key=nvidia_api_key)
        resp = client.chat.completions.create(
            model=model_link,
            messages=[{"role": "user", "content": prompt}],
            temperature=0.6,
            top_p=0.9,
            max_tokens=4096 * 4,
            stream=False,
        )
        return resp.choices[0].message.content, prompt
    except Exception as exc:
        print("[!] Completion error:", exc)
        return str(exc), prompt


def generate_first_code(instruction: str, model_link: str):
    return _chat_completion(model_link, _make_first_prompt(instruction))


def generate_second_third_code(instr: str, code: str, model_link: str):
    return _chat_completion(model_link, _make_second_third_prompt(instr, code))

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

def process_system(test_model: str, model_link: str, system_folder: str, dataset_root: str, out_model_dir: str, out_conv_dir: str):
    folder_in = os.path.join(dataset_root, system_folder)
    folder_out = os.path.join(out_model_dir, system_folder)
    os.makedirs(folder_out, exist_ok=True)

    # Round 1
    r1_text, p1 = generate_first_code(read_script(os.path.join(folder_in, "input1.txt")), model_link)
    with open(os.path.join(folder_out, "first_response.txt"), "w", encoding="utf-8") as fp:
        fp.write(r1_text)

    # Round 2
    r2_text, p2 = generate_second_third_code(
        read_script(os.path.join(folder_in, "input2.txt")),
        read_script(os.path.join(folder_in, "pyinput2.py")),
        model_link,
    )
    with open(os.path.join(folder_out, "second_response.txt"), "w", encoding="utf-8") as fp:
        fp.write(r2_text)

    # Round 3
    r3_text, p3 = generate_second_third_code(
        read_script(os.path.join(folder_in, "input3.txt")),
        read_script(os.path.join(folder_in, "pyinput3.py")),
        model_link,
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
    opensource_model_links = {
        "gemma-2-9b-it": "google/gemma-2-9b-it",
        "gemma-2-27b-it": "google/gemma-2-27b-it",
        "gemma-2-2b-it": "google/gemma-2-2b-it",
        "llama-3.1-405b-instruct": "meta/llama-3.1-405b-instruct",
        "llama-3.1-70b-instruct": "meta/llama-3.1-70b-instruct",
        "llama-3.1-8b-instruct": "meta/llama-3.1-8b-instruct",
        "phi-3-mini-128k-instruct": "microsoft/phi-3-mini-128k-instruct",
        "phi-3-medium-128k-instruct": "microsoft/Phi-3-medium-128k-instruct",
        "nemotron-4-340b-instruct": "nvidia/nemotron-4-340b-instruct",
        "mistral-nemo-12b-instruct": "nv-mistralai/mistral-nemo-12b-instruct",
        "mixtral-8x22b-instruct-v0.1": "mistralai/mixtral-8x22b-instruct-v0.1",
        "codestral-22b-instruct-v0.1": "mistralai/codestral-22b-instruct-v0.1",
        "mixtral-8x7b-instruct-v0.1": "mistralai/mixtral-8x7b-instruct-v0.1",
        "mistral-large-latest": "mistralai/mistral-large",
        "mistral-small-3.1-24b-instruct-2503": "mistralai/mistral-small-3.1-24b-instruct-2503",
        "mistral-medium-3-instruct": "mistralai/mistral-medium-3-instruct",
        "mamba-codestral-7b-v0.1": "mistralai/mamba-codestral-7b-v0.1",
        "llama4_maverick": "nvdev/meta/llama-4-maverick-17b-128e-instruct",
        "llama4_scout": "nvdev/meta/llama-4-scout-17b-16e-instruct",
        "llama-3.3-70b-instruct": "nvdev/meta/llama-3.3-70b-instruct",
        "deepseek-r1-8b": "deepseek-ai/deepseek-r1-distill-llama-8b",
        "deepseek-r1-32b": "deepseek-ai/deepseek-r1-distill-qwen-32b",
        "deepseek-r1": "deepseek-ai/deepseek-r1-0528",
        "gemma-3-27b-it": "nvdev/google/gemma-3-27b-it",
        "gemma-3-1b-it": "google/gemma-3-1b-it",
        "qwen3-235b-a22b": "qwen/qwen3-235b-a22b",
        "qwq-32b": "qwen/qwq-32b",
        "qwen3-7b-instuct": "qwen/qwen2-7b-instruct",
        "phi-4-mini-instruct": "microsoft/phi-4-mini-instruct"
    }

    systems_all = [
        "art", "beam", "buckling", "cable", "car", "camera", "citybus", "curiosity", "feda", "gator", "gear",
        "gps_imu", "handler", "hmmwv", "kraz", "lidar", "m113", "man", "mass_spring_damper", "particles",
        "pendulum", "rigid_highway", "rigid_multipatches", "rotor", "scm", "scm_hill", "sedan", "sensros",
        "slider_crank", "tablecloth", "turtlebot", "uazbus", "veh_app", "vehros", "viper",
    ]

    dataset_root = r"C:\Users\jingquanw\SimBench\demo_data"
    output_root = r"C:\Users\jingquanw\SimBench\output_llms"
    conv_root = r"C:\Users\jingquanw\SimBench\output_conversion"

    test_models = ["gemma-2-9b-it","gemma-2-27b-it","gemma-2-2b-it","llama-3.1-405b-instruct"
        ,"llama4_scout","llama4_maverick"]
    test_models =[
        #"gemma-2-2b-it", "gemma-2-9b-it", "gemma-2-27b-it", "llama-3.1-405b-instruct", "llama-3.1-70b-instruct",
     #"llama-3.1-8b-instruct", "phi-3-mini-128k-instruct", "phi-3-medium-128k-instruct",
     #"nemotron-4-340b-instruct", "mistral-nemo-12b-instruct", "mixtral-8x22b-instruct-v0.1",
     #"codestral-22b-instruct-v0.1",
     #"mixtral-8x7b-instruct-v0.1", "mistral-large-latest", "mamba-codestral-7b-v0.1",
     #"gpt-4o", "gpt-4o-mini", "claude-3-5-sonnet", "Gemini-1.5-pro",
     "llama4_maverick", "llama4_scout",
          #        "llama-3.3-70b-instruct",
         #         "deepseek-r1-8b","deepseek-r1-32b",
        #"deepseek-r1",
        #"gemma-3-1b-it", "qwen3-235b-a22b",
                  #"claude-3-7-sonnet-20250219","claude-4-sonnet-20250514", "Gemini-2.5-pro", "Gemini-1.5-pro",
                  #"gpt-4.1-mini", "gpt-4.1-nano","gpt-4.1", "o4-mini","o3"
                  ]

    MAX_WORKERS = 4

    for model_name in test_models:
        model_link = opensource_model_links[model_name]
        out_model_dir = os.path.join(output_root, f"pe_{model_name}")
        os.makedirs(out_model_dir, exist_ok=True)

        systems = [d for d in os.listdir(dataset_root) if d in systems_all]

        with concurrent.futures.ThreadPoolExecutor(max_workers=MAX_WORKERS) as exe:
            futs = {
                exe.submit(
                    process_system,
                    model_name,
                    model_link,
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
