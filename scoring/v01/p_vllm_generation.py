from openai import OpenAI
import os
import json
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
from tqdm import tqdm
import time
from typing import Tuple

# ---------------- Config ----------------
MAX_WORKERS = 8           # change to 10/20/etc
RETRIES     = 3
RETRY_SLEEP = 0             # seconds

client = OpenAI(base_url="http://localhost:8000/v1", api_key="0")

opensource_model_links = {
    "llama4-109b-lora1": "llama4-109b-lora1",
    "llama3.1-8b-lora1": "llama3.1-8b-lora1",
    "llama3.1-8b-f2": "llama3.1-8b-f2",
    "llama3.3-70b-lora1": "llama3.3-70b-lora1",
    "llama4-109b-lora1": "llama4-109b-lora1",
    "gpt-4o-mini-f1":  "ft:gpt-4o-mini-2024-07-18:personal::9xVAdwNY",
    "codestral-22b-f1":"codestral-22b-instruct-v0.1",
}

DATA_ROOT                  = Path("/lustre/fsw/coreai_nvfm_cupilot/jingquanw/SimBench")
dataset_path               = DATA_ROOT / "demo_data"
Output_path                = DATA_ROOT / "output_llms"
Output_conversation_path   = DATA_ROOT / "output_conversion"
test_model_list            = ["llama4-109b-lora1"]
Output_path.mkdir(exist_ok=True, parents=True)
Output_conversation_path.mkdir(exist_ok=True, parents=True)

# ---------------- Helpers ----------------
def read_script(file_path: Path) -> str:
    with open(file_path, "r", encoding="utf-8") as f:
        return f.read()

def safe_chat_create(messages, model, temperature=0.2, top_p=1.0, max_tokens=4096):
    """Simple retry wrapper around client.chat.completions.create"""
    last_err = None
    for _ in range(RETRIES):
        try:
            resp = client.chat.completions.create(
                model=model,
                messages=messages,
                temperature=temperature,
                top_p=top_p,
                max_tokens=max_tokens,
                stream=False
            )
            return resp.choices[0].message.content
        except Exception as e:
            last_err = e
            time.sleep(RETRY_SLEEP)
    return f"[ERROR AFTER {RETRIES} RETRIES] {last_err}"

def generate_first_code(first_prompt: str, model_link: str) -> Tuple[str, str]:
    prompt = f"""You are a PyChrono expert tasked with generating a simulation script based on the following instructions. Make sure to:
1. Initialize the PyChrono environment and core components.
2. Add the required physical systems and objects as specified.
3. Set necessary default parameters such as positions, forces, and interactions.

Instructions:
{first_prompt}
"""
    answer = safe_chat_create(
        [{"role": "user", "content": prompt}],
        model_link
    )
    return answer, prompt

def generate_second_third_code(prompt_text: str, code: str, model_link: str) -> Tuple[str, str]:
    prompt2 = f"""You are a PyChrono expert tasked with generating a simulation script based on the following instructions and a given PyChrono script (which may contain errors).

Tasks:
1. Identify and correct any errors (syntax, logic, wrong method names/parameters).
2. Modify the script to meet the new requirements.

Given PyChrono code:
{code}

Instructions to apply:
{prompt_text}

Output ONLY the corrected/modified script below:
"""
    answer = safe_chat_create(
        [{"role": "user", "content": prompt2}],
        model_link,
        max_tokens=4096
    )
    return answer, prompt2

def save_conversation_json(output_conversation_path: Path,
                           combined_prompt1, first_response,
                           combined_prompt2, second_response,
                           combined_prompt3, third_response):
    conversation_data = [{
        "instruction": combined_prompt3,
        "input": "",
        "output": third_response,
        "system": "You are a PyChrono expert tasked with generating a simulation script based on the following instructions.",
        "history": [
            [combined_prompt1, first_response],
            [combined_prompt2, second_response]
        ]
    }]
    output_conversation_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_conversation_path, 'w', encoding="utf-8") as f:
        json.dump(conversation_data, f, indent=4)

# ---------------- Worker ----------------
def process_system(system_folder: Path, model_name: str, model_link: str):
    """Run 3-round pipeline for one system. Returns system_folder.name for logging."""
    output_model_path   = Output_path / model_name
    output_system_path  = output_model_path / system_folder.name
    output_system_path.mkdir(exist_ok=True, parents=True)

    # Round 1
    input1_prompt = read_script(system_folder / "input1.txt")
    first_response, combined_prompt1 = generate_first_code(input1_prompt, model_link)
    (output_system_path / "first_response.txt").write_text(first_response, encoding="utf-8")

    # Round 2
    input2_prompt = read_script(system_folder / "input2.txt")
    input2_code   = read_script(system_folder / "pyinput2.py")
    second_response, combined_prompt2 = generate_second_third_code(input2_prompt, input2_code, model_link)
    (output_system_path / "second_response.txt").write_text(second_response, encoding="utf-8")

    # Round 3
    input3_prompt = read_script(system_folder / "input3.txt")
    input3_code   = read_script(system_folder / "pyinput3.py")
    third_response, combined_prompt3 = generate_second_third_code(input3_prompt, input3_code, model_link)
    (output_system_path / "third_response.txt").write_text(third_response, encoding="utf-8")

    # Save conversation JSON
    conv_path = Output_conversation_path / f"{model_name}_{system_folder.name}_conversation.json"
    save_conversation_json(conv_path,
                           combined_prompt1, first_response,
                           combined_prompt2, second_response,
                           combined_prompt3, third_response)

    return system_folder.name

# ---------------- Main ----------------
if __name__ == "__main__":
    system_dirs = [p for p in dataset_path.iterdir() if p.is_dir()]

    for test_model in test_model_list:
        print("Entering model:", test_model)
        test_model_link = opensource_model_links[test_model]

        # Parallel over systems
        with ThreadPoolExecutor(max_workers=MAX_WORKERS) as ex:
            futures = {
                ex.submit(process_system, sys_dir, test_model, test_model_link): sys_dir
                for sys_dir in system_dirs
            }
            for fut in tqdm(as_completed(futures), total=len(futures)):
                sys_dir = futures[fut]
                try:
                    done_name = fut.result()
                    # Uncomment for verbose per-system logging:
                    # print(f"[OK] {done_name}")
                except Exception as e:
                    print(f"[FAIL] {sys_dir.name}: {e}")

    print("finished")
