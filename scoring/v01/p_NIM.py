from openai import OpenAI
import os
import json
import concurrent.futures
from tqdm import tqdm

nvidia_api_key = os.getenv("NVIDIA_API_KEY")
if not nvidia_api_key:
    raise RuntimeError("Please set the NVIDIA_API_KEY environment variable!")

def read_script(file_path):
    with open(file_path, "r", encoding="utf-8") as f:
        return f.read()

# (keep your existing generate_first_code, generate_second_third_code, save_conversation_json here)

def generate_first_code(first_prompt, model_link):
    # because some of the models like gemma-2 do not have a system role, so we add the system role to the user role prompt
    prompt = f"""
    You are a PyChrono expert tasked with generating a simulation script based on the following instructions. Make sure to:
    1. Initialize the PyChrono environment and core components.
    2. Add the required physical systems and objects as specified.
    3. Set necessary default parameters such as positions, forces, and interactions.

    Instructions:
    “”"
    {first_prompt}
    “”"
    """
    try:
        global nvidia_api_key
        client = OpenAI(
            base_url="https://integrate.api.nvidia.com/v1",
            api_key=nvidia_api_key
        )
        completion = client.chat.completions.create(
            model=model_link,
            messages=[
                {"role": "user", "content": prompt}
            ],
            temperature=0.6,
            top_p=0.9,
            max_tokens=4096*4,
            stream=False
        )
        return completion.choices[0].message.content, prompt
    except Exception as e:
        print('error1:', e)
        return str(e), str(e)


def generate_second_third_code(prompt, code, model_link):
    prompt = f"""

    You are a PyChrono expert tasked with generating a simulation script based on the following instructions and a given PyChrono script, which may contain errors. Your task has two parts: identify the potential errors in the script and correct them if exist, also follow the instructions to modify the script to meet the requirements.

Here is the PyChrono code you need to modify:
{code}


Please modify the given code based on the following instructions:
“”"
{prompt}
“”"

To complete the task, follow these steps:

Review the given PyChrono script and identify any errors, including syntax errors, logical errors, incorrect method names, and parameter issues.
Correct the identified errors in the script to ensure it runs correctly.
Modify the script based on the provided instructions to ensure it meets the specified requirements.

Provide the corrected and modified script below:
    """
    try:
        global nvidia_api_key
        client = OpenAI(
            base_url="https://integrate.api.nvidia.com/v1",
            api_key=nvidia_api_key
        )
        completion = client.chat.completions.create(
            model=model_link,
            messages=[
                {"role": "user", "content": prompt}
            ],
            temperature=0.6,
            top_p=0.9,
            max_tokens=4096*4,
            stream=False
        )
        return completion.choices[0].message.content, prompt
    except Exception as e:
        print('error2:', e)
        return str(e), str(e)


def save_conversation_json(output_conversation_path, combined_prompt1, first_response, combined_prompt2,
                           second_response, combined_prompt3, third_response):
    # Prepare the conversation data
    # Ensure the directory exists
    directory = os.path.dirname(output_conversation_path)
    if not os.path.exists(directory):
        os.makedirs(directory)
    conversation_data = [
        {
            "instruction": combined_prompt3,
            "input": "",
            "output": third_response,
            "system": "You are a PyChrono expert tasked with generating a simulation script based on the following instructions.",
            "history": [
                [combined_prompt1, first_response],
                [combined_prompt2, second_response]
            ]
        }
    ]

    # Save the conversation data to a JSON file
    with open(output_conversation_path, 'w') as json_file:
        json.dump(conversation_data, json_file, indent=4)


def process_system(test_model, test_model_link, system_folder, dataset_path, output_model_path, output_conv_path):
    """Do rounds 1–3 for a single system_folder."""
    system_folder_path = os.path.join(dataset_path, system_folder)
    output_system_path = os.path.join(output_model_path, system_folder)
    os.makedirs(output_system_path, exist_ok=True)

    # --- Round 1 ---
    input1 = read_script(os.path.join(system_folder_path, "input1.txt"))
    first_resp, cp1 = generate_first_code(input1, test_model_link)
    with open(os.path.join(output_system_path, "first_response.txt"), "w", encoding="utf-8") as f:
        f.write(first_resp)

    # --- Round 2 ---
    input2 = read_script(os.path.join(system_folder_path, "input2.txt"))
    code2  = read_script(os.path.join(system_folder_path, "pyinput2.py"))
    second_resp, cp2 = generate_second_third_code(input2, code2, test_model_link)
    with open(os.path.join(output_system_path, "second_response.txt"), "w", encoding="utf-8") as f:
        f.write(second_resp)

    # --- Round 3 ---
    input3 = read_script(os.path.join(system_folder_path, "input3.txt"))
    code3  = read_script(os.path.join(system_folder_path, "pyinput3.py"))
    third_resp, cp3 = generate_second_third_code(input3, code3, test_model_link)
    with open(os.path.join(output_system_path, "third_response.txt"), "w", encoding="utf-8") as f:
        f.write(third_resp)

    # --- Save JSON conversation ---
    conv_file = os.path.join(
        output_conv_path,
        f"{test_model}_{system_folder}_conversation.json"
    )
    save_conversation_json(conv_file,
                           cp1, first_resp,
                           cp2, second_resp,
                           cp3, third_resp)

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
        "mistral-large-latest":  "mistralai/mistral-large",
        "mistral-small-3.1-24b-instruct-2503":"mistralai/mistral-small-3.1-24b-instruct-2503",
        "mistral-medium-3-instruct":"mistralai/mistral-medium-3-instruct",
        "mamba-codestral-7b-v0.1": "mistralai/mamba-codestral-7b-v0.1",
        "llama4_maverick": "nvdev/meta/llama-4-maverick-17b-128e-instruct",
        "llama4_scout": "nvdev/meta/llama-4-scout-17b-16e-instruct",
        "llama-3.3-70b-instruct": "nvdev/meta/llama-3.3-70b-instruct",
        "deepseek-r1-8b":"deepseek-ai/deepseek-r1-distill-llama-8b",
        "deepseek-r1-32b":"deepseek-ai/deepseek-r1-distill-qwen-32b",
        "deepseek-r1":"deepseek-ai/deepseek-r1-0528",
        "gemma-3-27b-it":"nvdev/google/gemma-3-27b-it",
        "gemma-3-1b-it":"google/gemma-3-1b-it",
        "qwen3-235b-a22b":"qwen/qwen3-235b-a22b",
        "qwq-32b":"qwen/qwq-32b",
        "qwen3-7b-instuct":"qwen/qwen2-7b-instruct",
        "phi-4-mini-instruct":"microsoft/phi-4-mini-instruct"
    }
    system_list = ["art", "beam", "buckling", "cable", "car", "camera", "citybus", "curiosity", "feda", "gator", "gear",
                   "gps_imu", "handler", "hmmwv", "kraz", "lidar", "m113", "man", "mass_spring_damper", "particles",
                   "pendulum",
                   "rigid_highway", "rigid_multipatches", "rotor", "scm", "scm_hill", "sedan", "sensros",
                   "slider_crank", "tablecloth", "turtlebot", "uazbus", "veh_app", "vehros", "viper"]
    # system_do_list=["rotor", "scm", "scm_hill", "sedan", "sensros", "slider_crank", "tablecloth", "turtlebot", "uazbus", "veh_app","vehros","viper"]
    system_do_list = system_list
    dataset_path = r"C:\Users\jingquanw\SimBench\demo_data"
    output_path  = r"C:\Users\jingquanw\SimBench\output_llms"
    output_conv  = r"C:\Users\jingquanw\SimBench\output_conversion"
    test_model_list = ["deepseek-r1"]

    MAX_WORKERS = 5

    for test_model in test_model_list:
        link = opensource_model_links[test_model]
        out_model_dir = os.path.join(output_path, test_model)
        os.makedirs(out_model_dir, exist_ok=True)

        # pick only the systems you want to run:
        systems = [
            d for d in os.listdir(dataset_path)
            if d in system_do_list
        ]

        # dispatch them in parallel
        with concurrent.futures.ThreadPoolExecutor(max_workers=MAX_WORKERS) as exec:
            futures = {
                exec.submit(
                    process_system,
                    test_model, link, folder,
                    dataset_path, out_model_dir, output_conv
                ): folder
                for folder in systems
            }

            for future in tqdm(
                concurrent.futures.as_completed(futures),
                total=len(futures),
                desc=f"⟳ {test_model}"
            ):
                folder = futures[future]
                try:
                    future.result()
                except Exception as e:
                    print(f"[!] Error in {folder}: {e}")

    print("All done.")

if __name__ == "__main__":
    main()
