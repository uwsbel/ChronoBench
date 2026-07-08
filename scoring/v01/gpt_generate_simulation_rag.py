from openai import OpenAI
import argparse
import os
import json
import pathlib
import sys
from tqdm import tqdm

# -----------------------------------------------------------------------------
# Auto-detect project root based on script location
# -----------------------------------------------------------------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
# Script is at: <PROJECT_ROOT>/SimBench/scoring/v01/gpt_generate_simulation.py
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", ".."))  # chrono-rag/
SIMBENCH_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))       # chrono-rag/SimBench/
sys.path.insert(0, str(pathlib.Path(PROJECT_ROOT) / "src"))

nvidia_api_key = os.getenv("OPENAI_API_KEY")
print(nvidia_api_key)
def read_script(file_path):
    with open(file_path, "r", encoding="utf-8") as file:
        return file.read()


def generate_first_code(first_prompt, model_link, rag_context=""):
    # because some of the models like gemma-2 do not have a system role, so we add the system role to the user role prompt
    prompt = f"""
    You are a PyChrono expert tasked with generating a simulation script based on the following instructions. Make sure to:
    1. Initialize the PyChrono environment and core components.
    2. Add the required physical systems and objects as specified.
    3. Set necessary default parameters such as positions, forces, and interactions.

    Instructions:
    {first_prompt}
    """
    if rag_context:
        prompt += f"\n\nAdditional context from PyChrono documentation:\n{rag_context}"
    try:
        global nvidia_api_key
        client = OpenAI(api_key=nvidia_api_key)
        completion = client.chat.completions.create(
            model=model_link,
            messages=[
                {"role": "user", "content": prompt}
            ],
            temperature=0.1,
            top_p=0.95,
            max_completion_tokens=4096*4,
            stream=False
        )
        return completion.choices[0].message.content, prompt
    except Exception as e:
        print('error1:', e)
        return str(e), str(e)


def generate_second_third_code(instructions, code, model_link, rag_context=""):
    prompt = f"""

    You are a PyChrono expert tasked with generating a simulation script based on the following instructions and a given PyChrono script, which may contain errors. Your task has two parts: identify the potential errors in the script and correct them if exist, also follow the instructions to modify the script to meet the requirements.

Here is the PyChrono code you need to modify:
{code}


Please modify the given code based on the following instructions:
{instructions}

To complete the task, follow these steps:

Review the given PyChrono script and identify any errors, including syntax errors, logical errors, incorrect method names, and parameter issues.
Correct the identified errors in the script to ensure it runs correctly.
Modify the script based on the provided instructions to ensure it meets the specified requirements.

Provide the corrected and modified script below:
    """
    if rag_context:
        prompt += f"\n\nAdditional context from PyChrono documentation:\n{rag_context}"
    try:
        global nvidia_api_key
        client = OpenAI(api_key=nvidia_api_key)
        completion = client.chat.completions.create(
            model=model_link,
            messages=[
                {"role": "user", "content": prompt}
            ],
            temperature=0.1,
            top_p=0.95,
            max_completion_tokens=1024 * 4 * 4,
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


opensource_model_links = {
    "gpt-4o-mini": "gpt-4o-mini",
    "gpt-4o-mini-f1":"ft:gpt-4o-mini-2024-07-18:personal::9xVAdwNY",
    "gpt-4.1":"gpt-4.1",
    "gpt-4.1-mini":"gpt-4.1-mini",
    "gpt-4.1-nano":"gpt-4.1-nano",
    "o4-mini":"o4-mini",
    "o3":"o3",
    "gpt-4o-mini-f3":"ft:gpt-4o-mini-2024-07-18:uw-sbel::A6Rd900h"
}
system_list = ["art", "beam", "buckling", "cable", "camera", "citybus", "curiosity", "feda", "gator", "gear",
               "gps_imu", "handler", "hmmwv", "kraz", "lidar", "m113", "man", "mass_spring_damper", "particles",
               "pendulum",
               "rigid_highway", "rigid_multipatches", "rotor", "scm", "scm_hill", "sedan", "sensros", "slider_crank",
               "tablecloth", "turtlebot", "uazbus", "veh_app", "vehros", "viper"]
system_do_list = ["art", "beam", "buckling", "cable", "camera", "citybus", "curiosity","feda", "gator", "gear", "gps_imu", "handler", "hmmwv", "kraz", "lidar", "m113", "man",
                  "mass_spring_damper", "particles", "pendulum",
                  "rigid_highway", "rigid_multipatches", "rotor", "scm", "scm_hill", "sedan", "sensros", "slider_crank",
                  "tablecloth", "turtlebot", "uazbus", "veh_app", "vehros", "viper"]
# Data paths live under SimBench/, not at the project root
dataset_path = os.path.join(SIMBENCH_ROOT, "demo_data")
output_path = os.path.join(SIMBENCH_ROOT, "output_llms")
output_conv = os.path.join(SIMBENCH_ROOT, "output_conversion")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run SimBench generation for GPT models.")
    parser.add_argument("--rag", action="store_true", help="Inject RAG context from the vector store")
    parser.add_argument("--model", default="gpt-4o-mini-f3", choices=list(opensource_model_links.keys()),
                        help="Model to benchmark")
    parser.add_argument("--systems", nargs="+", default=None,
                        help="Subset of systems to run (default: all in system_do_list)")
    args = parser.parse_args()

    if args.rag:
        from inference.vector_search import retrieve

    test_model_list = [args.model]
    systems_to_run = args.systems if args.systems else list(system_do_list)

    # using tqdm to show the progress bar
    for test_model in tqdm(test_model_list):
        print('entering model:', test_model)
        test_model_link = opensource_model_links[test_model]
        folder_name = f"{test_model}-rag" if args.rag else test_model
        output_model_path = os.path.join(output_path, folder_name)
        os.makedirs(output_model_path, exist_ok=True)
        # for each model, we create a folder to store the test results for each dynamical system
        for system_folder in systems_to_run:
            print('entering folder:', system_folder)
            system_folder_path = os.path.join(dataset_path, system_folder)
            # for each dynamical system, we create a folder to store the test results for each model
            output_system_path = os.path.join(output_model_path, system_folder)
            os.makedirs(output_system_path, exist_ok=True)

            # retrieve RAG context once per system using the turn-1 query
            rag_context = ""
            if args.rag:
                turn1_query = read_script(os.path.join(system_folder_path, 'input1.txt'))
                rag_context = retrieve(turn1_query)

            # read the input1.txt file
            input1_path = os.path.join(system_folder_path, 'input1.txt')
            input1_prompt = read_script(input1_path)
            print("first round")
            first_response, combined_prompt1 = generate_first_code(input1_prompt, test_model_link, rag_context)
            first_response_path = os.path.join(output_system_path, "first_response.txt")
            with open(first_response_path, 'w', encoding="utf-8") as file:
                file.write(first_response)
            # for the second and third input, the input is the input2.txt with pyinput2.py; input3.txt with pyinput3.py, respectively
            input2txt_path = os.path.join(system_folder_path, 'input2.txt')
            input2_prompt = read_script(input2txt_path)
            input2py_path = os.path.join(system_folder_path, 'pyinput2.py')
            input2_code = read_script(input2py_path)
            print("second round")
            second_response, combined_prompt2 = generate_second_third_code(input2_prompt, input2_code, test_model_link, rag_context)
            second_response_path = os.path.join(output_system_path, "second_response.txt")
            with open(second_response_path, 'w', encoding="utf-8") as file:
                file.write(second_response)
            input3txt_path = os.path.join(system_folder_path, 'input3.txt')
            input3_prompt = read_script(input3txt_path)
            input3py_path = os.path.join(system_folder_path, 'pyinput3.py')
            input3_code = read_script(input3py_path)
            print("third round")
            third_response, combined_prompt3 = generate_second_third_code(input3_prompt, input3_code, test_model_link, rag_context)
            third_response_path = os.path.join(output_system_path, "third_response.txt")
            with open(third_response_path, 'w', encoding="utf-8") as file:
                file.write(third_response)
            # save the combined prompt with the response into a json file
            output_conversation_path = os.path.join(output_conv,
                                                    f"{test_model}_{system_folder}_conversation.json")
            save_conversation_json(output_conversation_path, combined_prompt1, first_response, combined_prompt2,
                                   second_response, combined_prompt3, third_response)
    print("finished")

