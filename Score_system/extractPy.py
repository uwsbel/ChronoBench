from openai import OpenAI
import os
import json
from tqdm import tqdm
import re
import logging

def extract_python_code(txt_file_path, output_py_file, log_file='extraction.log'):
    logging.basicConfig(filename=log_file, level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

    try:
        with open(txt_file_path, 'r', encoding='utf-8') as file:
            content = file.read()

        python_code = ""
        start_match = re.search(r'```python', content)
        end_match = re.search(r'```', content[start_match.end():]) if start_match else None

        # Case 1: Full ` ```python ``` ` pair is found
        if start_match and end_match:
            python_code = content[start_match.end():start_match.end() + end_match.start()].strip()

        # Case 2: No ` ```python ` tags at all
        elif not start_match:
            python_code = content.strip()

        # Case 3: Only start ` ```python ` is detected
        elif start_match and not end_match:
            python_code = content[start_match.end():].strip()
            python_code += '\nprint("error happened with only start ```python")'

        # Save the extracted code into a Python file
        with open(output_py_file, 'w', encoding='utf-8') as py_file:
            py_file.write(python_code)

        logging.info(f"Extracted Python code saved to {output_py_file} successfully.")
        return f"{output_py_file} success"

    except FileNotFoundError:
        logging.error(f"Error: The file {txt_file_path} was not found.")
        return f"{output_py_file} The file {txt_file_path} was not found."
    except Exception as e:
        logging.error(f"An unexpected error occurred: {e}")
        return f"{output_py_file} An unexpected error occurred: {str(e)}"


nvidia_api_key = "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"
key2 = "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"
key3 = "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"


def read_script(file_path):
    with open(file_path, "r", encoding="utf-8") as file:
        return file.read()


opensource_model_links = {
    "gemma-2-9b-it": "google/gemma-2-9b-it",
    "gemma-2-27b-it": "google/gemma-2-27b-it",
    "gemma-2-2b-it": "google/gemma-2-2b-it",
    "llama-3.1-405b-instruct": "meta/llama-3.1-405b-instruct",
    "llama-3.1-70b-instruct": "meta/llama-3.1-70b-instruct",
    "codellama-70b": "meta/codellama-70b",
    "llama-3.1-8b-instruct": "meta/llama-3.1-8b-instruct",
    "phi-3-mini-128k-instruct": "microsoft/phi-3-mini-128k-instruct",
    "phi-3-small-8k-instruct": "microsoft/phi-3-small-8k-instruct",
    "phi-3-medium-128k-instruct": "microsoft/phi-3-medium-128k-instruct",
    "nemotron-4-340b-instruct": "nvidia/nemotron-4-340b-instruct",
    "mistral-nemo-12b-instruct": "nv-mistralai/mistral-nemo-12b-instruct",
    "mixtral-8x22b-instruct-v0.1": "mistralai/mixtral-8x22b-instruct-v0.1",
    "codestral-22b-instruct-v0.1": "mistralai/codestral-22b-instruct-v0.1",
    "mixtral-8x7b-instruct-v0.1": "mistralai/mixtral-8x7b-instruct-v0.1",
    "mistral-large": "mistralai/mistral-large",
    "mamba-codestral-7b-v0.1": "mistralai/mamba-codestral-7b-v0.1",
}
system_list = ["art", "beam", "buckling", "cable", "car", "camera", "citybus", "curiosity", "feda", "gator", "gear",
               "gps_imu", "handler", "hmmwv", "kraz", "lidar", "m113", "man", "mass_spring_damper", "particles",
               "pendulum",
               "rigid_highway", "rigid_multipatches", "rotor", "scm", "scm_hill", "sedan", "sensros", "slider_crank",
               "tablecloth", "turtlebot", "uazbus", "veh_app", "vehros", "viper"]

# data set path
dataset_path = 'D:\SimBench\demo_data'
Output_path = 'D:\SimBench\output'
Output_conversation_path = 'D:\SimBench\output_conversion'
# in the dataset_path, there are 34 dynamical system folders, each folder is a dyanmical system which contains 8 files [3 input text files, input1.txt, input2.txt, input3.txt;
# 2 python input files, pyinput2.py, pyinput3.py; 3 ground truth python files truth1.py, truth2.py, truth3.py]
test_model_list = ["gemma-2-2b-it", "gemma-2-9b-it", "gemma-2-27b-it", "llama-3.1-405b-instruct", "llama-3.1-70b-instruct", "codellama-70b", "llama-3.1-8b-instruct", "phi-3-mini-128k-instruct", "phi-3-small-8k-instruct", "phi-3-medium-128k-instruct", "nemotron-4-340b-instruct", "mistral-nemo-12b-instruct", "mixtral-8x22b-instruct-v0.1", "codestral-22b-instruct-v0.1", "mixtral-8x7b-instruct-v0.1", "mistral-large", "mamba-codestral-7b-v0.1"]
# define an output path for the test results for each model with the name of the model
# using tqdm to show the progress bar
for test_model in tqdm(test_model_list):
    print('entering model:', test_model)
    #test_model_link = opensource_model_links[test_model]
    output_model_path = os.path.join(Output_path, test_model)
    os.makedirs(output_model_path, exist_ok=True)
    # for each model, we create a folder to store the test results for each dynamical system
    for system_folder in os.listdir(dataset_path):
        print('entering folder:', system_folder)
        system_folder_path = os.path.join(dataset_path, system_folder)
        # for each dynamical system, we create a folder to store the test results for each model
        output_system_path = os.path.join(output_model_path, system_folder)
        os.makedirs(output_system_path, exist_ok=True)
        if True:
            #read the three reponse files

            first_response_path = os.path.join(output_system_path, "first_response.txt")

            if os.path.exists(first_response_path):
                second_response = read_script(first_response_path)
                message_1=extract_python_code(first_response_path, os.path.join(output_system_path, "first_response.py"))
                print(message_1)
            else:
                print(f"File not found: {first_response_path}")


            #print(first_response)
            second_response_path = os.path.join(output_system_path, "second_response.txt")
            if os.path.exists(second_response_path):
                second_response = read_script(second_response_path)
                message_2 = extract_python_code(second_response_path, os.path.join(output_system_path, "second_response.py"))
            else:
                print(f"File not found: {second_response_path}")

            #print(second_response)
            third_response_path = os.path.join(output_system_path, "third_response.txt")
            if os.path.exists(third_response_path):
                third_response = read_script(third_response_path)
                message_3 = extract_python_code(third_response_path, os.path.join(output_system_path, "third_response.py"))
            else:
                print(f"File not found: {third_response_path}")

            #save the three messages to a txt file
            message_path = os.path.join(output_system_path, "extraction_message.txt")
            with open(message_path, "w", encoding="utf-8") as file:
                file.write(message_1 + '\n')
                file.write(message_2 + '\n')
                file.write(message_3 + '\n')

            #print(third_response)

print("finished")

