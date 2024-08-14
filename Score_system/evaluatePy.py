from openai import OpenAI
import os
import json
from tqdm import tqdm
import re
import sys
import logging
import io
import subprocess


def run_python_file(python_file_path, log_file='execution.log'):
    logging.basicConfig(filename=log_file, level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

    try:
        # Run the Python file in a subprocess
        result = subprocess.run(
            ['python', python_file_path],
            capture_output=True,
            text=True
        )

        # Check for errors or warnings in stdout/stderr
        if result.returncode != 0:
            logging.error(
                f"An error occurred while executing the code from {python_file_path}: {result.stderr.strip()}")
            return f"{python_file_path} An error occurred: {result.stderr.strip()}"

        if "error" in result.stderr.lower() or "warning" in result.stderr.lower():
            logging.warning(f"Code executed with warnings/errors: {result.stderr.strip()}")
            return f"{python_file_path} executed with warnings/errors: {result.stderr.strip()}"

        logging.info(f"Code executed successfully from {python_file_path}!")
        return f"{python_file_path} success"

    except FileNotFoundError:
        logging.error(f"Error: The file {python_file_path} was not found.")
        return f"{python_file_path} The file was not found."
    except Exception as e:
        logging.error(f"An unexpected error occurred: {e}")
        return f"{python_file_path} An unexpected error occurred: {str(e)}"

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

# data set path
dataset_path = 'D:\SimBench\demo_data'
Output_path = 'D:\SimBench\output'
Output_conversation_path = 'D:\SimBench\output_conversion'
# in the dataset_path, there are 34 dynamical system folders, each folder is a dyanmical system which contains 8 files [3 input text files, input1.txt, input2.txt, input3.txt;
# 2 python input files, pyinput2.py, pyinput3.py; 3 ground truth python files truth1.py, truth2.py, truth3.py]
#test_model_list = ["gemma-2-2b-it", "gemma-2-9b-it", "gemma-2-27b-it", "llama-3.1-405b-instruct", "llama-3.1-70b-instruct", "codellama-70b", "llama-3.1-8b-instruct", "phi-3-mini-128k-instruct", "phi-3-small-8k-instruct", "phi-3-medium-128k-instruct",
 #                  "nemotron-4-340b-instruct", "mistral-nemo-12b-instruct", "mixtral-8x22b-instruct-v0.1", "codestral-22b-instruct-v0.1", "mixtral-8x7b-instruct-v0.1", "mistral-large", "mamba-codestral-7b-v0.1", "gpt-4o", "gpt-4o-mini", "claude-3-5-sonnet","Gemini"]
test_model_list = ["gpt-4o-mini"]
# define an output path for the test results for each model with the name of the model
# using tqdm to show the progress bar

system_list = ["art", "beam", "buckling", "cable", "car", "camera", "citybus", "curiosity", "feda", "gator", "gear", "gps_imu", "handler", "hmmwv", "kraz", "lidar", "m113", "man", "mass_spring_damper", "particles", "pendulum",
               "rigid_highway", "rigid_multipatches", "rotor", "scm", "scm_hill", "sedan", "sensros", "slider_crank", "tablecloth", "turtlebot", "uazbus", "veh_app","vehros","viper"]
system_do_list=system_list
# data set path
dataset_path = 'D:\SimBench\demo_data'
Output_path = 'D:\SimBench\output'
Output_statistic_path = 'D:\SimBench\statistic'

# using tqdm to show the progress bar
for test_model in tqdm(test_model_list):
    output_model_path = os.path.join(Output_path, test_model)
    os.makedirs(output_model_path, exist_ok=True)
    # for each model, we create a folder to store the test results for each dynamical system
    for system_folder in os.listdir(dataset_path):
        #print('entering folder:', system_folder)
        system_folder_path = os.path.join(dataset_path, system_folder)
        # for each dynamical system, we create a folder to store the test results for each model
        output_system_path = os.path.join(output_model_path, system_folder)
        os.makedirs(output_system_path, exist_ok=True)

        if system_folder in system_do_list:
            # read the three reponse python files
            first_response_path = os.path.join(output_system_path, "first_response.py")
            #print(first_response_path)
            second_response_path = os.path.join(output_system_path, "second_response.py")
            third_response_path = os.path.join(output_system_path, "third_response.py")
            message1 = run_python_file(first_response_path)

            message2 = run_python_file(second_response_path)
            print(message2)
            message3 = run_python_file(third_response_path)


print("finished")
