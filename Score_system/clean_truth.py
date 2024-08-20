from openai import OpenAI
import os
import json
from tqdm import tqdm
import re
import logging

def remove_comments_from_file(input_py_file, output_py_file, log_file='comment_removal.log'):
    """Remove comments from a Python file and save the output to another file."""
    # Set up logging configuration
    logging.basicConfig(filename=log_file, level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

    try:
        # Read the content of the input Python file
        with open(input_py_file, 'r', encoding='utf-8') as file:
            code = file.read()

        # Remove comments from the code
        cleaned_code = remove_comments(code)

        # Save the cleaned code into a new Python file
        with open(output_py_file, 'w', encoding='utf-8') as py_file:
            py_file.write(cleaned_code)

        logging.info(f"Comments removed and cleaned Python code saved to {output_py_file} successfully.")
        return f"{output_py_file} success"

    except FileNotFoundError:
        logging.error(f"Error: The file {input_py_file} was not found.")
        return f"{output_py_file} The file {input_py_file} was not found."
    except Exception as e:
        logging.error(f"An unexpected error occurred: {e}")
        return f"{output_py_file} An unexpected error occurred: {str(e)}"

def remove_comments(code):
    """Remove comments from Python code."""
    # Remove single-line comments
    code = re.sub(r'#.*', '', code)
    # Remove multi-line comments (docstrings)
    code = re.sub(r'("""[\s\S]*?"""|\'\'\'[\s\S]*?\'\'\')', '', code)
    return code.strip()


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
#test_model_list = ["gemma-2-2b-it", "gemma-2-9b-it", "gemma-2-27b-it", "llama-3.1-405b-instruct", "llama-3.1-70b-instruct", "codellama-70b", "llama-3.1-8b-instruct", "phi-3-mini-128k-instruct", "phi-3-small-8k-instruct", "phi-3-medium-128k-instruct","nemotron-4-340b-instruct", "mistral-nemo-12b-instruct", "mixtral-8x22b-instruct-v0.1", "codestral-22b-instruct-v0.1", "mixtral-8x7b-instruct-v0.1", "mistral-large", "mamba-codestral-7b-v0.1"]

test_model_list = ["gemma-2-2b-it", "gemma-2-9b-it", "gemma-2-27b-it", "llama-3.1-405b-instruct", "llama-3.1-70b-instruct", "codellama-70b", "llama-3.1-8b-instruct", "phi-3-mini-128k-instruct", "phi-3-small-8k-instruct", "phi-3-medium-128k-instruct",
                   "nemotron-4-340b-instruct", "mistral-nemo-12b-instruct", "mixtral-8x22b-instruct-v0.1", "codestral-22b-instruct-v0.1", "mixtral-8x7b-instruct-v0.1", "mistral-large", "mamba-codestral-7b-v0.1",
                   "gpt-4o", "gpt-4o-mini", "claude-3-5-sonnet","Gemini"]
# define an output path for the test results for each model with the name of the model
# using tqdm to show the progress bar

for system_folder in os.listdir(dataset_path):
    print('entering folder:', system_folder)
    system_folder_path = os.path.join(dataset_path, system_folder)
    # for each dynamical system, we create a folder to store the test results for each model

    if True:
        # we want to clean the truth1.py, truth2.py, truth3.py files
        first_truth_path = os.path.join(system_folder_path, "truth1.py")
        second_truth_path = os.path.join(system_folder_path, "truth2.py")
        third_truth_path = os.path.join(system_folder_path, "truth3.py")

        first_cleaned_truth_path = os.path.join(system_folder_path, "cleaned_truth1.py")
        print(first_cleaned_truth_path)

        message_1_cleaned = remove_comments_from_file(first_truth_path, first_cleaned_truth_path)
        print(message_1_cleaned)

        second_cleaned_truth_path = os.path.join(system_folder_path, "cleaned_truth2.py")
        message_2_cleaned = remove_comments_from_file(second_truth_path, second_cleaned_truth_path)
        print(message_2_cleaned)

        third_cleaned_truth_path = os.path.join(system_folder_path, "cleaned_truth3.py")
        message_3_cleaned = remove_comments_from_file(third_truth_path, third_cleaned_truth_path)
        print(message_3_cleaned)

        # save the three messages to a txt file
        message_path = os.path.join(dataset_path, "extraction_message.txt")
        with open(message_path, "w", encoding="utf-8") as file:
            file.write(message_1_cleaned + '\n')
            file.write(message_2_cleaned + '\n')
            file.write(message_3_cleaned + '\n')

        # print(third_response)

print("finished")

