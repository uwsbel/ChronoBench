import evaluate
from codebleu import calc_codebleu

from openai import OpenAI
import os
import json
from tqdm import tqdm
import re
import sys
import logging
import io
import subprocess
rouge = evaluate.load('rouge')
predictions = ["hello there, where are you?"]
references = ["hello there fukkkkk!"]
results = rouge.compute(predictions=predictions,references=references)
print(results)



prediction = "def add ( a , b ) :\n return a + b"
reference = "def sum ( first , second ) :\n return second + first"

result = calc_codebleu([reference], [prediction], lang="python", weights=(0.25, 0.25, 0.25, 0.25), tokenizer=None)
print(result)



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
            first_prediction = read_script(first_response_path)
            second_prediction = read_script(second_response_path)
            third_prediction = read_script(third_response_path)
            first_reference_path = os.path.join(system_folder_path, 'truth1.py')
            second_reference_path = os.path.join(system_folder_path, 'truth2.py')
            third_reference_path = os.path.join(system_folder_path, 'truth3.py')
            first_reference = read_script(first_reference_path)
            second_reference = read_script(second_reference_path)
            third_reference = read_script(third_reference_path)
            first_result = calc_codebleu([first_reference], [first_prediction], lang="python", weights=(0.25, 0.25, 0.25, 0.25), tokenizer=None)
            second_result = calc_codebleu([second_reference], [second_prediction], lang="python", weights=(0.25, 0.25, 0.25, 0.25), tokenizer=None)
            third_result = calc_codebleu([third_reference], [third_prediction], lang="python", weights=(0.25, 0.25, 0.25, 0.25), tokenizer=None)
            print(first_result, second_result, third_result)

            #do rouge score
            predictions = [first_prediction, second_prediction, third_prediction]
            references = [first_reference, second_reference, third_reference]
            results = rouge.compute(predictions=predictions,references=references)
            print(results)


print("finished")
