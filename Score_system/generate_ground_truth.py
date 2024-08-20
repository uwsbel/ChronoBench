from openai import OpenAI
import os
import json
from tqdm import tqdm



def read_script(file_path):
    with open(file_path, "r", encoding="utf-8") as file:
        return file.read()


def generate_first_ground_truth(prompt, ground_truth_code):
    input_text = f"""You are a PyChrono expert tasked with generating a simulation script based on the following instructions. Make sure to:
        1. Initialize the PyChrono environment and core components.
        2. Add the required physical systems and objects as specified.
        3. Set necessary default parameters such as positions, forces, and interactions.

        Instructions:
        {prompt}
        """

    # Generate the output JSON structure
    output = {
            "instruction": input_text,
            "input": "",
            "output": f"```python\n{ground_truth_code}\n```"
        }

    return output


def generate_second_third_ground_truth(prompt, code,ground_truth_code):
    input_text = f"""You are a PyChrono expert tasked with generating a simulation script based on the following instructions and a given PyChrono script, which may contain errors. Your task has two parts: identify the potential errors in the script and correct them if exist, also follow the instructions to modify the script to meet the requirements.

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
    # Generate the output JSON structure
    output = {
        "instruction": input_text,
        "input": "",
        "output": f"```python\n{ground_truth_code}\n```"
    }

    return output


def save_json(json_data, filename):
    with open(filename, 'w') as json_file:
        json.dump(json_data, json_file, indent=4)
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



system_list = ["art", "beam", "buckling", "cable", "car", "camera", "citybus", "curiosity", "feda", "gator", "gear",
               "gps_imu", "handler", "hmmwv", "kraz", "lidar", "m113", "man", "mass_spring_damper", "particles",
               "pendulum",
               "rigid_highway", "rigid_multipatches", "rotor", "scm", "scm_hill", "sedan", "sensros", "slider_crank",
               "tablecloth", "turtlebot", "uazbus", "veh_app", "vehros", "viper"]

# data set path
dataset_path = 'D:\SimBench\demo_data'
Output_path = 'D:\SimBench\output\ground_truth'
# List to hold all JSON objects for all systems
all_systems_json_data = []

for system_folder in os.listdir(dataset_path):
    print('entering folder:', system_folder)
    system_folder_path = os.path.join(dataset_path, system_folder)
    # for each dynamical system, we create a folder to store the test results for each model
    output_system_path = os.path.join(Output_path, system_folder)
    os.makedirs(output_system_path, exist_ok=True)

    if system_folder in system_list:
        input1_path = os.path.join(system_folder_path, 'input1.txt')
        input1_prompt = read_script(input1_path)
        input1_ground_truth_code_path = os.path.join(system_folder_path, 'truth1.py')
        input1_ground_truth_code = read_script(input1_ground_truth_code_path)
        print("first round")
        json1 = generate_first_ground_truth(input1_prompt, input1_ground_truth_code)

        # Append first round JSON to the list
        all_systems_json_data.append(json1)

        print(json1)
        save_json(json1, os.path.join(system_folder_path, 'output1.json'))

        input2_prompt_path = os.path.join(system_folder_path, 'input2.txt')
        input2_prompt = read_script(input2_prompt_path)
        input2_ground_truth_code_path = os.path.join(system_folder_path, 'truth2.py')
        input2_ground_truth_code = read_script(input2_ground_truth_code_path)
        input2_py_path = os.path.join(system_folder_path, 'pyinput2.py')
        input2_code = read_script(input2_py_path)

        json2 = generate_second_third_ground_truth(input2_prompt, input2_code, input2_ground_truth_code)

        # Append first round JSON to the list
        all_systems_json_data.append(json2)

        # Save the second round JSON to a file
        save_json(json2, os.path.join(system_folder_path, 'output2.json'))

        input3_prompt_path = os.path.join(system_folder_path, 'input3.txt')
        input3_prompt = read_script(input3_prompt_path)
        input3_ground_truth_code_path = os.path.join(system_folder_path, 'truth3.py')
        input3_ground_truth_code = read_script(input3_ground_truth_code_path)
        print("third round")
        json3 = generate_second_third_ground_truth(input3_prompt, input2_code, input3_ground_truth_code)

        # Append first round JSON to the list
        all_systems_json_data.append(json3)

        # Save the third round JSON to a file
        save_json(json3, os.path.join(system_folder_path, 'output3.json'))

# Save the combined JSON list for all systems to a single file
combined_json_path = os.path.join('D:\SimBench\\api', 'ground_truth.json')
save_json(all_systems_json_data, combined_json_path)

