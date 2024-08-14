from openai import OpenAI
import os
import json
from tqdm import tqdm
nvidia_api_key="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"
key2="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"
key3="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"
def read_script(file_path):
    with open(file_path, "r", encoding="utf-8") as file:
        return file.read()

def generate_first_code(first_prompt,model_link):
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
            temperature=0.2,
            top_p=0.7,
            max_tokens=4096,
            stream=False
        )
        return completion.choices[0].message.content,prompt
    except Exception as e:
        print('error1:', e)
        return str(e),str(e)

def generate_second_third_code(prompt, code,model_link):
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
            temperature=0.2,
            top_p=0.7,
            max_tokens=4096,
            stream=False
        )
        return completion.choices[0].message.content, prompt
    except Exception as e:
        print('error2:', e)
        return str(e),str(e)

def save_conversation_json(output_conversation_path, combined_prompt1, first_response, combined_prompt2, second_response, combined_prompt3, third_response):
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
    "gemma-2-9b-it": "google/gemma-2-9b-it",
    "gemma-2-27b-it": "google/gemma-2-27b-it",
    "gemma-2-2b-it":"google/gemma-2-2b-it",
    "llama-3.1-405b-instruct": "meta/llama-3.1-405b-instruct",
    "llama-3.1-70b-instruct":"meta/llama-3.1-70b-instruct",
    "codellama-70b":"meta/codellama-70b",
    "llama-3.1-8b-instruct":"meta/llama-3.1-8b-instruct",
    "phi-3-mini-128k-instruct":"microsoft/phi-3-mini-128k-instruct",
     "phi-3-small-128k-instruct":"microsoft/phi-3-small-128k-instruct",
    "phi-3-small-8k-instruct":"microsoft/phi-3-small-8k-instruct",
    "phi-3-medium-128k-instruct":"microsoft/phi-3-medium-128k-instruct",
    "nemotron-4-340b-instruct":"nvidia/nemotron-4-340b-instruct",
    "mistral-nemo-12b-instruct":"nv-mistralai/mistral-nemo-12b-instruct",
    "mixtral-8x22b-instruct-v0.1":"mistralai/mixtral-8x22b-instruct-v0.1",
    "codestral-22b-instruct-v0.1":"mistralai/codestral-22b-instruct-v0.1",
    "mixtral-8x7b-instruct-v0.1":"mistralai/mixtral-8x7b-instruct-v0.1",
    "mistral-large":"mistralai/mistral-large",
    "mamba-codestral-7b-v0.1":"mistralai/mamba-codestral-7b-v0.1",
}
system_list = ["art", "beam", "buckling", "cable", "car", "camera", "citybus", "curiosity", "feda", "gator", "gear", "gps_imu", "handler", "hmmwv", "kraz", "lidar", "m113", "man", "mass_spring_damper", "particles", "pendulum",
               "rigid_highway", "rigid_multipatches", "rotor", "scm", "scm_hill", "sedan", "sensros", "slider_crank", "tablecloth", "turtlebot", "uazbus", "veh_app","vehros","viper"]
#system_do_list=['art', 'citybus','feda','gator','hmmwv','scm','rigid_highway','rigid_multipatches']
# data set path
dataset_path = 'D:\SimBench\demo_data'
Output_path = 'D:\SimBench\output'
Output_conversation_path = 'D:\SimBench\output_conversion'
# in the dataset_path, there are 34 dynamical system folders, each folder is a dyanmical system which contains 8 files [3 input text files, input1.txt, input2.txt, input3.txt;
# 2 python input files, pyinput2.py, pyinput3.py; 3 ground truth python files truth1.py, truth2.py, truth3.py]
test_model_list= ["nemotron-4-340b-instruct"]
# define an output path for the test results for each model with the name of the model
# using tqdm to show the progress bar
for test_model in tqdm(test_model_list):
    print('entering model:', test_model)
    test_model_link = opensource_model_links[test_model]
    output_model_path = os.path.join(Output_path, test_model)
    os.makedirs(output_model_path, exist_ok=True)
    # for each model, we create a folder to store the test results for each dynamical system
    for system_folder in os.listdir(dataset_path):
        print('entering folder:', system_folder)
        system_folder_path = os.path.join(dataset_path, system_folder)
        # for each dynamical system, we create a folder to store the test results for each model
        output_system_path = os.path.join(output_model_path, system_folder)
        os.makedirs(output_system_path, exist_ok=True)

        if  system_folder in system_list:
            input1_path = os.path.join(system_folder_path, 'input1.txt')
            input1_prompt = read_script(input1_path)
            # print('input1:', input1_prompt)
            # generate the first response
            # only test with the first system
            # if system_folder == 'art':
            # if system_folder in test_system_list:
            print("first round")
            first_response, combined_prompt1 = generate_first_code(input1_prompt, test_model_link)
        #    print(first_response, combined_prompt1)
            first_response_path = os.path.join(output_system_path, "first_response.txt")
            with open(first_response_path, 'w', encoding="utf-8") as file:
                file.write(first_response)
            # for the second and third input, the input is the input2.txt with pyinput2.py; input3.txt with pyinput3.py, respectively
            input2txt_path = os.path.join(system_folder_path, 'input2.txt')
            input2_prompt = read_script(input2txt_path)
            #print('input2:', input2_prompt)
            input2py_path = os.path.join(system_folder_path, 'pyinput2.py')
            input2_code = read_script(input2py_path)
            #print('input2 code:', input2_code)
            print("second round")
            second_response, combined_prompt2 = generate_second_third_code(input2_prompt, input2_code, test_model_link)
        #    print(second_response, combined_prompt2)
            second_response_path = os.path.join(output_system_path, "second_response.txt")
            with open(second_response_path, 'w', encoding="utf-8") as file:
                file.write(second_response)
            input3txt_path = os.path.join(system_folder_path, 'input3.txt')
            input3_prompt = read_script(input3txt_path)
        #    print('input3:', input3_prompt)
            input3py_path = os.path.join(system_folder_path, 'pyinput3.py')
            input3_code = read_script(input3py_path)
        #    print('input3 code:', input3_code)
            print("third round")
            third_response, combined_prompt3 = generate_second_third_code(input3_prompt, input3_code, test_model_link)
        #    print(third_response, combined_prompt3)
            third_response_path = os.path.join(output_system_path, "third_response.txt")
            with open(third_response_path, 'w', encoding="utf-8") as file:
                file.write(third_response)
            # save the combined prompt with the response into a json file
            output_conversation_path = os.path.join(Output_conversation_path, f"{test_model}_{system_folder}_conversation.json")
            save_conversation_json(output_conversation_path, combined_prompt1, first_response, combined_prompt2, second_response, combined_prompt3, third_response)
print("finished")
















# Read input1.txt and generate the first response
"""
prompt_code = read_script(input1_path)
first_response = generate_code_first(prompt_code)
print(first_response)
first_response_path = os.path.join(responses_dir, "first_response.txt")
with open(first_response_path, 'w', encoding="utf-8") as file:
    file.write(first_response)

# Read input2.txt and generate the second responses for corresponding scripts
input2_path = os.path.join(new_scripts_dir, 'input2.txt')
input2_code = read_script(input2_path)
for script_name in os.listdir(new_scripts_dir):
    if "_input2" in script_name:
        script_path = os.path.join(new_scripts_dir, script_name)
        prompt_code = read_script(script_path)
        second_response = generate_code_second_third(prompt_code, input2_code)
        print(second_response)
        second_response_path = os.path.join(responses_dir, "second_response.txt")
        with open(second_response_path, 'w', encoding="utf-8") as file:
            file.write(second_response)

# Read input3.txt and generate the third responses for corresponding scripts
input3_path = os.path.join(new_scripts_dir, 'input3.txt')
input3_code = read_script(input3_path)
for script_name in os.listdir(new_scripts_dir):
    if "_input3" in script_name:
        script_path = os.path.join(new_scripts_dir, script_name)
        prompt_code = read_script(script_path)
        third_response = generate_code_second_third(prompt_code, input3_code)
        print(third_response)
        third_response_path = os.path.join(responses_dir, "third_response.txt")
        with open(third_response_path, 'w', encoding="utf-8") as file:
            file.write(third_response)
print("finished")
"""