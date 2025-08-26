from openai import OpenAI
import os
import json
from tqdm import tqdm
from pathlib import Path

def load_env():
    """Load environment variables from .env file"""
    env_path = Path(__file__).parent.parent.parent / '.env'
    if env_path.exists():
        with open(env_path) as f:
            for line in f:
                if line.strip() and not line.startswith('#'):
                    if '=' in line:
                        key, value = line.strip().split('=', 1)
                        # Remove quotes if present
                        value = value.strip('"').strip("'")
                        os.environ[key] = value
        print(f"✓ Loaded API keys from {env_path}")
    else:
        print(f"⚠ Warning: .env file not found at {env_path}")

# Load environment variables from .env
load_env()

# Get API key from environment
api_key = os.getenv("OPENAI_API_KEY", "")
if not api_key:
    print("❌ Error: OPENAI_API_KEY not found. Please check your .env file")
    print("   Expected: OPENAI_API_KEY='your-key-here' in .env")
    exit(1)
else:
    print(f"✓ OpenAI API key loaded (length: {len(api_key)})")
def read_script(file_path):
    with open(file_path, "r", encoding="utf-8") as file:
        return file.read()


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
        global api_key
        client = OpenAI(api_key=api_key)
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
        global api_key
        client = OpenAI(api_key=api_key)
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
system_list = ["art", "beam", "buckling", "cable", "car", "camera", "citybus", "curiosity", "feda", "gator", "gear",
               "gps_imu", "handler", "hmmwv", "kraz", "lidar", "m113", "man", "mass_spring_damper", "particles",
               "pendulum",
               "rigid_highway", "rigid_multipatches", "rotor", "scm", "scm_hill", "sedan", "sensros", "slider_crank",
               "tablecloth", "turtlebot", "uazbus", "veh_app", "vehros", "viper"]
system_do_list = ["art", "beam", "buckling", "cable", "car", "camera", "citybus", "curiosity","feda", "gator", "gear", "gps_imu", "handler", "hmmwv", "kraz", "lidar", "m113", "man",
                  "mass_spring_damper", "particles", "pendulum",
                  "rigid_highway", "rigid_multipatches", "rotor", "scm", "scm_hill", "sedan", "sensros", "slider_crank",
                  "tablecloth", "turtlebot", "uazbus", "veh_app", "vehros", "viper"]
# data set path
dataset_path = r"/home/hongyu/Documents/SimBench/demo_data"
output_path  = r"/home/hongyu/Documents/SimBench/output_llms"
output_conv  = r"/home/hongyu/Documents/SimBench/output_conversion"
# in the dataset_path, there are 34 dynamical system folders, each folder is a dyanmical system which contains 8 files [3 input text files, input1.txt, input2.txt, input3.txt;
# 2 python input files, pyinput2.py, pyinput3.py; 3 ground truth python files truth1.py, truth2.py, truth3.py]
test_model_list = ["gpt-4o-mini-f1", "gpt-4o-mini-f3"]
# define an output path for the test results for each model with the name of the model
# using tqdm to show the progress bar
for test_model in tqdm(test_model_list):
    print('entering model:', test_model)
    test_model_link = opensource_model_links[test_model]
    output_model_path = os.path.join(output_path, test_model)
    os.makedirs(output_model_path, exist_ok=True)
    # for each model, we create a folder to store the test results for each dynamical system
    for system_folder in os.listdir(dataset_path):
        system_folder_path = os.path.join(dataset_path, system_folder)
        # Skip if it's not a directory
        if not os.path.isdir(system_folder_path):
            continue
        print('entering folder:', system_folder)
        # for each dynamical system, we create a folder to store the test results for each model
        output_system_path = os.path.join(output_model_path, system_folder)
        os.makedirs(output_system_path, exist_ok=True)

        # read the input1.txt file
        input1_path = os.path.join(system_folder_path, 'input1.txt')
        input1_prompt = read_script(input1_path)
        # print('input1:', input1_prompt)
        # generate the first response
        # only test with the first system
        # if system_folder == 'art':
        # if system_folder in test_system_list:
        if True:
            print("first round")
            first_response, combined_prompt1 = generate_first_code(input1_prompt, test_model_link)
            #    print(first_response, combined_prompt1)
            first_response_path = os.path.join(output_system_path, "first_response.txt")
            with open(first_response_path, 'w', encoding="utf-8") as file:
                file.write(first_response)
            # for the second and third input, the input is the input2.txt with pyinput2.py; input3.txt with pyinput3.py, respectively
            input2txt_path = os.path.join(system_folder_path, 'input2.txt')
            input2_prompt = read_script(input2txt_path)
            # print('input2:', input2_prompt)
            input2py_path = os.path.join(system_folder_path, 'pyinput2.py')
            input2_code = read_script(input2py_path)
            # print('input2 code:', input2_code)
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
            output_conversation_path = os.path.join(output_conv,
                                                    f"{test_model}_{system_folder}_conversation.json")
            save_conversation_json(output_conversation_path, combined_prompt1, first_response, combined_prompt2,
                                   second_response, combined_prompt3, third_response)
print("finished")

