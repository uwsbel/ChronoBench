from openai import OpenAI
import os
from tqdm import tqdm
nvidia_api_key=""
def read_script(file_path):
    with open(file_path, "r", encoding="utf-8") as file:
        return file.read()

def generate_first_code(first_prompt,model_name):
    prompt = f"""
    {first_prompt}
    """
    try:
        global nvidia_api_key
        client = OpenAI(
            base_url="https://integrate.api.nvidia.com/v1",
            api_key=nvidia_api_key
        )
        completion = client.chat.completions.create(
            model=model_name,
            messages=[
                {"role": "system",
                 "content": "You are a PyChrono expert tasked with generating a simulation script based on the following instructions."},
                {"role": "user", "content": prompt}
            ],
            temperature=0.2,
            top_p=0.7,
            max_tokens=4096,
            stream=False
        )

    except Exception as e:
        return str(e)


# Initialize OpenAI client with your API key
openai_api_key = ""

nvidia_model_links = {
    "gemma-2-9b-it": "google/gemma-2-9b-it",
    "gemma-2-27b-it": "google/gemma-2-27b-it",
    "codegemma-1.1-7b":"google/codegemma-1.1-7b",
    "llama3-8b-instruct":"meta/llama3-8b-instruct",
    "llama-3.1-405b-instruct": "meta/llama-3.1-405b-instruct",
    "llama-3.1-70b-instruct":"meta/llama-3.1-70b-instruct",
    "llama-3.1-8b-instruct":"meta/llama-3.1-8b-instruct",
    "phi-3-mini-4k-instruct":"microsoft/phi-3-mini-4k-instruct",
    "phi-3-medium-4k-instruct":"microsoft/phi-3-medium-4k-instruct",
    "nemotron-4-304b-instruct":"nvidia/nemotron-4-340b-instruct",
    "mistral-nemo-12b-instruct":"nv-mistralai/mistral-nemo-12b-instruct",
    "mixtral-8x22b-instruct-v0.1":"mistralai/mixtral-8x22b-instruct-v0.1",
}

# data set path
dataset_path = 'D:\SimBench\demo_data'
Output_path = 'D:\SimBench\output'
# in the dataset_path, there are 34 dynamical system folders, each folder is a dyanmical system which contains 8 files [3 input text files, input1.txt, input2.txt, input3.txt;
# 2 python input files, pyinput2.py, pyinput3.py; 3 ground truth python files truth1.py, truth2.py, truth3.py]
test_model_list= ["gemma-2-9b-it","gemma-2-27b-it"]
# define an output path for the test results for each model with the name of the model
# using tqdm to show the progress bar
for test_model in tqdm(test_model_list):
    print('entering model:', test_model)
    output_model_path = os.path.join(Output_path, test_model)
    os.makedirs(output_model_path, exist_ok=True)
    # for each model, we create a folder to store the test results for each dynamical system
    for system_folder in os.listdir(dataset_path):
        print('entering folder:', system_folder)
        system_folder_path = os.path.join(dataset_path, system_folder)
        # for each dynamical system, we create a folder to store the test results for each model
        output_system_path = os.path.join(output_model_path, system_folder)
        os.makedirs(output_system_path, exist_ok=True)

        # read the input1.txt file
        input1_path = os.path.join(system_folder_path, 'input1.txt')
        input1_prompt = read_script(input1_path)
        print('input1:', input1_prompt)
        # generate the first response
        try:










input1_path = os.path.join(new_scripts_dir, 'input1.txt')
# Ensure the responses directory exists
os.makedirs(responses_dir, exist_ok=True)





def generate_code_second_third(prompt_code, input_code):
    prompt = f"""
    Here is the PyChrono code you need to modify:
        {prompt_code}
    Please modify the given code based on the following instructions:
        {input_code}
    """
    try:
        response = openai.chat.completions.create(
            model="gpt-4o",
            messages=[
                {"role": "system",
                 "content": "You are a PyChrono expert tasked with generating a simulation script based on the following instructions. Please modify the given code snippet accordingly."},
                {"role": "user", "content": prompt}
            ],
            max_tokens=4096
        )
        return response.choices[0].message.content
    except Exception as e:
        return str(e)

