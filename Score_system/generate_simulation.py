import openai
import os

# Initialize OpenAI client with your API key
openai_api_key = ""
nvidia_api_key=""
nvidia_model_links = {
    "gemma-2-9b-it": "google/gemma-2-9b-it",
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


new_scripts_dir = 'C:/Users/Jingquan Wang/OneDrive - UW-Madison/Desktop/pychrono_demos/data_gen/single_pendulum'
responses_dir = new_scripts_dir
input1_path = os.path.join(new_scripts_dir, 'input1.txt')
# Ensure the responses directory exists
os.makedirs(responses_dir, exist_ok=True)

def read_script(file_path):
    with open(file_path, "r", encoding="utf-8") as file:
        return file.read()

def generate_code_first(prompt_code):
    prompt = f"""
    {prompt_code}
    """
    try:
        response = openai.chat.completions.create(
            model="gpt-4o",
            messages=[
                {"role": "system",
                 "content": "You are a PyChrono expert tasked with generating a simulation script based on the following instructions."},
                {"role": "user", "content": prompt}
            ],
            max_tokens=4096
        )
        return response.choices[0].message.content
    except Exception as e:
        return str(e)

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

# Read input1.txt and generate the first response

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
