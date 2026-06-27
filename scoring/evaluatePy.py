from openai import OpenAI
import os
import json
from tqdm import tqdm
import re
import sys
import logging
import io
import subprocess

# -----------------------------------------------------------------------------
# Auto-detect project root based on script location
# -----------------------------------------------------------------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
# Script is at: <PROJECT_ROOT>/scoring/evaluatePy.py, so go up 1 level
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
sys.path.insert(0, PROJECT_ROOT)
from simbench.systems import all_systems  # noqa: E402


def run_python_file(python_file_path, log_file):
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

# Auto-detected paths based on project root
dataset_path = os.path.join(PROJECT_ROOT, "demo_data")
Output_path = os.path.join(PROJECT_ROOT, "output")
Output_conversation_path = os.path.join(PROJECT_ROOT, "output_conversion")
# in the dataset_path, there are 34 dynamical system folders, each folder is a dyanmical system which contains 8 files [3 input text files, input1.txt, input2.txt, input3.txt;
# 2 python input files, pyinput2.py, pyinput3.py; 3 ground truth python files truth1.py, truth2.py, truth3.py]
#test_model_list = ["gemma-2-2b-it", "gemma-2-9b-it", "gemma-2-27b-it", "llama-3.1-405b-instruct", "llama-3.1-70b-instruct", "codellama-70b", "llama-3.1-8b-instruct", "phi-3-mini-128k-instruct", "phi-3-small-8k-instruct", "phi-3-medium-128k-instruct",
def main(argv=None):
    """Compile/run each model's generated scripts (Compile@1). Requires PyChrono installed."""
    argv = sys.argv[1:] if argv is None else argv
    env = os.getenv("SIMBENCH_TEST_MODELS")
    test_model_list = argv or ([m.strip() for m in env.split(",") if m.strip()] if env else ["gpt-4o-mini-f2"])
    responses_dir = os.path.join(PROJECT_ROOT, "output_llms")
    print(f"Compile/run check for models: {test_model_list}")
    for test_model in tqdm(test_model_list):
        output_model_path = os.path.join(responses_dir, test_model)
        os.makedirs(output_model_path, exist_ok=True)
        for system_folder in all_systems():
            output_system_path = os.path.join(output_model_path, system_folder)
            os.makedirs(output_system_path, exist_ok=True)
            for resp in ("first_response.py", "second_response.py", "third_response.py"):
                print(run_python_file(os.path.join(output_system_path, resp), "execution.log"))
    print("finished")


if __name__ == "__main__":
    main()
