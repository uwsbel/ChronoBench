from openai import OpenAI
import os
import json
from tqdm import tqdm
import re
import logging

# -----------------------------------------------------------------------------
# Auto-detect project root based on script location
# -----------------------------------------------------------------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
# Script is at: <PROJECT_ROOT>/scoring/clean_truth.py, so go up 1 level
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
import sys
sys.path.insert(0, PROJECT_ROOT)
from chronobench.systems import all_systems  # noqa: E402

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


# Auto-detected paths based on project root
dataset_path = os.path.join(PROJECT_ROOT, "demo_data")


def main(argv=None):
    """Regenerate cleaned_truth{1,2,3}.py (comments stripped) for every benchmark system.

    NOTE: this rewrites files inside demo_data, which is part of the frozen contract. The regex
    is deterministic so output should be byte-identical; if it ever changes, the contract's
    tasks hash changes and the contract must be re-pinned.
    """
    for system_folder in all_systems():
        system_folder_path = os.path.join(dataset_path, system_folder)
        for t in (1, 2, 3):
            src = os.path.join(system_folder_path, f"truth{t}.py")
            dst = os.path.join(system_folder_path, f"cleaned_truth{t}.py")
            print(remove_comments_from_file(src, dst))
    print("finished")


if __name__ == "__main__":
    main()

