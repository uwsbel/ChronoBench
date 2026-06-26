"""Extract runnable Python from S-LLM .txt responses, plus a comment-stripped copy.

For each model and system, reads
``output_llms/<model>/<system>/{first,second,third}_response.txt`` and writes the matching
``*_response.py`` (code extracted from the markdown/code fences) and ``*_cleaned_response.py``
(comments removed, for the similarity metrics in ``p_sim_score.py``).

Models come from the command line, else ``$SIMBENCH_TEST_MODELS`` (comma-separated), else
``DEFAULT_TEST_MODELS``:

    python scoring/extractPy.py <model> [<model> ...]

(This file previously contained its entire body twice by accident; it has been de-duplicated.
The system list is taken from the canonical ``simbench.systems`` taxonomy rather than scanning
``demo_data/`` so that non-system entries like ``manifest.json`` are not treated as systems.)
"""

from __future__ import annotations

import logging
import os
import re
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from simbench.systems import all_systems  # noqa: E402

DEFAULT_TEST_MODELS = ["llama3.3-70b-lora1"]
ROUNDS = ["first", "second", "third"]

Output_path = os.path.join(PROJECT_ROOT, "output_llms")


def extract_python_code(txt_file_path, output_py_file, log_file="extraction.log"):
    logging.basicConfig(filename=log_file, level=logging.INFO,
                        format="%(asctime)s - %(levelname)s - %(message)s")
    try:
        with open(txt_file_path, "r", encoding="utf-8") as file:
            content = file.read()

        python_code = ""
        start_match = re.search(r"```python", content)
        end_match = re.search(r"```", content[start_match.end():]) if start_match else None

        # Case 1: a full ```python ... ``` pair is found
        if start_match and end_match:
            python_code = content[start_match.end():start_match.end() + end_match.start()].strip()

        # Case (multiple): several ```python ... ``` pairs
        multiple_matches = re.findall(r"```python(.*?)```", content, re.DOTALL)
        if multiple_matches:
            python_code = "\n\n".join(match.strip() for match in multiple_matches)

        # Case 2: no ```python fences at all
        elif not start_match:
            python_code = content.strip()

        # Case 3: only an opening ```python fence
        elif start_match and not end_match:
            python_code = content[start_match.end():].strip()
            python_code += '\nprint("error happened with only start ```python")'

        with open(output_py_file, "w", encoding="utf-8") as py_file:
            py_file.write(python_code)

        logging.info(f"Extracted Python code saved to {output_py_file} successfully.")
        return f"{output_py_file} success"

    except FileNotFoundError:
        logging.error(f"Error: The file {txt_file_path} was not found.")
        return f"{output_py_file} The file {txt_file_path} was not found."
    except Exception as e:
        logging.error(f"An unexpected error occurred: {e}")
        return f"{output_py_file} An unexpected error occurred: {str(e)}"


def remove_comments(code):
    """Remove comments and docstrings from Python source."""
    code = re.sub(r"#.*", "", code)
    code = re.sub(r"(\"\"\"[\s\S]*?\"\"\"|'''[\s\S]*?''')", "", code)
    return code.strip()


def remove_comments_from_file(input_py_file, output_py_file, log_file="comment_removal.log"):
    logging.basicConfig(filename=log_file, level=logging.INFO,
                        format="%(asctime)s - %(levelname)s - %(message)s")
    try:
        with open(input_py_file, "r", encoding="utf-8") as file:
            code = file.read()
        with open(output_py_file, "w", encoding="utf-8") as py_file:
            py_file.write(remove_comments(code))
        logging.info(f"Comments removed; cleaned code saved to {output_py_file} successfully.")
        return f"{output_py_file} success"
    except FileNotFoundError:
        logging.error(f"Error: The file {input_py_file} was not found.")
        return f"{output_py_file} The file {input_py_file} was not found."
    except Exception as e:
        logging.error(f"An unexpected error occurred: {e}")
        return f"{output_py_file} An unexpected error occurred: {str(e)}"


def process_model(test_model):
    output_model_path = os.path.join(Output_path, test_model)
    os.makedirs(output_model_path, exist_ok=True)
    print("entering model:", test_model)

    for i, system_folder in enumerate(all_systems(), start=1):
        output_system_path = os.path.join(output_model_path, system_folder)
        os.makedirs(output_system_path, exist_ok=True)
        print(f"  [{i}] {system_folder}")

        messages = []
        for round_name in ROUNDS:
            txt_path = os.path.join(output_system_path, f"{round_name}_response.txt")
            py_path = os.path.join(output_system_path, f"{round_name}_response.py")
            cleaned_path = os.path.join(output_system_path, f"{round_name}_cleaned_response.py")
            if os.path.exists(txt_path):
                messages.append(extract_python_code(txt_path, py_path))
                messages.append(remove_comments_from_file(py_path, cleaned_path))
            else:
                print(f"    File not found: {txt_path}")
                messages.append(f"{py_path} source .txt not found")

        with open(os.path.join(output_system_path, "extraction_message.txt"), "w",
                  encoding="utf-8") as file:
            file.write("\n".join(messages) + "\n")


def resolve_test_models(argv):
    if argv:
        return argv
    env = os.getenv("SIMBENCH_TEST_MODELS")
    if env:
        return [m.strip() for m in env.split(",") if m.strip()]
    return DEFAULT_TEST_MODELS


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    test_model_list = resolve_test_models(argv)
    print(f"Extracting for models: {test_model_list}")
    for test_model in test_model_list:
        process_model(test_model)
    print("finished")


if __name__ == "__main__":
    main()
