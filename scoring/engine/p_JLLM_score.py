"""Batch J-LLM scoring of S-LLM outputs (the operational engine, NOT legacy code).

This drives the rule-based judge over ``output_llms/<model>/<system>/{first,second,third}_response.py``
and writes, per (model, system): three ``*_score_*.txt`` files (doc / reference / reference+doc
modes), a ``*_evaluation.json`` audit record, a per-system ``evaluation_scores.csv``, and finally
a merged ``output_llms/combined_evaluation_scores.csv``.

The rubric/judge logic now lives in the reusable ``chronobench.judge`` package (single source of
truth for the prompts); this file is the batch harness around it. Earlier versions duplicated the
rubric three times inline, hardcoded the judge model, printed the API key, and ran at import time.

Usage:
    # judge model: $CHRONOBENCH_JUDGE_MODEL (default gpt-4o-mini); needs $OPENAI_API_KEY
    python scoring/engine/p_JLLM_score.py <test_model> [<test_model> ...]
    # or set CHRONOBENCH_TEST_MODELS="modelA,modelB"; else falls back to DEFAULT_TEST_MODELS below.
"""

import os
import sys
import json
import csv
import re
from concurrent.futures import ThreadPoolExecutor, as_completed

from tqdm import tqdm

# -----------------------------------------------------------------------------
# Make the repo root importable so `import chronobench` works when run from scoring/engine/.
# Script is at <PROJECT_ROOT>/scoring/engine/p_JLLM_score.py, so go up 2 levels.
# -----------------------------------------------------------------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from chronobench.judge import evaluate_script, DEFAULT_MODEL  # noqa: E402
from chronobench.systems import all_systems  # noqa: E402

# Judge model + sampling come from chronobench.judge defaults ($CHRONOBENCH_JUDGE_MODEL).
JUDGE_MODEL = DEFAULT_MODEL

# Default S-LLMs to score if none given on the CLI / via $CHRONOBENCH_TEST_MODELS.
DEFAULT_TEST_MODELS = ["llama3.3-70b-lora1"]

dataset_path = os.path.join(PROJECT_ROOT, "demo_data")
Output_path = os.path.join(PROJECT_ROOT, "output_llms")
api_text_path = os.path.join(PROJECT_ROOT, "api", "api.txt")


def read_script(file_path):
    with open(file_path, "r", encoding="utf-8") as file:
        return file.read()


def evaluate_and_save_results(round_name, prediction, reference_code, api_documentation,
                              output_system_path, client=None):
    """Run the three rubric modes for one turn and persist scores + prompts (unchanged schema)."""
    # doc / reference / reference+document, matching the original three output files.
    ev_doc = evaluate_script(prediction, api_doc=api_documentation, mode="doc",
                         model=JUDGE_MODEL, client=client)
    ev_ref = evaluate_script(prediction, reference=reference_code, mode="ref",
                         model=JUDGE_MODEL, client=client)
    ev_ref_doc = evaluate_script(prediction, reference=reference_code, api_doc=api_documentation,
                             mode="ref_doc", model=JUDGE_MODEL, client=client)

    paths = {
        "score_document": (f"{round_name}_score_document.txt", ev_doc),
        "score_reference": (f"{round_name}_score_reference.txt", ev_ref),
        "score_reference_document": (f"{round_name}_score_reference_document.txt", ev_ref_doc),
    }
    for _, (fname, ev) in paths.items():
        with open(os.path.join(output_system_path, fname), "w", encoding="utf-8") as f:
            f.write(ev.raw)

    evaluation_data = {
        "round_name": round_name,
        "prediction": prediction,
        "reference_code": reference_code,
        "api_documentation": api_documentation,
        "output_system_path": output_system_path,
        "judge_model": JUDGE_MODEL,
        "scores": {
            "score_document": ev_doc.raw,
            "score_reference": ev_ref.raw,
            "score_reference_document": ev_ref_doc.raw,
        },
        "parsed_scores": {
            "score_document": ev_doc.score,
            "score_reference": ev_ref.score,
            "score_reference_document": ev_ref_doc.score,
        },
        "prompts": {
            "prompt_document": ev_doc.prompt,
            "prompt_reference": ev_ref.prompt,
            "prompt_reference_document": ev_ref_doc.prompt,
        },
    }
    with open(os.path.join(output_system_path, f"{round_name}_evaluation.json"), "w",
              encoding="utf-8") as jf:
        json.dump(evaluation_data, jf, indent=4, ensure_ascii=False)


def extract_scores_from_txt(file_path):
    """Extract the numerical score in the format [[x]] from a saved judge response."""
    with open(file_path, "r", encoding="utf-8") as file:
        content = file.read()
    match = re.search(r"\[\[(\d+)\]\]", content)
    if match:
        return int(match.group(1))
    raise ValueError(f"No valid score found in {file_path}")


def save_scores_to_csv_with_metadata(output_system_path, test_model, system_folder,
                                     csv_filename="evaluation_scores.csv"):
    """Read the per-round score txt files and write a per-system CSV (unchanged schema)."""
    csv_data = [["Test Model", "System", "Round", "Score Document", "Score Reference",
                 "Score Reference Document"]]
    for round_name in ["first", "second", "third"]:
        try:
            score_document = extract_scores_from_txt(
                os.path.join(output_system_path, f"{round_name}_score_document.txt"))
            score_reference = extract_scores_from_txt(
                os.path.join(output_system_path, f"{round_name}_score_reference.txt"))
            score_reference_document = extract_scores_from_txt(
                os.path.join(output_system_path, f"{round_name}_score_reference_document.txt"))
            csv_data.append([test_model, system_folder, round_name, score_document,
                             score_reference, score_reference_document])
        except Exception as e:
            print(f"Error processing {round_name} in {system_folder} for {test_model}: {e}")

    csv_output_path = os.path.join(output_system_path, csv_filename)
    with open(csv_output_path, "w", newline="", encoding="utf-8") as csvfile:
        csv.writer(csvfile).writerows(csv_data)
    print(f"Scores saved to {csv_output_path}")


def merge_csv_files(output_path, combined_csv_filename="combined_evaluation_scores.csv"):
    """Merge every per-system evaluation_scores.csv under output_path into one CSV."""
    combined_csv_data = []
    for model_dir in os.listdir(output_path):
        model_path = os.path.join(output_path, model_dir)
        if not os.path.isdir(model_path):
            continue
        for system_dir in os.listdir(model_path):
            small_csv_path = os.path.join(model_path, system_dir, "evaluation_scores.csv")
            if os.path.exists(small_csv_path):
                with open(small_csv_path, "r", encoding="utf-8") as csvfile:
                    reader = csv.reader(csvfile)
                    headers = next(reader)
                    if not combined_csv_data:
                        combined_csv_data.append(headers)
                    combined_csv_data.extend(list(reader))
    combined_csv_path = os.path.join(output_path, combined_csv_filename)
    with open(combined_csv_path, "w", newline="", encoding="utf-8") as csvfile:
        csv.writer(csvfile).writerows(combined_csv_data)
    print(f"Combined CSV file saved to {combined_csv_path}")


def process_model_system(test_model, system_folder, client=None):
    system_folder_path = os.path.join(dataset_path, system_folder)
    output_system_path = os.path.join(Output_path, test_model, system_folder)
    os.makedirs(output_system_path, exist_ok=True)
    print(f"Processing model {test_model} on system {system_folder}")

    predictions = {
        "first": os.path.join(output_system_path, "first_response.py"),
        "second": os.path.join(output_system_path, "second_response.py"),
        "third": os.path.join(output_system_path, "third_response.py"),
    }
    references = {
        "first": os.path.join(system_folder_path, "truth1.py"),
        "second": os.path.join(system_folder_path, "truth2.py"),
        "third": os.path.join(system_folder_path, "truth3.py"),
    }
    api_documentation = read_script(api_text_path)

    for round_name in ("first", "second", "third"):
        if not os.path.exists(predictions[round_name]):
            print(f"  missing prediction: {predictions[round_name]}, skipping {round_name}")
            continue
        evaluate_and_save_results(
            round_name,
            read_script(predictions[round_name]),
            read_script(references[round_name]),
            api_documentation,
            output_system_path,
            client=client,
        )

    save_scores_to_csv_with_metadata(output_system_path, test_model, system_folder)
    return f"Completed {system_folder} for model {test_model}"


def resolve_test_models(argv):
    if argv:
        return argv
    env = os.getenv("CHRONOBENCH_TEST_MODELS")
    if env:
        return [m.strip() for m in env.split(",") if m.strip()]
    return DEFAULT_TEST_MODELS


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    test_model_list = resolve_test_models(argv)
    system_list = all_systems()

    # One shared OpenAI-compatible client for the whole batch.
    from openai import OpenAI
    client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

    print(f"Judge model: {JUDGE_MODEL} | test models: {test_model_list} "
          f"| systems: {len(system_list)}")

    with ThreadPoolExecutor(max_workers=100) as executor:
        futures = []
        for test_model in test_model_list:
            os.makedirs(os.path.join(Output_path, test_model), exist_ok=True)
            for system_folder in system_list:
                futures.append(executor.submit(
                    process_model_system, test_model, system_folder, client))
        for future in tqdm(as_completed(futures), total=len(futures)):
            print(future.result())

    # Merge AFTER scoring so the combined CSV reflects the new results.
    merge_csv_files(Output_path)
    print("Finished processing all models and systems.")


if __name__ == "__main__":
    main()
