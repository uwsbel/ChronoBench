"""Similarity metrics (CodeBLEU + ROUGE) vs the cleaned ground truth. Pipeline stage 4.

For each (model, system) pair, compare the model's comment-stripped ``*_cleaned_response.py`` against
the expert ``cleaned_truth{1,2,3}.py`` and emit per-turn CodeBLEU and ROUGE rows to a CSV (default
``metrics/evaluation_results.csv``). These cheap, reference-based scores complement the J-LLM scores
(stage 5) and feed the ranking (stage 6). Models default to every dir under ``--responses-dir``;
work fans out across processes. Run:

    python scoring/p_sim_score.py [<model> ...] [--responses-dir DIR] [--out CSV] [--systems a,b]
"""
import evaluate
from codebleu import calc_codebleu
import os
import json
from tqdm import tqdm
import pandas as pd
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path
import logging
logging.getLogger("evaluate").setLevel(logging.ERROR)
os.environ["HF_HUB_DISABLE_PROGRESS_BARS"] = "1"

# -----------------------------------------------------------------------------
# Auto-detect project root based on script location
# -----------------------------------------------------------------------------
SCRIPT_DIR = Path(__file__).resolve().parent
# Script is at: <PROJECT_ROOT>/scoring/p_sim_score.py, so go up 1 level
PROJECT_ROOT = SCRIPT_DIR.parent

# Make the repo root importable so we can use the canonical system taxonomy.
import sys
sys.path.insert(0, str(PROJECT_ROOT))
from chronobench.systems import all_systems  # noqa: E402

# Default paths (auto-detected); all overridable via CLI.
dataset_path = PROJECT_ROOT / "demo_data"
DEFAULT_RESPONSES = PROJECT_ROOT / "output_llms"
DEFAULT_OUT = PROJECT_ROOT / "metrics" / "evaluation_results.csv"


def resolve_models(argv_models, responses_dir):
    """Models to score: CLI args, else $CHRONOBENCH_TEST_MODELS, else every dir under responses_dir."""
    if argv_models:
        return argv_models
    env = os.getenv("CHRONOBENCH_TEST_MODELS")
    if env:
        return [m.strip() for m in env.split(",") if m.strip()]
    return sorted(d for d in os.listdir(responses_dir)
                  if os.path.isdir(os.path.join(responses_dir, d)))

def read_script(file_path):
    with open(file_path, "r", encoding="utf-8") as file:
        return file.read()

def evaluate_system(system_folder, model, output_model_path, dataset_path):
    rouge = evaluate.load('rouge')
    # Paths based on the current system and model
    system_folder_path = os.path.join(dataset_path, system_folder)
    output_system_path = os.path.join(output_model_path, system_folder)

    if not os.path.exists(system_folder_path):
        print(f"System folder not found: {system_folder_path}")
        return None

    # Read predictions and references
    predictions = [
        read_script(os.path.join(output_system_path, "first_cleaned_response.py")),
        read_script(os.path.join(output_system_path, "second_cleaned_response.py")),
        read_script(os.path.join(output_system_path, "third_cleaned_response.py"))
    ]

    references = [
        read_script(os.path.join(system_folder_path, 'cleaned_truth1.py')),
        read_script(os.path.join(system_folder_path, 'cleaned_truth2.py')),
        read_script(os.path.join(system_folder_path, 'cleaned_truth3.py'))
    ]

    if "" in predictions + references:
        print(f"Skipping system {output_model_path},{system_folder} due to missing files.")
        #also print the missing files
        if "" in predictions:
            print(f"Missing predictions: {output_system_path}")
        if "" in references:
            print(f"Missing references: {system_folder_path}")
        return None

    # Calculate CodeBLEU
    codebleu_scores = [calc_codebleu([ref], [pred], lang="python") for ref, pred in zip(references, predictions)]

    # Calculate ROUGE
    rouge_scores = [rouge.compute(predictions=[pred], references=[ref]) for pred, ref in zip(predictions, references)]

    # Prepare data for the DataFrame
    data = []
    for i, (codebleu, rouge) in enumerate(zip(codebleu_scores, rouge_scores), 1):
        row = {
            'model': model,
            'system': system_folder,
            'round': f'round_{i}',
            'codebleu': codebleu.get('codebleu'),
            'ngram_match_score': codebleu.get('ngram_match_score'),
            'weighted_ngram_match_score': codebleu.get('weighted_ngram_match_score'),
            'syntax_match_score': codebleu.get('syntax_match_score'),
            'dataflow_match_score': codebleu.get('dataflow_match_score'),
            'rouge1': rouge.get('rouge1'),
            'rouge2': rouge.get('rouge2'),
            'rougeL': rouge.get('rougeL'),
            'rougeLsum': rouge.get('rougeLsum')
        }
        data.append(row)

    return data

def process_model_system_pair(model, system_folder, responses_dir, dataset_path):
    output_model_path = os.path.join(responses_dir, model)
    os.makedirs(output_model_path, exist_ok=True)
    return evaluate_system(system_folder, model, output_model_path, dataset_path)


def main(argv=None):
    import argparse
    ap = argparse.ArgumentParser(prog="python scoring/p_sim_score.py",
                                 description="CodeBLEU/ROUGE similarity metrics vs cleaned ground truth.")
    ap.add_argument("models", nargs="*",
                    help="models to score (default: every dir under --responses-dir; or $CHRONOBENCH_TEST_MODELS)")
    ap.add_argument("--responses-dir", default=str(DEFAULT_RESPONSES),
                    help="base dir with <model>/<system>/*_cleaned_response.py (default: output_llms/)")
    ap.add_argument("--out", default=str(DEFAULT_OUT),
                    help="output CSV (default: metrics/evaluation_results.csv). Use a runs/ path for new agents.")
    ap.add_argument("--systems", default="", help="comma-separated subset (default: all 34)")
    args = ap.parse_args(argv)

    responses_dir = args.responses_dir
    models = resolve_models(args.models, responses_dir)
    systems = [s.strip() for s in args.systems.split(",") if s.strip()] or all_systems()
    print(f"sim-score: {len(models)} model(s) x {len(systems)} systems -> {args.out}")

    all_data = []
    with ProcessPoolExecutor() as executor:
        futures = [executor.submit(process_model_system_pair, m, s, responses_dir, str(dataset_path))
                   for m in models for s in systems]
        for future in tqdm(as_completed(futures), total=len(futures)):
            result = future.result()
            if result is not None:
                all_data.extend(result)

    df = pd.DataFrame(all_data)
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    df.to_csv(args.out, index=False)
    print(f"Results saved to {args.out}\nFinished")


if __name__ == '__main__':
    main()
