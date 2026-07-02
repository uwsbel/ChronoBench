import argparse
import json
import os
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
from typing import Iterable, Optional, Tuple

from openai import OpenAI
from tqdm import tqdm


SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent.parent


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate SimBench model outputs through a local OpenAI-compatible vLLM endpoint."
    )
    parser.add_argument("--model-name", required=True, help="Name used for SimBench output directories.")
    parser.add_argument(
        "--model-id",
        default=None,
        help="Model id sent to the OpenAI API. Defaults to --model-name.",
    )
    parser.add_argument(
        "--base-url",
        default=os.environ.get("SIMBENCH_OPENAI_BASE_URL")
        or os.environ.get("OPENAI_BASE_URL")
        or "http://127.0.0.1:8000/v1",
        help="OpenAI-compatible endpoint, usually http://127.0.0.1:8000/v1.",
    )
    parser.add_argument(
        "--api-key",
        default=os.environ.get("OPENAI_API_KEY", "0"),
        help="API key for the OpenAI-compatible endpoint.",
    )
    parser.add_argument(
        "--dataset-dir",
        type=Path,
        default=PROJECT_ROOT / "demo_data",
        help="Path to SimBench demo_data.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=PROJECT_ROOT / "output_llms",
        help="Directory for per-model response files.",
    )
    parser.add_argument(
        "--conversation-dir",
        type=Path,
        default=PROJECT_ROOT / "output_conversion",
        help="Directory for conversation JSON files.",
    )
    parser.add_argument(
        "--systems",
        default=None,
        help="Comma-separated system names to run, for example rotor,viper. Defaults to all systems.",
    )
    parser.add_argument("--limit", type=int, default=None, help="Run only the first N selected systems.")
    parser.add_argument("--max-workers", type=int, default=4, help="Parallel systems to submit to vLLM.")
    parser.add_argument("--retries", type=int, default=3)
    parser.add_argument("--retry-sleep", type=float, default=2.0)
    parser.add_argument("--temperature", type=float, default=0.2)
    parser.add_argument("--top-p", type=float, default=1.0)
    parser.add_argument("--max-tokens", type=int, default=4096)
    parser.add_argument(
        "--chat-template-kwargs-json",
        default=None,
        help='Optional JSON passed as extra_body.chat_template_kwargs, e.g. \'{"enable_thinking": false}\'.',
    )
    parser.add_argument(
        "--skip-existing",
        action="store_true",
        help="Skip systems with all three response files already present.",
    )
    return parser.parse_args()


def read_script(file_path: Path) -> str:
    return file_path.read_text(encoding="utf-8")


def selected_system_dirs(dataset_dir: Path, systems: Optional[str], limit: Optional[int]) -> list[Path]:
    all_dirs = sorted(p for p in dataset_dir.iterdir() if p.is_dir())
    if systems:
        wanted = [name.strip() for name in systems.split(",") if name.strip()]
        by_name = {p.name: p for p in all_dirs}
        missing = [name for name in wanted if name not in by_name]
        if missing:
            raise FileNotFoundError(f"Unknown SimBench system(s): {', '.join(missing)}")
        all_dirs = [by_name[name] for name in wanted]
    if limit is not None:
        all_dirs = all_dirs[:limit]
    return all_dirs


def build_extra_body(chat_template_kwargs_json: Optional[str]) -> Optional[dict]:
    if not chat_template_kwargs_json:
        return None
    try:
        chat_template_kwargs = json.loads(chat_template_kwargs_json)
    except json.JSONDecodeError as exc:
        raise ValueError(f"Invalid --chat-template-kwargs-json: {exc}") from exc
    if not isinstance(chat_template_kwargs, dict):
        raise ValueError("--chat-template-kwargs-json must decode to a JSON object")
    return {"chat_template_kwargs": chat_template_kwargs}


def safe_chat_create(
    client: OpenAI,
    messages: list[dict],
    model: str,
    *,
    retries: int,
    retry_sleep: float,
    temperature: float,
    top_p: float,
    max_tokens: int,
    extra_body: Optional[dict],
) -> str:
    last_err = None
    for attempt in range(1, retries + 1):
        try:
            kwargs = {
                "model": model,
                "messages": messages,
                "temperature": temperature,
                "top_p": top_p,
                "max_tokens": max_tokens,
                "stream": False,
            }
            if extra_body:
                kwargs["extra_body"] = extra_body
            resp = client.chat.completions.create(**kwargs)
            content = resp.choices[0].message.content or ""
            if content.strip():
                return content
            last_err = RuntimeError("empty response content")
            if attempt < retries:
                time.sleep(retry_sleep)
        except Exception as exc:
            last_err = exc
            if attempt < retries:
                time.sleep(retry_sleep)
    return f"[ERROR AFTER {retries} RETRIES] {last_err}"


def generate_first_code(first_prompt: str, model_id: str, args: argparse.Namespace, extra_body: Optional[dict]) -> Tuple[str, str]:
    prompt = f"""You are a PyChrono expert tasked with generating a simulation script based on the following instructions. Make sure to:
1. Initialize the PyChrono environment and core components.
2. Add the required physical systems and objects as specified.
3. Set necessary default parameters such as positions, forces, and interactions.

Instructions:
{first_prompt}

Output only the Python simulation script.
"""
    client = OpenAI(base_url=args.base_url, api_key=args.api_key)
    answer = safe_chat_create(
        client,
        [{"role": "user", "content": prompt}],
        model_id,
        retries=args.retries,
        retry_sleep=args.retry_sleep,
        temperature=args.temperature,
        top_p=args.top_p,
        max_tokens=args.max_tokens,
        extra_body=extra_body,
    )
    return answer, prompt


def generate_second_third_code(
    prompt_text: str,
    code: str,
    model_id: str,
    args: argparse.Namespace,
    extra_body: Optional[dict],
) -> Tuple[str, str]:
    prompt = f"""You are a PyChrono expert tasked with generating a simulation script based on the following instructions and a given PyChrono script, which may contain errors.

Tasks:
1. Identify and correct any errors, including syntax errors, logical errors, incorrect method names, and parameter issues.
2. Modify the script based on the provided instructions.

Given PyChrono code:
{code}

Instructions to apply:
{prompt_text}

Output only the corrected and modified Python simulation script.
"""
    client = OpenAI(base_url=args.base_url, api_key=args.api_key)
    answer = safe_chat_create(
        client,
        [{"role": "user", "content": prompt}],
        model_id,
        retries=args.retries,
        retry_sleep=args.retry_sleep,
        temperature=args.temperature,
        top_p=args.top_p,
        max_tokens=args.max_tokens,
        extra_body=extra_body,
    )
    return answer, prompt


def save_conversation_json(
    output_conversation_path: Path,
    combined_prompt1: str,
    first_response: str,
    combined_prompt2: str,
    second_response: str,
    combined_prompt3: str,
    third_response: str,
) -> None:
    conversation_data = [
        {
            "instruction": combined_prompt3,
            "input": "",
            "output": third_response,
            "system": "You are a PyChrono expert tasked with generating a simulation script based on the following instructions.",
            "history": [
                [combined_prompt1, first_response],
                [combined_prompt2, second_response],
            ],
        }
    ]
    output_conversation_path.parent.mkdir(parents=True, exist_ok=True)
    output_conversation_path.write_text(json.dumps(conversation_data, indent=4), encoding="utf-8")


def response_files(output_system_path: Path) -> Iterable[Path]:
    yield output_system_path / "first_response.txt"
    yield output_system_path / "second_response.txt"
    yield output_system_path / "third_response.txt"


def is_valid_response(path: Path) -> bool:
    if not path.exists() or path.stat().st_size == 0:
        return False
    return "[ERROR AFTER" not in path.read_text(encoding="utf-8", errors="replace")


def process_system(system_folder: Path, model_name: str, model_id: str, args: argparse.Namespace, extra_body: Optional[dict]) -> str:
    output_system_path = args.output_dir / model_name / system_folder.name
    output_system_path.mkdir(exist_ok=True, parents=True)

    if args.skip_existing and all(is_valid_response(path) for path in response_files(output_system_path)):
        return f"{system_folder.name}: skipped"

    first_response, combined_prompt1 = generate_first_code(
        read_script(system_folder / "input1.txt"), model_id, args, extra_body
    )
    (output_system_path / "first_response.txt").write_text(first_response, encoding="utf-8")

    second_response, combined_prompt2 = generate_second_third_code(
        read_script(system_folder / "input2.txt"),
        read_script(system_folder / "pyinput2.py"),
        model_id,
        args,
        extra_body,
    )
    (output_system_path / "second_response.txt").write_text(second_response, encoding="utf-8")

    third_response, combined_prompt3 = generate_second_third_code(
        read_script(system_folder / "input3.txt"),
        read_script(system_folder / "pyinput3.py"),
        model_id,
        args,
        extra_body,
    )
    (output_system_path / "third_response.txt").write_text(third_response, encoding="utf-8")

    conv_path = args.conversation_dir / f"{model_name}_{system_folder.name}_conversation.json"
    save_conversation_json(
        conv_path,
        combined_prompt1,
        first_response,
        combined_prompt2,
        second_response,
        combined_prompt3,
        third_response,
    )
    return f"{system_folder.name}: ok"


def main() -> None:
    args = parse_args()
    model_id = args.model_id or args.model_name
    extra_body = build_extra_body(args.chat_template_kwargs_json)

    args.output_dir.mkdir(exist_ok=True, parents=True)
    args.conversation_dir.mkdir(exist_ok=True, parents=True)
    system_dirs = selected_system_dirs(args.dataset_dir, args.systems, args.limit)

    print(f"Project root: {PROJECT_ROOT}")
    print(f"Dataset: {args.dataset_dir}")
    print(f"Model name: {args.model_name}")
    print(f"Model id: {model_id}")
    print(f"Endpoint: {args.base_url}")
    print(f"Systems: {len(system_dirs)}")
    print(f"Max workers: {args.max_workers}")

    failures = []
    with ThreadPoolExecutor(max_workers=args.max_workers) as executor:
        futures = {
            executor.submit(process_system, system_dir, args.model_name, model_id, args, extra_body): system_dir
            for system_dir in system_dirs
        }
        for future in tqdm(as_completed(futures), total=len(futures)):
            system_dir = futures[future]
            try:
                print(future.result())
            except Exception as exc:
                failures.append((system_dir.name, str(exc)))
                print(f"{system_dir.name}: fail: {exc}")

    if failures:
        print("Failures:")
        for name, message in failures:
            print(f"- {name}: {message}")
        raise SystemExit(1)

    print("finished")


if __name__ == "__main__":
    main()
