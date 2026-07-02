import argparse
import logging
import re
from pathlib import Path

from tqdm import tqdm


SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Extract Python files from SimBench response text outputs.")
    parser.add_argument("--model-name", required=True)
    parser.add_argument("--output-dir", type=Path, default=PROJECT_ROOT / "output_llms")
    parser.add_argument("--systems", default=None, help="Comma-separated system names. Defaults to all model outputs.")
    parser.add_argument("--log-file", type=Path, default=PROJECT_ROOT / "extraction.log")
    return parser.parse_args()


def remove_comments(code: str) -> str:
    code = re.sub(r"#.*", "", code)
    code = re.sub(r"(\"\"\"[\s\S]*?\"\"\"|'''[\s\S]*?''')", "", code)
    return code.strip()


def extract_python_code(content: str) -> str:
    multiple_matches = re.findall(r"```python(.*?)```", content, re.DOTALL)
    if multiple_matches:
        return "\n\n".join(match.strip() for match in multiple_matches)

    start_match = re.search(r"```python", content)
    if start_match:
        end_match = re.search(r"```", content[start_match.end() :])
        if end_match:
            return content[start_match.end() : start_match.end() + end_match.start()].strip()
        return content[start_match.end() :].strip() + '\nprint("error happened with only start ```python")'

    generic_matches = re.findall(r"```(.*?)```", content, re.DOTALL)
    if generic_matches:
        return "\n\n".join(match.strip() for match in generic_matches)

    return content.strip()


def selected_system_dirs(model_dir: Path, systems: str | None) -> list[Path]:
    all_dirs = sorted(p for p in model_dir.iterdir() if p.is_dir())
    if not systems:
        return all_dirs
    wanted = [name.strip() for name in systems.split(",") if name.strip()]
    by_name = {p.name: p for p in all_dirs}
    missing = [name for name in wanted if name not in by_name]
    if missing:
        raise FileNotFoundError(f"Missing generated output for system(s): {', '.join(missing)}")
    return [by_name[name] for name in wanted]


def process_response(system_dir: Path, round_name: str) -> str:
    response_path = system_dir / f"{round_name}_response.txt"
    output_py_path = system_dir / f"{round_name}_response.py"
    cleaned_path = system_dir / f"{round_name}_cleaned_response.py"

    if not response_path.exists():
        return f"{response_path} not found"

    code = extract_python_code(response_path.read_text(encoding="utf-8"))
    output_py_path.write_text(code, encoding="utf-8")
    cleaned_path.write_text(remove_comments(code), encoding="utf-8")
    return f"{cleaned_path} success"


def main() -> None:
    args = parse_args()
    logging.basicConfig(
        filename=args.log_file,
        level=logging.INFO,
        format="%(asctime)s - %(levelname)s - %(message)s",
    )

    model_dir = args.output_dir / args.model_name
    if not model_dir.exists():
        raise FileNotFoundError(f"Model output directory not found: {model_dir}")

    messages = []
    for system_dir in tqdm(selected_system_dirs(model_dir, args.systems)):
        system_messages = [process_response(system_dir, round_name) for round_name in ("first", "second", "third")]
        messages.extend(system_messages)
        (system_dir / "extraction_message.txt").write_text("\n".join(system_messages) + "\n", encoding="utf-8")

    for message in messages:
        logging.info(message)
    print("finished")


if __name__ == "__main__":
    main()
