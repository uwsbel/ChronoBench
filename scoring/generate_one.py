"""Generate ONE candidate script for a single task dir (turn 1), then extract runnable Python.

The thin end-to-end companion to ``scoring/judge_v2.py``: judge_v2 grades a candidate, this makes
one. It exists because ``chronobench.generate.generate_system`` is coupled to the full 3-turn protocol
(it reads ``input{1,2,3}.txt`` + ``pyinput{2,3}.py``), so it cannot run on a turn-1-only task in the
redesigned ``demo_data_10/`` suite yet. This driver runs just turn 1, reusing the same wrapper prompt
(``chronobench.generate.build_turn1``) and provider adapters (``make_caller``) so the framing matches
production generation.

    conda run -n chronobench python scoring/generate_one.py demo_data_10/pendulum \
        --provider openai --model gpt-4o --out-dir runs/pilot_pendulum/gpt-4o

Writes ``<out-dir>/first_response.txt`` (raw model output) and ``<out-dir>/candidate.py`` (extracted
code, ready for ``judge_v2.py <task_dir> <out-dir>/candidate.py``). Needs the provider's API key in the
environment (``OPENAI_API_KEY`` etc.); it is read via the env, never logged.
"""
from __future__ import annotations

import argparse
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
for p in (PROJECT_ROOT, SCRIPT_DIR):
    if p not in sys.path:
        sys.path.insert(0, p)

from chronobench.generate import build_turn1, make_caller  # noqa: E402
from extractPy import extract_python_code  # noqa: E402


def generate_one(task_dir, out_dir, provider="openai", model="gpt-4o",
                 base_url=None, api_key_env=None, turn=1):
    """Run turn `turn` of the S-LLM protocol on one task dir; return (txt_path, py_path)."""
    if turn != 1:
        raise NotImplementedError("generate_one currently supports turn 1 only "
                                  "(multi-turn staging is not authored for demo_data_10 yet).")
    instructions = open(os.path.join(task_dir, f"input{turn}.txt"), encoding="utf-8").read()
    prompt = build_turn1(instructions)
    call = make_caller(provider, model, base_url=base_url, api_key_env=api_key_env)
    text = call(prompt)

    os.makedirs(out_dir, exist_ok=True)
    txt_path = os.path.join(out_dir, "first_response.txt")
    py_path = os.path.join(out_dir, "candidate.py")
    with open(txt_path, "w", encoding="utf-8") as f:
        f.write(text)
    extract_python_code(txt_path, py_path, log_file=os.path.join(out_dir, "extract.log"))
    return txt_path, py_path


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python scoring/generate_one.py", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("task_dir", help="path to a task dir containing input1.txt (e.g. demo_data_10/pendulum)")
    ap.add_argument("--provider", default="openai", choices=["openai", "anthropic", "google"])
    ap.add_argument("--model", default="gpt-4o", help="provider model id")
    ap.add_argument("--base-url", default=None, help="OpenAI-compatible base_url (NIM/vLLM/...)")
    ap.add_argument("--api-key-env", default=None, help="env var holding the API key (default per provider)")
    ap.add_argument("--out-dir", default=None, help="output dir (default runs/<task>/<model>)")
    args = ap.parse_args(argv)

    task = os.path.basename(os.path.normpath(args.task_dir))
    out_dir = args.out_dir or os.path.join(PROJECT_ROOT, "runs", task, args.model.replace("/", "_"))
    print(f"generate_one: task={task} provider={args.provider} model={args.model} -> {out_dir}")
    txt_path, py_path = generate_one(args.task_dir, out_dir, provider=args.provider, model=args.model,
                                     base_url=args.base_url, api_key_env=args.api_key_env)
    print(f"  raw:       {txt_path}")
    print(f"  candidate: {py_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
