"""Generate an agent's virtual experiment scripts over the benchmark (the "generate" half).

Companion to ``chronobench.score`` (the "judge" half). For each system, this runs the 3-turn
protocol against a model and writes the exact layout the rest of the pipeline expects:

    <responses-dir>/<label>/<system>/{first,second,third}_response.txt
    <responses-dir>/<label>/<system>/conversation.json   (Alpaca-style record of the 3 turns)

Turn 1 creates a script from ``input1.txt``; Turns 2-3 fix-and-modify the provided
``pyinput{2,3}.py`` per ``input{2,3}.txt``. The two wrapper prompts live in
``chronobench/prompts/`` (versioned, like the rubric). This single parametrized generator replaces
the former per-provider scripts under ``scoring/engine/``.

Providers (lazy-imported, so importing this module needs no SDK):
    - ``openai``  (default): any OpenAI-compatible endpoint via ``--base-url`` (OpenAI, NVIDIA NIM,
      DeepInfra, Together, a local vLLM, etc.).
    - ``anthropic`` : Claude (``messages.create``).
    - ``google``    : Gemini (``generativeai``).

Example
-------
    # generate baseline outputs for a published model, then score them
    python -m chronobench.generate gpt-4o --provider openai --model gpt-4o
    python -m chronobench.score gpt-4o

    # a new agent on a local vLLM, into runs/ (kept out of the frozen run)
    python -m chronobench.generate my-agent --base-url http://localhost:8000/v1 \
        --model my-model --api-key-env VLLM_KEY --responses-dir runs
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

_PKG = Path(__file__).resolve().parent
PROJECT_ROOT = _PKG.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from chronobench.systems import all_systems  # noqa: E402

_PROMPTS = _PKG / "prompts"
ROUNDS = [("first", 1), ("second", 2), ("third", 3)]

# Generation sampling defaults (match the published generation drivers; judging uses different
# defaults, see chronobench.judge).
DEFAULT_TEMPERATURE = 0.1
DEFAULT_TOP_P = 0.95
DEFAULT_MAX_TOKENS = 16384


def build_turn1(instructions: str) -> str:
    return (_PROMPTS / "s_llm_turn1.txt").read_text(encoding="utf-8").format(instructions=instructions)


def build_turn23(instructions: str, code: str) -> str:
    return (_PROMPTS / "s_llm_turn23.txt").read_text(encoding="utf-8").format(
        instructions=instructions, code=code)


# --- provider adapters: make_caller returns a function prompt(str) -> text(str) ---------------
def _openai_caller(model, base_url, api_key_env, temperature, top_p, max_tokens):
    from openai import OpenAI
    key = os.getenv(api_key_env)
    client = OpenAI(api_key=key, base_url=base_url) if base_url else OpenAI(api_key=key)

    def call(prompt):
        c = client.chat.completions.create(
            model=model, messages=[{"role": "user", "content": prompt}],
            temperature=temperature, top_p=top_p, max_tokens=max_tokens, stream=False)
        return c.choices[0].message.content
    return call


def _anthropic_caller(model, api_key_env, temperature, top_p, max_tokens):
    import anthropic
    client = anthropic.Anthropic(api_key=os.getenv(api_key_env))

    def call(prompt):
        m = client.messages.create(
            model=model, max_tokens=max_tokens, temperature=temperature,
            messages=[{"role": "user", "content": prompt}])
        return "".join(b.text for b in m.content if getattr(b, "type", None) == "text")
    return call


def _google_caller(model, api_key_env, temperature, top_p, max_tokens):
    import google.generativeai as genai
    genai.configure(api_key=os.getenv(api_key_env))
    gm = genai.GenerativeModel(model, generation_config={
        "temperature": temperature, "top_p": top_p, "max_output_tokens": max_tokens})

    def call(prompt):
        return gm.generate_content(prompt).text
    return call


_DEFAULT_KEY_ENV = {"openai": "OPENAI_API_KEY", "anthropic": "ANTHROPIC_API_KEY", "google": "GEMINI_API_KEY"}


def make_caller(provider, model, *, base_url=None, api_key_env=None,
                temperature=DEFAULT_TEMPERATURE, top_p=DEFAULT_TOP_P, max_tokens=DEFAULT_MAX_TOKENS):
    """Return a prompt->text callable for the chosen provider."""
    key_env = api_key_env or _DEFAULT_KEY_ENV.get(provider)
    if provider == "openai":
        return _openai_caller(model, base_url, key_env, temperature, top_p, max_tokens)
    if provider == "anthropic":
        return _anthropic_caller(model, key_env, temperature, top_p, max_tokens)
    if provider == "google":
        return _google_caller(model, key_env, temperature, top_p, max_tokens)
    raise ValueError(f"Unknown provider {provider!r}; expected openai|anthropic|google.")


def _read(path):
    with open(path, "r", encoding="utf-8") as f:
        return f.read()


def generate_system(label, system, data_dir, responses_dir, call, dry_run):
    """Run the 3 turns for one system; write the response txts + a conversation JSON."""
    sys_data = os.path.join(data_dir, system)
    out_dir = os.path.join(responses_dir, label, system)
    if not dry_run:
        os.makedirs(out_dir, exist_ok=True)

    prompts, responses = {}, {}
    for round_name, t in ROUNDS:
        instructions = _read(os.path.join(sys_data, f"input{t}.txt"))
        if t == 1:
            prompt = build_turn1(instructions)
        else:
            code = _read(os.path.join(sys_data, f"pyinput{t}.py"))
            prompt = build_turn23(instructions, code)
        prompts[round_name] = prompt
        if dry_run:
            responses[round_name] = "DRY_RUN"
            continue
        try:
            text = call(prompt)
        except Exception as exc:  # surface, do not abort the batch
            text = f"ERROR: {exc}"
        responses[round_name] = text
        with open(os.path.join(out_dir, f"{round_name}_response.txt"), "w", encoding="utf-8") as f:
            f.write(text)

    if not dry_run:
        conv = [{
            "instruction": prompts["third"], "input": "", "output": responses["third"],
            "system": "You are a PyChrono expert tasked with generating a simulation script based on the following instructions.",
            "history": [[prompts["first"], responses["first"]], [prompts["second"], responses["second"]]],
        }]
        with open(os.path.join(out_dir, "conversation.json"), "w", encoding="utf-8") as f:
            json.dump(conv, f, indent=4, ensure_ascii=False)
    return f"{label}/{system}: {'prompts built' if dry_run else 'generated'}"


def build_parser():
    p = argparse.ArgumentParser(prog="python -m chronobench.generate", description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("label", help="output folder name under <responses-dir> (e.g. the agent/model name)")
    p.add_argument("--model", default=None, help="provider model id (default: same as label)")
    p.add_argument("--provider", default="openai", choices=["openai", "anthropic", "google"])
    p.add_argument("--base-url", default=None, help="OpenAI-compatible base_url (NIM/DeepInfra/vLLM/...)")
    p.add_argument("--api-key-env", default=None, help="env var holding the API key (default per provider)")
    p.add_argument("--responses-dir", default=os.path.join(PROJECT_ROOT, "output_llms"),
                   help="base output dir (default output_llms/; use runs/ for new agents)")
    p.add_argument("--data-dir", default=os.path.join(PROJECT_ROOT, "demo_data"))
    p.add_argument("--systems", default="", help="comma-separated subset (default: all 34)")
    p.add_argument("--temperature", type=float, default=DEFAULT_TEMPERATURE)
    p.add_argument("--top-p", type=float, default=DEFAULT_TOP_P)
    p.add_argument("--max-tokens", type=int, default=DEFAULT_MAX_TOKENS)
    p.add_argument("--max-workers", type=int, default=8)
    p.add_argument("--dry-run", action="store_true", help="build prompts + lay out dirs; no API calls")
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    systems = [s.strip() for s in args.systems.split(",") if s.strip()] or all_systems()
    model = args.model or args.label

    call = None
    if not args.dry_run:
        call = make_caller(args.provider, model, base_url=args.base_url, api_key_env=args.api_key_env,
                           temperature=args.temperature, top_p=args.top_p, max_tokens=args.max_tokens)

    print(f"Generating '{args.label}' | provider={args.provider} model={model} "
          f"| systems={len(systems)} | dry_run={args.dry_run}")
    with ThreadPoolExecutor(max_workers=args.max_workers) as ex:
        futs = [ex.submit(generate_system, args.label, s, args.data_dir, args.responses_dir,
                          call, args.dry_run) for s in systems]
        for fut in as_completed(futs):
            print(" ", fut.result())
    print("finished")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
