"""Evaluate ONE agent's virtual experiment scripts against the SimBench benchmark.

This is the "evaluate your own agent" entry point. Point it at a directory holding your
agent's generated code in the standard layout:

    <responses-dir>/<model>/<system>/{first,second,third}_response.py

(the same layout the published runs use under ``output_llms/``), and it scores every
(system, turn) with the rule-based J-LLM, writes a CSV in the same schema as
``combined_evaluation_scores.csv`` (so ``scoring/rank_llm.py`` can consume it), and prints a
summary by rubric mode, category, and turn.

Examples
--------
    # score the published outputs of one model (judge = $SIMBENCH_JUDGE_MODEL or gpt-4o-mini)
    python -m simbench.score claude-4-sonnet-20250514

    # score your own agent living in a custom directory, only the FEA systems
    python -m simbench.score my-agent --responses-dir /path/to/outputs --systems beam,cable,rotor

    # check your files/prompts without spending any API calls
    python -m simbench.score my-agent --dry-run

Requires ``$OPENAI_API_KEY`` (unless ``--dry-run``). Use ``--base-url`` to point the
OpenAI-compatible client at another provider (NVIDIA NIM, Together, a local vLLM, etc.).
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
from concurrent.futures import ThreadPoolExecutor, as_completed

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from simbench.judge import DEFAULT_MODEL, build_prompt, evaluate_script, select_mode  # noqa: E402
from simbench.systems import all_systems, category_of  # noqa: E402

ROUNDS = [("first", 1), ("second", 2), ("third", 3)]
# CSV columns match combined_evaluation_scores.csv so rank_llm.py is drop-in.
CSV_HEADER = ["Test Model", "System", "Round", "Score Document", "Score Reference",
              "Score Reference Document"]
MODE_TO_COL = {"doc": 3, "ref": 4, "ref_doc": 5}


def _read(path: str) -> str | None:
    if not os.path.exists(path):
        return None
    with open(path, "r", encoding="utf-8") as f:
        return f.read()


def score_one(model: str, system: str, responses_dir: str, data_dir: str, api_doc: str,
              modes: list[str], judge_model: str, client, dry_run: bool) -> list[dict]:
    """Score all three turns of one system. Returns a list of per-turn result dicts."""
    sys_out = os.path.join(responses_dir, model, system)
    sys_data = os.path.join(data_dir, system)
    rows = []
    for round_name, t in ROUNDS:
        candidate = _read(os.path.join(sys_out, f"{round_name}_response.py"))
        reference = _read(os.path.join(sys_data, f"truth{t}.py"))
        if candidate is None:
            print(f"  [skip] {model}/{system}/{round_name}: no {round_name}_response.py")
            continue
        row = {"Test Model": model, "System": system, "Round": round_name,
               "category": category_of(system)}
        for mode in modes:
            ref = reference if mode in ("ref", "ref_doc") else None
            doc = api_doc if mode in ("doc", "ref_doc") else None
            if dry_run:
                # validate that context exists and the prompt renders; no API call.
                try:
                    build_prompt(mode, candidate, ref, doc)
                    row[mode] = "OK"
                except ValueError as e:
                    row[mode] = f"ERR:{e}"
            else:
                ev = evaluate_script(candidate, reference=ref, api_doc=doc, mode=mode,
                                 model=judge_model, client=client)
                row[mode] = ev.score
        rows.append(row)
    return rows


def summarize(rows: list[dict], modes: list[str]) -> None:
    scored = [r for r in rows if any(isinstance(r.get(m), int) for m in modes)]
    if not scored:
        print("\nNo numeric scores to summarize (dry-run or all failed).")
        return
    print(f"\n=== Summary over {len(scored)} turn-level evaluations ===")
    for mode in modes:
        vals = [r[mode] for r in scored if isinstance(r.get(mode), int)]
        if vals:
            print(f"  mode {mode:8s}: mean={sum(vals)/len(vals):.1f}  n={len(vals)}")
    # by category and by turn, using the richest selected mode
    primary = "ref_doc" if "ref_doc" in modes else modes[0]
    by_cat: dict[str, list[int]] = {}
    by_turn: dict[str, list[int]] = {}
    for r in scored:
        v = r.get(primary)
        if isinstance(v, int):
            by_cat.setdefault(r["category"], []).append(v)
            by_turn.setdefault(r["Round"], []).append(v)
    print(f"  -- by category ({primary}) --")
    for cat in sorted(by_cat):
        vs = by_cat[cat]
        print(f"     {cat:4s}: mean={sum(vs)/len(vs):.1f}  n={len(vs)}")
    print(f"  -- by turn ({primary}) --")
    for rn, _ in ROUNDS:
        if rn in by_turn:
            vs = by_turn[rn]
            print(f"     {rn:7s}: mean={sum(vs)/len(vs):.1f}  n={len(vs)}")


def write_csv(rows: list[dict], out_path: str, modes: list[str]) -> None:
    with open(out_path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(CSV_HEADER)
        for r in rows:
            line = [r["Test Model"], r["System"], r["Round"], "", "", ""]
            for mode in modes:
                line[MODE_TO_COL[mode]] = r.get(mode, "")
            w.writerow(line)
    print(f"\nWrote {out_path} ({len(rows)} rows)")


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="python -m simbench.score", description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("model", help="model/agent name; outputs at <responses-dir>/<model>/<system>/")
    p.add_argument("--responses-dir", default=os.path.join(PROJECT_ROOT, "output_llms"),
                   help="base dir containing <model>/<system>/ outputs (default: output_llms/)")
    p.add_argument("--data-dir", default=os.path.join(PROJECT_ROOT, "demo_data"),
                   help="benchmark data dir with <system>/truth{1,2,3}.py (default: demo_data/)")
    p.add_argument("--api", default=os.path.join(PROJECT_ROOT, "api", "api.txt"),
                   help="API documentation text file (default: api/api.txt)")
    p.add_argument("--systems", default="", help="comma-separated subset (default: all 34)")
    p.add_argument("--modes", default="doc,ref,ref_doc",
                   help="comma-separated rubric modes to run (default: doc,ref,ref_doc)")
    p.add_argument("--judge-model", default=DEFAULT_MODEL,
                   help=f"judge model (default: {DEFAULT_MODEL}; or $SIMBENCH_JUDGE_MODEL)")
    p.add_argument("--base-url", default=None,
                   help="OpenAI-compatible base_url for a non-OpenAI provider")
    p.add_argument("--out", default=None,
                   help="output CSV path (default: <responses-dir>/<model>/evaluation_scores_simbench.csv)")
    p.add_argument("--max-workers", type=int, default=16)
    p.add_argument("--dry-run", action="store_true",
                   help="validate files and render prompts without calling the judge")
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    systems = [s.strip() for s in args.systems.split(",") if s.strip()] or all_systems()
    modes = [m.strip() for m in args.modes.split(",") if m.strip()]
    bad = [m for m in modes if m not in MODE_TO_COL]
    if bad:
        print(f"Unknown mode(s): {bad}; valid: {sorted(MODE_TO_COL)}")
        return 2

    api_doc = _read(args.api)
    if api_doc is None and any(m in ("doc", "ref_doc") for m in modes):
        print(f"API doc not found at {args.api} (needed for doc/ref_doc modes).")
        return 2

    client = None
    if not args.dry_run:
        from openai import OpenAI
        key = os.getenv("OPENAI_API_KEY")
        client = OpenAI(api_key=key, base_url=args.base_url) if args.base_url else OpenAI(api_key=key)

    print(f"Evaluating '{args.model}' | judge={args.judge_model} | modes={modes} "
          f"| systems={len(systems)} | dry_run={args.dry_run}")

    all_rows: list[dict] = []
    with ThreadPoolExecutor(max_workers=args.max_workers) as ex:
        futs = {ex.submit(score_one, args.model, s, args.responses_dir, args.data_dir,
                          api_doc, modes, args.judge_model, client, args.dry_run): s
                for s in systems}
        for fut in as_completed(futs):
            all_rows.extend(fut.result())

    all_rows.sort(key=lambda r: (r["System"], r["Round"]))
    out_path = args.out or os.path.join(args.responses_dir, args.model,
                                        "evaluation_scores_simbench.csv")
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    write_csv(all_rows, out_path, modes)
    summarize(all_rows, modes)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
