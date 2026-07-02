"""Score the redesigned (v2 / PyChrono 10.0) task suite end-to-end with the de-scoped judge.

Main-package CLI companion to ``scoring/judge_v2.py``. It iterates every task in a suite directory
(default ``demo_data_10/``) and, for every turn, runs ``judge_v2.judge`` (L1 execution gate + L2
capability checks + L3 derived behavioral invariants + tunable scoring), then prints a summary table and
an aggregate pass count.

  - No ``--candidates``  : SELF-CHECK the references (``run.entry`` of each turn). Every turn should be 100.
  - ``--candidates DIR`` : score candidate scripts at ``DIR/<task>/turn{N}.py`` (missing ones are skipped).

Because ``judge_v2`` executes candidate scripts via ``sys.executable``, run this UNDER the pinned
``pychrono10`` env so candidates run on PyChrono 10.0:

    conda run -n pychrono10 python -m chronobench.score_v2                      # self-check references
    conda run -n pychrono10 python -m chronobench.score_v2 --task beam          # one task
    conda run -n pychrono10 python -m chronobench.score_v2 --candidates runs/my-agent --json runs/my-agent/score.json
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

_PKG = Path(__file__).resolve().parent
PROJECT_ROOT = _PKG.parent
_SCORING = PROJECT_ROOT / "scoring"
for _p in (str(PROJECT_ROOT), str(_SCORING)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import judge_v2  # noqa: E402  (lives in scoring/, executes candidates via sys.executable)


def turn_numbers(contract):
    turns = contract.get("turns")
    if turns:
        return [t.get("turn", i + 1) for i, t in enumerate(turns)]
    return [1]


def discover_tasks(suite_dir):
    return sorted(name for name in os.listdir(suite_dir)
                  if os.path.isfile(os.path.join(suite_dir, name, "contract.json")))


def score_suite(suite_dir, candidates=None, only_task=None):
    tasks = [only_task] if only_task else discover_tasks(suite_dir)
    results = []
    for task in tasks:
        tdir = os.path.join(suite_dir, task)
        contract = json.load(open(os.path.join(tdir, "contract.json"), encoding="utf-8"))
        for turn in turn_numbers(contract):
            cand = None
            if candidates:
                cand = os.path.join(candidates, task, f"turn{turn}.py")
                if not os.path.exists(cand):
                    results.append({"task": task, "turn": turn, "score": None, "triage": "no-candidate"})
                    continue
            try:
                v = judge_v2.judge(tdir, cand, turn=turn)
                results.append({"task": task, "turn": turn, "score": v["score"],
                                "triage": v["triage"], "layers": v.get("layers", {})})
            except Exception as exc:   # surface, do not abort the sweep
                results.append({"task": task, "turn": turn, "score": None, "triage": f"judge-error: {exc}"})
    return results


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python -m chronobench.score_v2", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--suite", default=str(PROJECT_ROOT / "demo_data_10"), help="task suite dir (default demo_data_10/)")
    ap.add_argument("--candidates", default=None, help="dir with <task>/turn{N}.py (default: self-check the references)")
    ap.add_argument("--task", default=None, help="score only this one task")
    ap.add_argument("--json", default=None, help="also write the full results (incl. layers) to this JSON file")
    args = ap.parse_args(argv)

    mode = f"candidates: {args.candidates}" if args.candidates else "self-check references"
    print(f"ChronoBench v2 suite | {args.suite} | {mode}\n")
    results = score_suite(args.suite, candidates=args.candidates, only_task=args.task)

    print(f"{'task':<24}{'turn':>5}{'score':>8}   triage")
    print("-" * 60)
    scored = passed = 0
    for r in results:
        s = "  -  " if r["score"] is None else f"{r['score']:6.1f}"
        print(f"{r['task']:<24}{r['turn']:>5}{s:>8}   {r['triage']}")
        if r["score"] is not None:
            scored += 1
            passed += (r["triage"] == "pass")
    print("-" * 60)
    print(f"{passed}/{scored} turns pass ({mode})")

    if args.json:
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump(results, f, indent=2)
        print(f"wrote {args.json}")
    # exit nonzero if any scored turn did not pass (useful for CI / self-check)
    return 0 if scored and passed == scored else 1


if __name__ == "__main__":
    raise SystemExit(main())
