"""Generate ``demo_data/manifest.json``: a machine-readable index of the benchmark.

Scans the ``demo_data`` folders, assigns each system its category from the canonical
``simbench.systems`` map, records the per-turn file roles, and validates that every system
has the expected files. Run from anywhere:

    python scoring/generate_manifest.py

The manifest lets a tool (or an agent harness) discover the benchmark without parsing the
research scripts: which systems exist, their category, and which file plays which role per turn.
"""

from __future__ import annotations

import json
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
sys.path.insert(0, PROJECT_ROOT)

from simbench.systems import CATEGORIES, SYSTEMS, TURNS, category_of  # noqa: E402

DEMO_DATA = os.path.join(PROJECT_ROOT, "demo_data")

# role -> filename template, where {t} is the turn number (1/2/3)
TURN_FILES = {
    "prompt": "input{t}.txt",          # natural-language request given to the agent
    "reference": "truth{t}.py",        # expert ground-truth virtual experiment script
    "reference_cleaned": "cleaned_truth{t}.py",  # comments stripped, for similarity metrics
    "conversation": "output{t}.json",  # Alpaca-style instruction/input/output record
}
# starter code handed to the agent on the modify/extend turns (no turn 1)
STARTER_FILES = {2: "pyinput2.py", 3: "pyinput3.py"}


def build_manifest() -> dict:
    systems: dict[str, dict] = {}
    missing: list[str] = []

    for system in SYSTEMS:
        sys_dir = os.path.join(DEMO_DATA, system)
        turns: dict[str, dict] = {}
        for t in TURNS:
            files = {role: tmpl.format(t=t) for role, tmpl in TURN_FILES.items()}
            if t in STARTER_FILES:
                files["starter_code"] = STARTER_FILES[t]
            for fname in files.values():
                if not os.path.exists(os.path.join(sys_dir, fname)):
                    missing.append(f"{system}/{fname}")
            turns[str(t)] = files
        systems[system] = {"category": category_of(system), "turns": turns}

    if missing:
        print(f"WARNING: {len(missing)} expected files missing, e.g. {missing[:5]}")

    return {
        "description": (
            "SimBench benchmark index. 34 physical systems x 3 turns = 102 turn-level tasks. "
            "Turn 1 creates a virtual experiment script from a prompt; Turns 2-3 modify/extend it. The "
            "expert reference (truth{t}.py) and api/api.txt are the J-LLM's grounding context."
        ),
        "n_systems": len(SYSTEMS),
        "n_turns": len(TURNS),
        "n_tasks": len(SYSTEMS) * len(TURNS),
        "categories": {
            cat: {"name": meta["name"], "systems": meta["systems"]}
            for cat, meta in CATEGORIES.items()
        },
        "file_roles": {
            "prompt": "natural-language request given to the agent for this turn",
            "reference": "expert-authored ground-truth PyChrono virtual experiment script",
            "reference_cleaned": "reference with comments removed (for CodeBLEU/ROUGE)",
            "conversation": "Alpaca-style {instruction,input,output} record of the turn",
            "starter_code": "existing code handed to the agent to modify (turns 2-3 only)",
        },
        "systems": systems,
    }


def main() -> None:
    manifest = build_manifest()
    out_path = os.path.join(DEMO_DATA, "manifest.json")
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(manifest, f, indent=2)
        f.write("\n")
    print(f"Wrote {out_path}: {manifest['n_systems']} systems, {manifest['n_tasks']} tasks")


if __name__ == "__main__":
    main()
