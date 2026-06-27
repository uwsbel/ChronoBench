"""Example: a generate -> evaluate -> revise loop using ChronoBench.

Drives an agent against ONE benchmark system with the rule-based judge in the loop: the agent
drafts a virtual experiment script, the J-LLM scores it AND explains the deductions, and the agent
revises using that feedback. This is the diagnostic-feedback use the paper shows matters most.

For illustration this uses an OpenAI-compatible model for BOTH the agent and the judge; in practice
the agent is YOUR system under test and the judge is the contract's. Requires $OPENAI_API_KEY.

    python examples/evaluate_agent_loop.py --system pendulum --rounds 2
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from chronobench.contract import load_contract       # noqa: E402
from chronobench.generate import build_turn1, make_caller  # noqa: E402
from chronobench.judge import evaluate_script         # noqa: E402


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--system", default="pendulum", help="a benchmark system (see demo_data/manifest.json)")
    ap.add_argument("--rounds", type=int, default=2, help="max generate/evaluate/revise iterations")
    ap.add_argument("--agent-model", default="gpt-4o-mini", help="the model under test (OpenAI-compatible)")
    ap.add_argument("--contract", default="v1.0-ieee-access-2026")
    ap.add_argument("--target", type=int, default=90, help="stop once the J-LLM score reaches this")
    args = ap.parse_args(argv)

    contract = load_contract(args.contract)
    data = ROOT / "demo_data" / args.system
    instructions = (data / "input1.txt").read_text(encoding="utf-8")
    reference = (data / "truth1.py").read_text(encoding="utf-8")
    api_doc = contract.read_api_doc()

    agent = make_caller("openai", args.agent_model)  # your agent under test

    # Turn 1: the agent drafts a script from the (Turn 1) instructions.
    candidate = agent(build_turn1(instructions))

    for i in range(1, args.rounds + 1):
        ev = evaluate_script(
            candidate, reference=reference, api_doc=api_doc,
            model=contract.judge_model, rubric_dir=str(contract.rubric_dir),
            temperature=contract.temperature, top_p=contract.top_p, max_tokens=contract.max_tokens)
        print(f"round {i}: score={ev.score} (mode={ev.mode}, judge={ev.model})")
        if ev.score is not None and ev.score >= args.target:
            print("target reached; stopping.")
            break
        # Feed the judge's rationale (the per-criterion deductions) back for a revision.
        candidate = agent(
            "Revise your PyChrono script to address this expert feedback. "
            "Output only the corrected, runnable script.\n\n"
            f"--- your script ---\n{candidate}\n\n--- expert feedback ---\n{ev.rationale}")

    print("\n=== final script (first 2000 chars) ===\n" + candidate[:2000])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
