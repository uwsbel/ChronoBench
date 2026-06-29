"""The --dry-run paths build prompts and lay out work without spending an API call. These
exercise generate + score end-to-end against the real demo_data and the v1.0 contract rubric,
with no network and no OPENAI_API_KEY needed."""
from pathlib import Path

from chronobench.score import score_one
from chronobench.generate import generate_system
from chronobench.contract import load_contract, DEFAULT_CONTRACT

REPO = Path(__file__).resolve().parent.parent
DEMO = REPO / "demo_data"
SYS = "pendulum"


def test_score_one_dry_run_renders_all_three_modes(tmp_path):
    cand_dir = tmp_path / "model" / SYS
    cand_dir.mkdir(parents=True)
    for r in ("first", "second", "third"):
        (cand_dir / f"{r}_response.py").write_text("import pychrono as chrono\n", encoding="utf-8")
    rd = str(load_contract(DEFAULT_CONTRACT).rubric_dir)
    rows = score_one(
        "model", SYS, str(tmp_path), str(DEMO),
        api_doc="SOME API DOCUMENTATION",
        modes=["api", "ref", "ref_api"],
        judge_kwargs={"rubric_dir": rd}, client=None, dry_run=True,
    )
    assert len(rows) == 3  # one row per turn
    for row in rows:
        assert row["api"] == "OK" and row["ref"] == "OK" and row["ref_api"] == "OK"


def test_generate_system_dry_run_builds_prompts(tmp_path):
    msg = generate_system("model", SYS, str(DEMO), str(tmp_path), call=None, dry_run=True)
    assert msg.endswith("prompts built")
