"""
Driver: drive chrono-code over all 34 SimBench systems by feeding each
system a hand-authored 3-step plan.json and invoking
``chrono-agent run-from-plan`` exactly once per system.

Architecture (matches /home/hongyu/.claude/plans/declarative-yawning-flamingo.md):

  - Each SimBench system maps to ONE chrono-agent invocation.
  - SimBench's 3 progressive-refinement turns (input1/2/3) are encoded as
    3 SimulationStep entries in the plan's ``implementation_steps``.
  - chrono-agent's workflow already supports step-by-step CodeGen with the
    prior step's APPROVED code auto-injected as ``previous_code`` for the
    next step (chrono_agent/workflow/nodes.py:_backfill_previous_code +
    chrono_agent/agents/code_generation_agent.py "fix mode").
  - After the subprocess returns, harvest each step's final
    ``simulation.py`` from chrono-code's history/iteration_NNN/ by
    grouping iterations on ``step_context.json.step_index``.

Produces the SimBench S-LLM contract:
    output_llms/pe_chrono-code/<system>/{first,second,third}_response.txt

Run:
    python scripts/run_chrono_code.py --systems pendulum         # smoke
    python scripts/run_chrono_code.py                            # all 34
    python scripts/run_chrono_code.py --limit 3                  # first 3
"""

from __future__ import annotations

import argparse
import concurrent.futures
import json
import logging
import os
import re
import shlex
import shutil
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional

SIMBENCH_ROOT = Path(__file__).resolve().parents[1]
CHRONO_ROOT = Path(os.environ.get("CHRONO_CODE_ROOT", "/home/hongyu/Documents/chrono-code"))
MODEL_NAME = "pe_chrono-code"

DEFAULT_CMD = os.environ.get(
    "CHRONO_AGENT_CMD",
    "/home/hongyu/anaconda3/envs/chrono-code/bin/chrono-agent",
)
PER_SYSTEM_TIMEOUT_S = int(os.environ.get("CHRONO_SYSTEM_TIMEOUT", "1800"))
PER_EXEC_TIMEOUT_S = int(os.environ.get("CHRONO_EXEC_TIMEOUT", "60"))

SYSTEMS: List[str] = [
    "art", "beam", "buckling", "cable", "camera", "citybus", "curiosity",
    "feda", "gator", "gear", "gps_imu", "handler", "hmmwv", "kraz", "lidar",
    "m113", "man", "mass_spring_damper", "particles", "pendulum",
    "rigid_highway", "rigid_multipatches", "rotor", "scm", "scm_hill",
    "sedan", "sensros", "slider_crank", "tablecloth", "turtlebot", "uazbus",
    "veh_app", "vehros", "viper",
]

RESPONSE_FILES = {
    0: "first_response.txt",
    1: "second_response.txt",
    2: "third_response.txt",
}

# Regex over each system's ``cleaned_truth1.py`` to pull a canonical
# time_step. Most SimBench truths use DoStepDynamics(1e-3).
_RE_DOSTEP = re.compile(r"DoStepDynamics\(\s*([0-9.eE+\-_]+)\s*\)")
_RE_STEPSIZE = re.compile(r"step_size\s*=\s*([0-9.eE+\-_]+)")


def _parse_float(token: str) -> Optional[float]:
    try:
        return float(token.replace("_", ""))
    except ValueError:
        return None


def extract_canonical_time_step(truth_path: Path) -> float:
    if truth_path.exists():
        text = truth_path.read_text(encoding="utf-8", errors="ignore")
        for rx in (_RE_DOSTEP, _RE_STEPSIZE):
            m = rx.search(text)
            if m:
                v = _parse_float(m.group(1))
                if v and 1e-6 <= v <= 1.0:
                    return v
    return 1e-3


def _default_cameras() -> List[dict]:
    # 3 complementary viewing directions per step — SimulationPlan validator
    # requires 2-3 cameras for ``recording_mode='sensor_cams'`` (the default).
    return [
        {"position": [4, 3, 4],  "target": [0, 0, 0], "up": [0, 1, 0]},
        {"position": [-4, 3, 4], "target": [0, 0, 0], "up": [0, 1, 0]},
        {"position": [0, 8, 0],  "target": [0, 0, 0], "up": [0, 0, 1]},
    ]


def build_plan(system: str) -> dict:
    """Author a 3-step SimulationPlan whose step descriptions are the
    raw input{1,2,3}.txt content. Asset / camera fields are minimal
    placeholders required by the plan schema; CodeGen derives real
    semantics from the description text."""
    instructions = [
        (SIMBENCH_ROOT / "demo_data" / system / f"input{i}.txt")
        .read_text(encoding="utf-8")
        .strip()
        for i in (1, 2, 3)
    ]
    time_step = extract_canonical_time_step(
        SIMBENCH_ROOT / "demo_data" / system / "cleaned_truth1.py"
    )
    asset_name = f"{system}_system"
    return {
        "plan_type": "mbs_in_scene",
        "simulation_parameters": {
            "time_step": time_step,
            "simulation_duration": 10.0,
        },
        "objectives": [
            f"SimBench {system} task — three progressive refinement turns "
            "(turn1 builds initial system; turn2/3 refine it).",
        ],
        "topology": {"gravity_axis": "-y"},
        "assets": [{
            "name": asset_name,
            "type": "placeholder",
            "description": f"Primary {system} setup; CodeGen decides actual structure from step description.",
        }],
        "implementation_steps": [
            {
                "description": instr,
                "assets": [asset_name],
                "scene_objects": [],
                "objects": [],
                "cameras": _default_cameras(),
                "constraints": [],
                "motion_expectations": [],
            }
            for instr in instructions
        ],
    }


def _cmd_argv() -> List[str]:
    return shlex.split(DEFAULT_CMD) if " " in DEFAULT_CMD else [DEFAULT_CMD]


def harvest_responses(out_dir: Path, started_wall: float, log: logging.Logger) -> Dict[int, str]:
    """Pick the latest iteration_NNN under chrono-code/history/ for each
    step_index in {0,1,2}, copy its simulation.py to the matching
    response.txt. Returns {turn(1-3): status} where status is 'ok' or
    'missing'."""
    history_dir = CHRONO_ROOT / "history"
    iters_by_step: Dict[int, Path] = {}

    if history_dir.exists():
        def _iter_num(p: Path) -> int:
            try:
                return int(p.name.split("_")[-1])
            except ValueError:
                return -1

        for it in sorted(history_dir.iterdir(), key=_iter_num):
            if not it.is_dir() or not it.name.startswith("iteration_"):
                continue
            if it.stat().st_mtime < started_wall:
                continue  # stale (we wiped history/ before run, but guard)
            ctx_path = it / "step_context.json"
            sim_path = it / "simulation.py"
            if not (ctx_path.exists() and sim_path.exists() and sim_path.stat().st_size > 0):
                continue
            try:
                ctx = json.loads(ctx_path.read_text(encoding="utf-8"))
                step_index = int(ctx.get("step_index", -1))
            except (json.JSONDecodeError, ValueError, TypeError):
                continue
            if step_index in RESPONSE_FILES:
                iters_by_step[step_index] = sim_path  # later iteration wins

    results: Dict[int, str] = {}
    for step_index, fname in RESPONSE_FILES.items():
        target = out_dir / fname
        if step_index in iters_by_step:
            target.write_text(
                iters_by_step[step_index].read_text(encoding="utf-8"),
                encoding="utf-8",
            )
            log.info("  captured step_index=%d ← %s → %s",
                     step_index, iters_by_step[step_index].parent.name, fname)
            results[step_index + 1] = "ok"
        else:
            results[step_index + 1] = "missing"
    return results


def run_one_system(system: str, log: logging.Logger) -> Dict[int, str]:
    out_dir = SIMBENCH_ROOT / "output_llms" / MODEL_NAME / system
    out_dir.mkdir(parents=True, exist_ok=True)

    # Resume: all three response.txt non-empty ⇒ skip whole system.
    if all(
        (out_dir / fname).exists() and (out_dir / fname).stat().st_size > 0
        for fname in RESPONSE_FILES.values()
    ):
        log.info("%s: all 3 response.txt present, skipping system", system)
        return {1: "skipped", 2: "skipped", 3: "skipped"}

    plan_path = out_dir / "plan.json"
    plan = build_plan(system)
    plan_path.write_text(json.dumps(plan, indent=2), encoding="utf-8")

    # Wipe chrono-code/history/ so this run's iterations are unambiguous.
    history_dir = CHRONO_ROOT / "history"
    if history_dir.exists():
        shutil.rmtree(history_dir, ignore_errors=True)

    cmd = _cmd_argv() + [
        "run-from-plan",
        "--plan-file", str(plan_path),
        "--detail-level", "minimal",
        "--timeout", str(PER_EXEC_TIMEOUT_S),
    ]

    log.info("%s: %s", system, " ".join(shlex.quote(c) for c in cmd))
    started_wall = time.time()
    started = time.monotonic()

    work_logs = out_dir / ".chrono_work"
    work_logs.mkdir(exist_ok=True)
    timed_out = False
    proc_rc: Optional[int] = None
    proc_stdout = ""
    proc_stderr = ""
    try:
        proc = subprocess.run(
            cmd,
            cwd=str(CHRONO_ROOT),
            capture_output=True,
            text=True,
            # run-from-plan skips PlanningAgent so the parameter / approval
            # gates don't fire — but feed empty newlines just in case any
            # unexpected typer.prompt() pops up.
            input="\n" * 32,
            timeout=PER_SYSTEM_TIMEOUT_S,
            check=False,
        )
        proc_rc = proc.returncode
        proc_stdout = proc.stdout or ""
        proc_stderr = proc.stderr or ""
    except subprocess.TimeoutExpired as exc:
        timed_out = True
        if isinstance(exc.stdout, bytes):
            proc_stdout = exc.stdout.decode("utf-8", errors="ignore")
        elif exc.stdout:
            proc_stdout = exc.stdout
        if isinstance(exc.stderr, bytes):
            proc_stderr = exc.stderr.decode("utf-8", errors="ignore")
        elif exc.stderr:
            proc_stderr = exc.stderr
        log.warning("%s: timeout after %ds — harvesting whatever is in history/",
                    system, PER_SYSTEM_TIMEOUT_S)

    elapsed = time.monotonic() - started
    (work_logs / "stdout.log").write_text(proc_stdout, encoding="utf-8")
    (work_logs / "stderr.log").write_text(proc_stderr, encoding="utf-8")
    log.info("%s: subprocess done in %.1fs (rc=%s, timed_out=%s)",
             system, elapsed, proc_rc, timed_out)

    results = harvest_responses(out_dir, started_wall, log)
    captured = sum(1 for v in results.values() if v == "ok")
    log.info("%s: captured %d/3 turns", system, captured)
    return results


def setup_logger() -> logging.Logger:
    log_dir = SIMBENCH_ROOT / "scripts"
    log_dir.mkdir(exist_ok=True)
    log_path = log_dir / "run_chrono_code.log"
    logger = logging.getLogger("run_chrono_code")
    logger.setLevel(logging.INFO)
    fmt = logging.Formatter("%(asctime)s %(levelname)s %(message)s", "%H:%M:%S")
    fh = logging.FileHandler(log_path, mode="a", encoding="utf-8")
    fh.setFormatter(fmt)
    sh = logging.StreamHandler(sys.stdout)
    sh.setFormatter(fmt)
    logger.handlers = [fh, sh]
    logger.propagate = False
    return logger


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--systems", nargs="*", default=None,
                    help="Subset of system names to run. Default: all 34.")
    ap.add_argument("--limit", type=int, default=None,
                    help="Run only first N systems (after --systems filter).")
    ap.add_argument("--workers", type=int, default=1,
                    help="Concurrent systems. >1 races chrono-code/history/, so leave at 1 "
                         "unless you arrange per-system isolation.")
    args = ap.parse_args()

    log = setup_logger()
    log.info("=" * 60)
    log.info("run started: %s", datetime.now().isoformat(timespec="seconds"))
    log.info("SIMBENCH_ROOT=%s", SIMBENCH_ROOT)
    log.info("CHRONO_ROOT=%s",    CHRONO_ROOT)
    log.info("CHRONO_AGENT_CMD=%s", DEFAULT_CMD)
    log.info("PER_SYSTEM_TIMEOUT_S=%d", PER_SYSTEM_TIMEOUT_S)
    log.info("PER_EXEC_TIMEOUT_S=%d", PER_EXEC_TIMEOUT_S)

    systems = SYSTEMS if not args.systems else [s for s in SYSTEMS if s in args.systems]
    if args.limit:
        systems = systems[: args.limit]
    log.info("running %d systems: %s", len(systems), ", ".join(systems))

    if args.workers > 1:
        log.warning("workers=%d > 1: chrono-code/history/ is global and will race; "
                    "expect harvest collisions. Use workers=1 unless you redirect "
                    "chrono-agent's cwd per call.", args.workers)

    summary: Dict[str, Dict[int, str]] = {}
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.workers) as ex:
        futs = {ex.submit(run_one_system, s, log): s for s in systems}
        for fut in concurrent.futures.as_completed(futs):
            s = futs[fut]
            try:
                summary[s] = fut.result()
            except Exception as exc:
                log.exception("system %s crashed: %s", s, exc)
                summary[s] = {1: "crashed", 2: "crashed", 3: "crashed"}

    log.info("-" * 60)
    log.info("summary:")
    n_ok = n_skip = n_miss = n_crash = 0
    for s in systems:
        r = summary.get(s, {})
        for t in (1, 2, 3):
            status = r.get(t, "missing") if isinstance(r, dict) else "crashed"
            log.info("  %-22s turn%d  %s", s, t, status)
            if status == "ok":
                n_ok += 1
            elif status == "skipped":
                n_skip += 1
            elif status == "crashed":
                n_crash += 1
            else:
                n_miss += 1
    log.info("totals: ok=%d skipped=%d missing=%d crashed=%d (of %d)",
             n_ok, n_skip, n_miss, n_crash, len(systems) * 3)
    return 0 if (n_miss + n_crash) == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
