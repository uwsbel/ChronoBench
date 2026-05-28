"""
New experiment driver: model name `chrono-agent`.

Per the user's requirements:
  1. Every demo goes THROUGH the PlanningAgent (use its topology-arrangement
     ability) — do NOT bypass planning.
  2. The plan has EXACTLY 3 implementation_steps = the demo's 3 turns.
  3. Each turn's final code must execute AND pass review.

Flow per system:
  1. Build a combined 3-stage prompt (the 3 SimBench turns as progressive
     stages) with canonical time_step + duration embedded.
  2. Run PlanningAgent.execute(prompt) in-process -> a SimulationPlan
     (topology / assets / simulation_parameters arranged by planning).
  3. Coerce: force plan_type="mbs_in_scene" (so the step loop runs) and set
     implementation_steps = exactly 3 turn-steps (description = inputN.txt),
     preserving planning's topology / assets / simulation_parameters.
  4. Write plan.json, run `chrono-agent run-from-plan` (subprocess, cwd=
     chrono-code for .env, CHRONO_AGENT_EXEC_PYTHON=chrono-agent env for
     libvsg 1.1.13). The step loop does CodeGen -> Execution -> StepReview ->
     retry per step; the prior step's APPROVED code feeds the next step.
  5. Parse chrono_agent.log for per-step "Step N PASSED/FAILED" + give-up.
  6. Harvest each step's final iteration simulation.py (by step_context
     step_index) -> output_llms/chrono-agent/<system>/{first,second,third}_response.txt
  7. Escalate (higher retry cap) for systems where a turn didn't pass review;
     record unrecoverable ones in _review_failures.txt.

This driver MUST run with the chrono-code conda env python (has chrono_agent
+ reads chrono-code/.env for keys) and cwd=/home/hongyu/Documents/chrono-code.
"""
from __future__ import annotations

import argparse
import asyncio
import json
import logging
import os
import re
import shutil
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Dict, List, Optional

SIMBENCH_ROOT = Path("/home/hongyu/Documents/SimBench")
CHRONO_ROOT = Path("/home/hongyu/Documents/chrono-code")
MODEL = "chrono-agent"
CHRONO_AGENT_BIN = "/home/hongyu/anaconda3/envs/chrono-code/bin/chrono-agent"
EXEC_PY = "/home/hongyu/anaconda3/envs/chrono-agent/bin/python"

PER_SYSTEM_TIMEOUT = int(os.environ.get("CA_SYSTEM_TIMEOUT", "2400"))
PER_EXEC_TIMEOUT = int(os.environ.get("CA_EXEC_TIMEOUT", "60"))
PLANNING_TIMEOUT = int(os.environ.get("CA_PLANNING_TIMEOUT", "300"))

ALL_SYSTEMS = [
    "art", "beam", "buckling", "cable", "camera", "citybus", "curiosity",
    "feda", "gator", "gear", "gps_imu", "handler", "hmmwv", "kraz", "lidar",
    "m113", "man", "mass_spring_damper", "particles", "pendulum",
    "rigid_highway", "rigid_multipatches", "rotor", "scm", "scm_hill",
    "sedan", "sensros", "slider_crank", "tablecloth", "turtlebot", "uazbus",
    "veh_app", "vehros", "viper",
]
RESPONSE = {0: "first_response", 1: "second_response", 2: "third_response"}
DEFAULT_CAMERAS = [
    {"position": [4, 3, 4],  "target": [0, 0, 0], "up": [0, 1, 0]},
    {"position": [-4, 3, 4], "target": [0, 0, 0], "up": [0, 1, 0]},
    {"position": [0, 8, 0],  "target": [0, 0, 0], "up": [0, 0, 1]},
]
_RE_DOSTEP = re.compile(r"DoStepDynamics\(\s*([0-9.eE+\-_]+)\s*\)")


def canonical_time_step(system: str) -> float:
    p = SIMBENCH_ROOT / "demo_data" / system / "cleaned_truth1.py"
    if p.exists():
        m = _RE_DOSTEP.search(p.read_text(errors="ignore"))
        if m:
            try:
                v = float(m.group(1).replace("_", ""))
                if 1e-6 <= v <= 1.0:
                    return v
            except ValueError:
                pass
    return 1e-3


def read_inputs(system: str) -> List[str]:
    sd = SIMBENCH_ROOT / "demo_data" / system
    return [(sd / f"input{i}.txt").read_text(encoding="utf-8").strip() for i in (1, 2, 3)]


def combined_prompt(system: str, instrs: List[str], ts: float) -> str:
    return (
        "Create a PyChrono simulation developed in 3 progressive stages. "
        "Produce a plan with EXACTLY 3 implementation_steps, one per stage; "
        "each stage MODIFIES the previous stage's resulting script (it is the "
        "same evolving simulation, not new co-existing objects). Arrange the "
        "scene so bodies do not overlap.\n\n"
        f"Stage 1: {instrs[0]}\n\n"
        f"Stage 2 (modify the Stage 1 script): {instrs[1]}\n\n"
        f"Stage 3 (modify the Stage 2 script): {instrs[2]}\n\n"
        f"time_step: {ts}\nsimulation_duration: 10s\n"
    )


def run_planning(prompt: str) -> dict:
    """In-process PlanningAgent.execute -> plan.dump_all() dict."""
    from chrono_agent.agents.planning_agent import PlanningAgent

    async def _go():
        plan = await asyncio.wait_for(
            PlanningAgent().execute(prompt, plan_mode="auto"), timeout=PLANNING_TIMEOUT)
        return plan.dump_all()

    return asyncio.run(_go())


def coerce_plan(raw: dict, system: str, instrs: List[str], ts: float) -> dict:
    """Force mbs_in_scene + exactly 3 turn-steps, preserving planning's
    topology/assets/simulation_parameters."""
    from chrono_agent.models.plan import SimulationPlan

    raw = dict(raw)
    raw["plan_type"] = "mbs_in_scene"
    raw.setdefault("simulation_parameters", {})
    raw["simulation_parameters"].setdefault("time_step", ts)
    raw["simulation_parameters"].setdefault("simulation_duration", 10.0)

    assets = raw.get("assets") or []
    asset_names = [a.get("name") for a in assets if isinstance(a, dict) and a.get("name")]
    if not asset_names:
        asset_names = [f"{system}_system"]
        raw["assets"] = [{"name": asset_names[0], "type": "placeholder",
                          "description": f"Primary {system} setup."}]

    raw["implementation_steps"] = [
        {
            "description": instrs[i],
            "assets": asset_names,
            "scene_objects": [],
            "objects": [],
            "cameras": DEFAULT_CAMERAS,
            "constraints": [],
            "motion_expectations": [],
        }
        for i in range(3)
    ]
    SimulationPlan.model_validate(raw)  # raises if invalid
    return raw


def build_env(extra_retries: Optional[int] = None) -> dict:
    env = os.environ.copy()
    env["CHRONO_AGENT_EXEC_PYTHON"] = EXEC_PY
    if extra_retries:
        env["CHRONO_AGENT_MAX_EXEC_RETRIES"] = str(extra_retries)
    env.setdefault("DISPLAY", ":1")
    return env


def watch_history(system: str, out_dir: Path, started_wall: float,
                  stop: threading.Event, log: logging.Logger) -> None:
    hist = CHRONO_ROOT / "history"
    seen: set = set()
    while not stop.is_set():
        try:
            if hist.exists():
                for it in sorted(hist.iterdir(), key=lambda p: p.name):
                    if not it.is_dir() or not it.name.startswith("iteration_") or it.name in seen:
                        continue
                    if it.stat().st_mtime < started_wall:
                        continue
                    ctx, sim = it / "step_context.json", it / "simulation.py"
                    if not (ctx.exists() and sim.exists() and sim.stat().st_size > 0):
                        continue
                    try:
                        si = int(json.loads(ctx.read_text()).get("step_index", -1))
                    except Exception:
                        continue
                    if si in RESPONSE:
                        try:
                            (out_dir / f"{RESPONSE[si]}.txt").write_text(
                                sim.read_text(encoding="utf-8"), encoding="utf-8")
                            log.info("%s: %s landed step_index=%d", system, it.name, si)
                            seen.add(it.name)
                        except OSError:
                            pass
        except Exception:
            pass
        stop.wait(5.0)


def harvest(system: str, out_dir: Path, started_wall: float, log: logging.Logger) -> Dict[int, str]:
    hist = CHRONO_ROOT / "history"
    by_step: Dict[int, Path] = {}
    if hist.exists():
        def _n(p: Path) -> int:
            try:
                return int(p.name.split("_")[-1])
            except ValueError:
                return -1
        for it in sorted(hist.iterdir(), key=_n):
            if not it.is_dir() or not it.name.startswith("iteration_"):
                continue
            if it.stat().st_mtime < started_wall:
                continue
            ctx, sim = it / "step_context.json", it / "simulation.py"
            if not (ctx.exists() and sim.exists() and sim.stat().st_size > 0):
                continue
            try:
                si = int(json.loads(ctx.read_text()).get("step_index", -1))
            except Exception:
                continue
            if si in RESPONSE:
                by_step[si] = sim
    res = {}
    for si, name in RESPONSE.items():
        if si in by_step:
            (out_dir / f"{name}.txt").write_text(by_step[si].read_text(encoding="utf-8"), encoding="utf-8")
            res[si] = "ok"
        else:
            res[si] = "missing"
    return res


def parse_step_passes(stdout: str) -> Dict[int, bool]:
    """From a run's stdout/log text, which step_index PASSED review."""
    passed = {}
    for m in re.finditer(r"Step (\d+) PASSED", stdout):
        passed[int(m.group(1))] = True
    return passed


def run_one_system(system: str, log: logging.Logger, extra_retries: Optional[int]) -> dict:
    out_dir = SIMBENCH_ROOT / "output_llms" / MODEL / system
    out_dir.mkdir(parents=True, exist_ok=True)
    instrs = read_inputs(system)
    ts = canonical_time_step(system)

    log.info("%s: planning...", system)
    try:
        raw_plan = run_planning(combined_prompt(system, instrs, ts))
    except Exception as exc:
        log.error("%s: planning failed: %s", system, exc)
        return {"status": "planning-failed"}
    log.info("%s: planning plan_type=%s steps=%d topology=%s", system,
             raw_plan.get("plan_type"), len(raw_plan.get("implementation_steps") or []),
             "yes" if raw_plan.get("topology") else "none")

    try:
        plan = coerce_plan(raw_plan, system, instrs, ts)
    except Exception as exc:
        log.error("%s: coerce/validate failed: %s", system, str(exc)[:300])
        return {"status": "coerce-failed"}

    plan_path = out_dir / "plan.json"
    plan_path.write_text(json.dumps(plan, indent=2), encoding="utf-8")

    shutil.rmtree(CHRONO_ROOT / "history", ignore_errors=True)
    cmd = [CHRONO_AGENT_BIN, "run-from-plan", "--plan-file", str(plan_path),
           "--detail-level", "minimal", "--timeout", str(PER_EXEC_TIMEOUT)]
    env = build_env(extra_retries)
    started_wall = time.time()
    work = out_dir / ".chrono_work"
    work.mkdir(exist_ok=True)

    stop = threading.Event()
    watcher = threading.Thread(target=watch_history,
                               args=(system, out_dir, started_wall, stop, log), daemon=True)
    watcher.start()

    timed_out = False
    stdout_text = ""
    try:
        with open(work / "stdout.log", "w", encoding="utf-8", buffering=1) as fout:
            proc = subprocess.Popen(cmd, cwd=str(CHRONO_ROOT), stdin=subprocess.PIPE,
                                    stdout=fout, stderr=subprocess.STDOUT, text=True, bufsize=1)
            try:
                proc.stdin.write("\n" * 32); proc.stdin.close()
            except (BrokenPipeError, OSError):
                pass
            try:
                proc.wait(timeout=PER_SYSTEM_TIMEOUT)
            except subprocess.TimeoutExpired:
                timed_out = True
                proc.kill()
                proc.wait(timeout=10)
    finally:
        stop.set(); watcher.join(timeout=15)

    try:
        stdout_text = (work / "stdout.log").read_text(encoding="utf-8", errors="ignore")
    except OSError:
        pass
    passes = parse_step_passes(stdout_text)
    res = harvest(system, out_dir, started_wall, log)
    captured = sum(1 for v in res.values() if v == "ok")
    n_passed = sum(1 for si in (0, 1, 2) if passes.get(si))
    log.info("%s: captured=%d/3 review_passed=%d/3 timed_out=%s", system, captured, n_passed, timed_out)
    return {"status": "done", "captured": captured, "passed": n_passed,
            "passes": {si: bool(passes.get(si)) for si in (0, 1, 2)}, "timed_out": timed_out}


def setup_logger() -> logging.Logger:
    logp = SIMBENCH_ROOT / "scripts" / "run_chrono_agent_exp.log"
    logp.parent.mkdir(exist_ok=True)
    lg = logging.getLogger("ca_exp"); lg.setLevel(logging.INFO)
    fmt = logging.Formatter("%(asctime)s %(message)s", "%H:%M:%S")
    fh = logging.FileHandler(logp, mode="a", encoding="utf-8"); fh.setFormatter(fmt)
    sh = logging.StreamHandler(sys.stdout); sh.setFormatter(fmt)
    lg.handlers = [fh, sh]; lg.propagate = False
    return lg


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--systems", nargs="*", default=None)
    ap.add_argument("--escalate-retries", type=int, default=18,
                    help="On a system with <3 review passes, re-run once with this exec-retry cap.")
    args = ap.parse_args()
    systems = [s for s in ALL_SYSTEMS if (not args.systems or s in args.systems)]

    log = setup_logger()
    log.info("=" * 60)
    log.info("chrono-agent experiment: %d systems: %s", len(systems), ", ".join(systems))

    failures = []
    for system in systems:
        r = run_one_system(system, log, extra_retries=None)
        # escalate if not all 3 turns passed review
        if r.get("status") == "done" and r.get("passed", 0) < 3:
            log.info("%s: only %d/3 review-passed — escalating retries=%d",
                     system, r.get("passed", 0), args.escalate_retries)
            r = run_one_system(system, log, extra_retries=args.escalate_retries)
        if r.get("status") != "done" or r.get("passed", 0) < 3:
            for si in (0, 1, 2):
                if not r.get("passes", {}).get(si):
                    failures.append(f"{system}/turn{si+1}")

    if failures:
        fp = SIMBENCH_ROOT / "output_llms" / MODEL / "_review_failures.txt"
        fp.write_text("\n".join(failures) + "\n", encoding="utf-8")
        log.info("review-not-passed turns (%d): see %s", len(failures), fp)
    else:
        log.info("ALL turns passed review across %d systems", len(systems))
    return 0


if __name__ == "__main__":
    sys.exit(main())
