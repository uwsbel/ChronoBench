"""Minimal de-scoped judge (Phase 0 pilot) for the pendulum task.

Demonstrates the de-scoped oracle on one task:
  L1 execution gate  -- run the candidate headless under timeout; require clean exit, out.csv, and a
                        parsable JSON result with no NaN.
  L2 capability (minimal) -- static "capability present" checks only (revolute joint, gravity set,
                        a stepping loop); NOT "preferred idiom".
  L3 behavioral invariant -- period within +/-10% of the analytic small-angle period, and amplitude
                        does not grow.
Returns a structured verdict with a failure-triage category. (The existing reference+api rubric LLM
would attach as a residual partial-credit layer; omitted here to keep the pilot offline.)

Run UNDER the pychrono10 env so sys.executable runs candidates on PyChrono 10.0:
    conda run -n pychrono10 python scoring/judge_pilot.py <candidate.py>

NOTE: L1 executes candidate code. Pilot candidates are trusted (hand-written). Production must
sandbox/resource-limit untrusted LLM-generated scripts.
"""
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile

ANALYTIC_T = 2 * math.pi * math.sqrt(1.0 / 9.81)   # ~2.006 s
THETA0 = math.radians(5.0)


def judge(candidate_path, timeout=120):
    src = open(candidate_path, encoding="utf-8").read()
    v = {"candidate": os.path.basename(candidate_path), "layers": {}, "triage": None, "score": 0.0}

    # L2 (minimal capability-present, static)
    caps = {
        "revolute_joint": bool(re.search(r"Revolute", src)),
        "gravity_set": bool(re.search(r"SetGravitationalAcceleration|Gravitational|G_acc", src)),
        "step_loop": bool(re.search(r"DoStepDynamics", src)),
    }
    v["layers"]["L2_caps"] = caps

    # L1 (execution gate)
    work = tempfile.mkdtemp(prefix="cb_judge_")
    shutil.copyfile(candidate_path, os.path.join(work, "cand.py"))
    try:
        r = subprocess.run([sys.executable, "cand.py"], cwd=work, capture_output=True,
                           text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        v["triage"] = "run:timeout"; return v
    if r.returncode != 0:
        v["layers"]["L1"] = {"ok": False, "stderr_tail": r.stderr.strip()[-400:]}
        v["triage"] = "run:exception"; return v
    if not os.path.exists(os.path.join(work, "out.csv")):
        v["layers"]["L1"] = {"ok": False}; v["triage"] = "missing-output:csv"; return v
    result = None
    for line in r.stdout.splitlines():
        line = line.strip()
        if line.startswith("{"):
            try:
                result = json.loads(line)
            except ValueError:
                pass
    if not result or "period_est" not in result or "theta_max" not in result:
        v["triage"] = "missing-output:json"; return v
    pe, tm = result["period_est"], result["theta_max"]
    if not (isinstance(pe, (int, float)) and isinstance(tm, (int, float)) and pe == pe and tm == tm):
        v["layers"]["L1"] = {"ok": False}; v["triage"] = "run:nan"; return v
    v["layers"]["L1"] = {"ok": True}

    # L3 (behavioral invariants)
    period_ok = abs(pe - ANALYTIC_T) / ANALYTIC_T <= 0.10
    ampl_ok = tm <= 1.2 * THETA0
    v["layers"]["L3"] = {"period_est": pe, "analytic_T": round(ANALYTIC_T, 4), "period_ok": period_ok,
                         "theta_max": tm, "ampl_ok": ampl_ok}

    l2 = sum(caps.values()) / len(caps)
    l3 = 0.7 * (1.0 if period_ok else 0.0) + 0.3 * (1.0 if ampl_ok else 0.0)
    v["score"] = round(100 * (0.30 * 1.0 + 0.20 * l2 + 0.50 * l3), 1)   # L1 passed => 1.0
    v["triage"] = "pass" if (period_ok and ampl_ok) else "invariant-fail"
    return v


if __name__ == "__main__":
    print(json.dumps(judge(sys.argv[1]), indent=2))
