"""Generalized de-scoped judge for the redesigned (v2 / 10.0) suite.

Reads a task's machine-readable `contract.json` and scores a candidate script through the de-scoped
layered oracle:
  L1 execution gate     -- run the candidate headless under timeout; require clean exit, the declared
                           output file, and a parsable JSON result with the declared keys, no NaN.
  L2 capability (minimal) -- static "capability present" regex checks from the contract (NOT idioms).
  L3 behavioral invariants -- check the candidate's emitted JSON values against the contract's
                           analytic targets/bounds (rel_tol / max / min / range). Low-bias: targets
                           are physics-derived, not reference-output matching.
The existing reference+api rubric LLM attaches later as a residual partial-credit layer (Phase 5);
omitted here to keep authoring/verification offline.

Run UNDER the pychrono10 env so sys.executable runs candidates on PyChrono 10.0:
    conda run -n pychrono10 python scoring/judge_v2.py <task_dir> <candidate.py>
If <candidate.py> is omitted, the task's own reference (run.entry) is judged (gate self-check).

contract.json schema (per task dir):
{
  "task": "...", "axis": "...", "simulator": "pychrono|pydeme", "source": "...",
  "probe_vector": ["..."],
  "run": {"entry": "truth1.py", "timeout": 120, "expect_csv": "out.csv",
          "expect_json_keys": ["k1","k2"]},
  "L2_caps": {"name": "regex", ...},
  "L3": [ {"kind":"rel_tol","key":"k1","target":2.0,"tol":0.1,"desc":"..."},
          {"kind":"max","key":"k2","bound":0.1,"desc":"..."},
          {"kind":"min","key":"k3","bound":0.0},
          {"kind":"range","key":"k4","lo":0.0,"hi":1.0} ]
}

NOTE: L1 executes candidate code. References/authored tasks are trusted; production scoring of
untrusted LLM output must sandbox + resource-limit.
"""
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile


def _check_l3(spec, vals):
    k = spec["key"]
    if k not in vals:
        return False, f"{k}: missing"
    x = vals[k]
    if not isinstance(x, (int, float)) or x != x:   # missing/NaN
        return False, f"{k}: not-a-number ({x})"
    kind = spec["kind"]
    if kind == "rel_tol":
        ok = abs(x - spec["target"]) / abs(spec["target"]) <= spec["tol"]
        return ok, f"{k}={x:.4g} vs target {spec['target']:.4g} (tol {spec['tol']})"
    if kind == "max":
        return x <= spec["bound"], f"{k}={x:.4g} <= {spec['bound']:.4g}"
    if kind == "min":
        return x >= spec["bound"], f"{k}={x:.4g} >= {spec['bound']:.4g}"
    if kind == "range":
        return spec["lo"] <= x <= spec["hi"], f"{k}={x:.4g} in [{spec['lo']},{spec['hi']}]"
    return False, f"{k}: unknown check kind {kind}"


def judge(task_dir, candidate_path=None):
    contract = json.load(open(os.path.join(task_dir, "contract.json"), encoding="utf-8"))
    run = contract["run"]
    candidate_path = candidate_path or os.path.join(task_dir, run["entry"])
    src = open(candidate_path, encoding="utf-8").read()
    v = {"task": contract.get("task"), "candidate": os.path.basename(candidate_path),
         "layers": {}, "triage": None, "score": 0.0}

    # L2 (minimal capability-present, static)
    caps = {name: bool(re.search(pat, src)) for name, pat in contract.get("L2_caps", {}).items()}
    v["layers"]["L2_caps"] = caps

    # L1 (execution gate)
    work = tempfile.mkdtemp(prefix="cb_judge_")
    shutil.copyfile(candidate_path, os.path.join(work, "cand.py"))
    try:
        r = subprocess.run([sys.executable, "cand.py"], cwd=work, capture_output=True,
                           text=True, timeout=run.get("timeout", 120))
    except subprocess.TimeoutExpired:
        v["triage"] = "run:timeout"; return v
    if r.returncode != 0:
        v["layers"]["L1"] = {"ok": False, "stderr_tail": r.stderr.strip()[-400:]}
        v["triage"] = "run:exception"; return v
    if run.get("expect_csv") and not os.path.exists(os.path.join(work, run["expect_csv"])):
        v["layers"]["L1"] = {"ok": False}; v["triage"] = "missing-output:csv"; return v
    result = None
    for line in r.stdout.splitlines():
        line = line.strip()
        if line.startswith("{"):
            try:
                result = json.loads(line)
            except ValueError:
                pass
    keys = run.get("expect_json_keys", [])
    if result is None or any(k not in result for k in keys):
        v["triage"] = "missing-output:json"; return v
    v["layers"]["L1"] = {"ok": True}

    # L3 (behavioral invariants from the contract)
    l3_results = []
    all_ok = True
    for spec in contract.get("L3", []):
        ok, detail = _check_l3(spec, result)
        all_ok = all_ok and ok
        l3_results.append({"ok": ok, "detail": detail, "desc": spec.get("desc", "")})
    v["layers"]["L3"] = l3_results

    l2 = (sum(caps.values()) / len(caps)) if caps else 1.0
    l3 = (sum(1 for c in l3_results if c["ok"]) / len(l3_results)) if l3_results else 1.0
    v["score"] = round(100 * (0.30 * 1.0 + 0.20 * l2 + 0.50 * l3), 1)   # L1 passed => 1.0
    v["triage"] = "pass" if all_ok else "invariant-fail"
    return v


if __name__ == "__main__":
    td = sys.argv[1]
    cand = sys.argv[2] if len(sys.argv) > 2 else None
    print(json.dumps(judge(td, cand), indent=2))
