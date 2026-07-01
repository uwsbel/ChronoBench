"""Generalized de-scoped judge for the redesigned (v2 / 10.0) suite.

Reads a task's machine-readable `contract.json` and scores a candidate script through the de-scoped
layered oracle:
  L1 execution gate     -- run the candidate headless under timeout; require clean exit, the declared
                           output file, and a parsable JSON result with the declared keys, no NaN.
  L2 capability (minimal) -- static "capability present" regex checks from the contract (NOT idioms).
  L3 behavioral invariants -- check values against the contract's analytic targets/bounds
                           (rel_tol / max / min / range). Low-bias: targets are physics-derived, not
                           reference-output matching.

Anti-gaming (the key hardening): invariants should be checked against quantities the judge DERIVES
itself from the candidate's emitted CSV trajectory (`derive` block), not against numbers the candidate
self-reports in its JSON line. A model can print the analytic answer (e.g. T = 2*pi*sqrt(L/g)) without
actually simulating; deriving the period from the logged theta(t) closes that hole. The candidate's
JSON is still required (a liveness/completion signal) but need not carry the graded physics.

The existing reference+api rubric LLM attaches later as a residual partial-credit layer (Phase 5);
omitted here to keep authoring/verification offline.

Run UNDER the pychrono10 env so sys.executable runs candidates on PyChrono 10.0:
    conda run -n pychrono10 python scoring/judge_v2.py <task_dir> [candidate.py] [--turn N]
If [candidate.py] is omitted, the task's own reference (run.entry) is judged (gate self-check).

contract.json schema (per task dir). A task is EITHER single-turn (flat `run`/`L2_caps`/`derive`/`L3`
at top level) OR multi-turn (a `turns` list, each element carrying those same fields plus its own
`turn` number; select with --turn, default 1):
{
  "task": "...", "axis": "...", "simulator": "pychrono|pydeme", "source": "...",
  "turns": [ {"turn": 1, "run": {...}, "L2_caps": {...}, "derive": [...], "L3": [...]}, ... ]
  # -- or, single-turn --
  "run": {"entry": "truth1.py", "timeout": 120, "expect_csv": "out.csv",
          "expect_json_keys": ["k1","k2"]},
  "L2_caps": {"name": "regex", ...},
  "derive": [ {"name":"period_meas","kind":"period","csv":"out.csv","column":"theta","time_column":"t"},
              {"name":"amp_meas","kind":"max_abs","csv":"out.csv","column":"theta"},
              {"name":"e_drift","kind":"drift_rel","csv":"out.csv","column":"energy"} ],
  "L3": [ {"kind":"rel_tol","key":"period_meas","target":2.0,"tol":0.1,"desc":"..."},
          {"kind":"max","key":"e_drift","bound":0.05,"desc":"..."},
          {"kind":"min","key":"k3","bound":0.0},
          {"kind":"range","key":"k4","lo":0.0,"hi":1.0} ]
}
`derive` kinds: period (mean full period from upward zero-crossings of column-minus-mean, needs
time_column), max_abs, max, min, range, final, mean, drift_rel (max|x-x0|/|x0|). A derived name is
merged into the result dict alongside the model's JSON keys, so L3 `key` may reference either.

NOTE: L1 executes candidate code. References/authored tasks are trusted; production scoring of
untrusted LLM output must sandbox + resource-limit.
"""
import csv as _csv
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile


# --- CSV-derived observables (anti-gaming: measured from the trajectory, not self-reported) --------
def _read_csv_columns(path):
    with open(path, newline="", encoding="utf-8") as f:
        rows = list(_csv.reader(f))
    if not rows:
        return {}
    header = [h.strip() for h in rows[0]]
    cols = {name: [] for name in header}
    for r in rows[1:]:
        if not r:
            continue
        for i, name in enumerate(header):
            try:
                cols[name].append(float(r[i]))
            except (ValueError, IndexError):
                cols[name].append(float("nan"))
    return cols


def _period(ts, xs):
    """Mean full period from upward zero-crossings of xs (mean-subtracted)."""
    pts = [(t, x) for t, x in zip(ts, xs) if t == t and x == x]
    if len(pts) < 3:
        return float("nan")
    mean = sum(x for _, x in pts) / len(pts)
    cr = [pts[i][0] for i in range(1, len(pts))
          if (pts[i - 1][1] - mean) < 0.0 <= (pts[i][1] - mean)]
    if len(cr) < 2:
        return float("nan")
    return (cr[-1] - cr[0]) / (len(cr) - 1)


def _derive(spec, work):
    """Compute one derived quantity from the candidate's emitted CSV; NaN if it cannot be computed."""
    path = os.path.join(work, spec.get("csv", "out.csv"))
    if not os.path.exists(path):
        return float("nan")
    cols = _read_csv_columns(path)
    kind = spec["kind"]
    if kind == "period":
        return _period(cols.get(spec.get("time_column", "t"), []), cols.get(spec.get("column"), []))
    xs = [x for x in cols.get(spec.get("column"), []) if x == x]
    if not xs:
        return float("nan")
    if kind == "max_abs":
        return max(abs(x) for x in xs)
    if kind == "max":
        return max(xs)
    if kind == "min":
        return min(xs)
    if kind == "range":
        return max(xs) - min(xs)
    if kind == "final":
        return xs[-1]
    if kind == "mean":
        return sum(xs) / len(xs)
    if kind == "drift_rel":
        x0 = xs[0]
        denom = abs(x0) if abs(x0) > 1e-12 else (max(abs(x) for x in xs) or 1.0)
        return max(abs(x - x0) for x in xs) / denom
    return float("nan")


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


def _select_turn(contract, turn):
    """Return the per-turn spec dict. Multi-turn: pick `turns[turn]`; single-turn: the contract itself."""
    turns = contract.get("turns")
    if not turns:
        return contract
    sel = turn or 1
    for tc in turns:
        if tc.get("turn") == sel:
            return tc
    raise SystemExit(f"turn {sel} not found in contract (have {[t.get('turn') for t in turns]})")


def judge(task_dir, candidate_path=None, turn=None):
    contract = json.load(open(os.path.join(task_dir, "contract.json"), encoding="utf-8"))
    tc = _select_turn(contract, turn)
    run = tc["run"]
    candidate_path = candidate_path or os.path.join(task_dir, run["entry"])
    src = open(candidate_path, encoding="utf-8").read()
    v = {"task": contract.get("task"), "turn": tc.get("turn", 1),
         "candidate": os.path.basename(candidate_path), "layers": {}, "triage": None, "score": 0.0}

    # L2 (minimal capability-present, static)
    caps = {name: bool(re.search(pat, src)) for name, pat in tc.get("L2_caps", {}).items()}
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

    # Derived observables (measured from the emitted CSV; merged in so L3 keys may reference them).
    derived = {spec["name"]: _derive(spec, work) for spec in tc.get("derive", [])}
    if derived:
        v["layers"]["derived"] = {k: (round(x, 6) if x == x else None) for k, x in derived.items()}
    result.update(derived)

    # L3 (behavioral invariants from the contract)
    l3_results = []
    all_ok = True
    for spec in tc.get("L3", []):
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
    args = [a for a in sys.argv[1:]]
    turn = None
    if "--turn" in args:
        i = args.index("--turn")
        turn = int(args[i + 1]); del args[i:i + 2]
    td = args[0]
    cand = args[1] if len(args) > 1 else None
    print(json.dumps(judge(td, cand, turn=turn), indent=2))
