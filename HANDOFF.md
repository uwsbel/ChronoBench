# HANDOFF -- ChronoBench v2.0 / PyChrono 10.0 redesign (morning of 2026-06-30)

Overnight autonomous progress on branch `chrono10-redesign` (NOT pushed; `git push -u origin
chrono10-redesign` to back up). Nothing on `main` changed; v1.0/paper is safe at tag
`paper-ieee-access-2026`.

## What got done (autonomous, all committed)

1. **De-scoped judge harness, generalized and validated.** `scoring/judge_v2.py` reads each task's
   `contract.json` and runs the de-scoped oracle: L1 execution gate on `pychrono10` + minimal L2
   capability checks + L3 behavioral invariants, with failure-triage. Gate self-check:
   `conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/<task>`
2. **3 tasks authored + VERIFIED** (gate passes, score 100), spanning the panel's hard axes:
   - `demo_data_10/pendulum` (mechanism; period invariant) -- also has good/bad samples proving the
     judge passes correct-but-different style and catches runs-but-wrong physics.
   - `demo_data_10/mass_spring_damper` (force element/TSDA; damped-period + damping-ratio invariants).
   - `demo_data_10/swig_contact_reporter` (SWIG callback + collision + contact instrumentation).
3. **Real 10.0 idioms discovered + recorded** in `docs/DELTAS_10.md` (notably: collision system must
   be set explicitly, and `OnReportContact` has 10 args). This is the seed delta table for porting.
4. **Ledger + design docs current:** `demo_data_10/STATUS.md` (resumable per-task state), and the
   de-scoped judge decision is reflected in `docs/SUITE_DESIGN.md` + `docs/PANEL_REDTEAM.md`.

## How to resume (any fresh session)
Read `demo_data_10/STATUS.md` (the ledger). Each `pending` task = author `truth1.py` + `contract.json`
in `demo_data_10/<task>/`, gate-verify, commit, mark `verified`. Reuse `docs/DELTAS_10.md` for idioms
and the verified tasks as templates.

## Gates that need YOU (Dan)
1. **Approve/adjust the task list** (the STATUS.md table): confirm the final ground-locomotion set
   (cap <=3-4 incl. rovers, so `scm`/`rigid_multipatches`/`curiosity` may drop or re-tag), pick the
   one car (sedan or citybus), and decide CAD include/defer. I authored only low-risk items so far.
2. **Expert sign-off** on a per-axis sample of references (execution + invariant pass is necessary,
   not sufficient for correctness).
3. **Recalibration** of the de-scoped judge vs your human judgment (inherently yours).
4. **Deferred tracks** (GPU sensors, ROS, PyDEME) need your infra; they are designed + flagged in
   STATUS/SUITE_DESIGN, not authored.

## What you can do right now
- Run a real bench job on the EXISTING v1.0 (on `main`): score a model already in `output_llms/`,
  e.g. `conda run -n chronobench python scoring/extractPy.py claude-4-sonnet-20250514` then (with
  `OPENAI_API_KEY` set) `conda run -n chronobench python -m chronobench.score claude-4-sonnet-20250514`.
- Kick the tires on the NEW judge: `conda run -n pychrono10 python scoring/judge_v2.py
  demo_data_10/pendulum <your_candidate.py>`.

## Suggested next steps (tomorrow, together)
1. You approve the task list (gate 1).
2. I port the remaining MBS/FEA keepers + author the rest of the feasible-now adds (URDF/YAML,
   checkpoint, coupling, ANCF FEA), each gate-verified, using DELTAS_10.md.
3. We do an expert-sign-off pass on a sample, then wire judge_v2's L1/L3 into the contract + cut a
   trial `v2.0-pychrono-10.0` after a small recalibration check.
