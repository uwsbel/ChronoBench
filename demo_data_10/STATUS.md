# demo_data_10 build ledger (v2.0 / PyChrono 10.0 redesign)

Resumable status of the redesigned task suite on branch `chrono10-redesign`. A fresh session resumes
from this file. State per task: `pending` (planned) / `authored` (files written) / `verified` (passes
the de-scoped gate: L1 execution on pychrono10 + minimal L2 + L3 invariant via `scoring/judge_v2.py`)
/ `blocked` (infra-gated or unresolved). Gate self-check command (add `--turn N` for multi-turn tasks):
`conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/<task> [--turn N]`

## Pilot end-to-end (2026-07-01): `pendulum` closes the full loop

The `pendulum` task now runs the COMPLETE benchmark loop, prompt -> generate -> extract -> judge, and is
the template for the rest:
1. **Asking half added.** `input1.txt` (turn-1 prompt) written for `pendulum` AND the other four verified
   tasks (`mass_spring_damper`, `swig_contact_reporter`, `slider_crank`, `beam`). Reusable turn-1 driver
   `scoring/generate_one.py` (build_turn1 + provider caller + extractPy) generates one candidate per task.
2. **Judge hardened (`scoring/judge_v2.py`).** (a) CSV-DERIVED observables: the graded physics is now
   MEASURED by the judge from the candidate's emitted trajectory (`derive` block), not trusted from the
   model's self-reported JSON, closing the "print the analytic answer" gaming hole. (b) Multi-turn `turns`
   in `contract.json` (select with `--turn`), backward-compatible with the flat single-turn contracts.
3. **`pendulum` is now 3 turns**, each authored + gate-verified at 100: turn 1 create (small-angle period),
   turn 2 modify (large-angle 60deg: period 2.12 s + amplitude), turn 3 extend (double pendulum: energy
   drift 0.13% + second-arm swing). `pyinput2.py`=truth1, `pyinput3.py`=truth2.
4. **Discrimination demonstrated** on real generations (turn 1): good sample 100 (pass) / bad sample 40
   (invariant-fail, capped; period 1.42 measured from CSV) / gpt-4o 0 and gpt-4o-mini 0 (L1 run:exception,
   both emitted PyChrono 9.0 API `Set_G_acc`/`ChVectorD` that does not exist in 10.0).

## Second task + oracle upgrade (2026-07-01): `mass_spring_damper` done fully

Following "one done well before breadth", `mass_spring_damper` is now a full 3-turn, oracle-grounded task,
and the judge/methodology gained three things (also applied to `pendulum`):
1. **Independent-oracle ground truth.** Targets come from `demo_data_10/<task>/oracle.py` (pure Python, NO
   Chrono): the governing equations solved closed-form + high-fidelity RK4, run offline once, kept in-repo.
   Tests "matches the true physics", not "matches our Chrono run"; the Chrono `truth{t}.py` is validated to
   AGREE with the oracle (two-way check, agrees to ~4 sig figs).
2. **Tunable scoring (`contract.json -> scoring`).** weights L1/L2/L3 = 0.30/0.20/0.50 + `invariant_fail_cap`
   = 40 (any failed invariant caps the score); per-task/per-turn override, global default in `judge_v2.py`.
   A wrong-physics candidate now scores 40, not 75.
3. **New derived observables + a bug fix.** `log_decrement` (damping ratio) and a `t_min` tail window
   (steady-state amplitude). The oracle also exposed a period-measurement bug: cross the known equilibrium
   (0), NOT the empirical mean (mean-crossing biased a DECAYING signal's period by 1.3-6.8%); fixed in
   `judge_v2.py:_period`.

`mass_spring_damper` turns: 1 create (c=2, zeta=0.1, Td 0.6315) / 2 modify (c=6, zeta=0.3, Td 0.6587) /
3 extend (resonant forcing F=sin(10 t), steady-state amp 0.0167). All gate-verify 100; good sample 100,
bad sample (k=400 typo) 40. `pendulum` re-aligned: turn-2 target set to the oracle's elliptic-integral
period 2.153 (was the Chrono value 2.122), tolerance widened to cover Chrono's numerical damping.

## Third task done fully (2026-07-02): `beam` (FEA/static, oracle = independent FE solver)

`beam` is now a full 3-turn, oracle-grounded FEA task, the static/FEA stress-test of the template:
1. **Oracle is an independent FE SOLVER, not just a formula**: `beam/oracle.py` (numpy, NO Chrono) is an
   Euler-Bernoulli Hermite-beam cantilever solver refined to N=50, cross-checked against the textbook
   closed forms; the Chrono reference agrees with it (two-way: tip load exact, self-weight +0.33%).
2. **Un-gameable static scalar**: each turn logs the full deflected shape (`x,y` per node) to out.csv and
   the judge derives the tip deflection as `max_abs(y)`, so a model must produce a consistent deflected
   curve, not just print a number.
3. Turns: 1 tip point load (0.0048) / 2 self-weight via automatic gravity (0.0023) / 3 tip + mid-span
   superposition (0.0063). All gate-verify 100; good sample (N=8) 100, bad sample (E=2e11 units slip) 40.
4. **FEA-dynamics finding (documented, deferred)**: a first-natural-frequency turn was dropped because
   PyChrono's `modal` module is not built here and a free-vibration transient is timestepper-biased (HHT
   damping shifts f1 ~9% low; Newmark/Trapezoidal go unstable and segfault). Static superposition keeps
   turn 3 oracle-exact. See `beam/CONTRACT.md`.

Three tasks (`pendulum`, `mass_spring_damper`, `beam`) are now fully template-complete across mechanism,
damped/forced oscillator, and FEA/static axes. The remaining verified tasks (`slider_crank`,
`swig_contact_reporter`) have `input1.txt` but are NOT yet multi-turn / oracle-grounded, that is the
breadth step.

## Tasks

| Task | Axis | Sim | State | Self-score | Notes |
|------|------|-----|-------|-----------|-------|
| pendulum | mechanism | pychrono | verified+3turn | 100 | PILOT COMPLETE; 3 turns; input1-3.txt; CSV-derived invariants; end-to-end generate->judge proven |
| slider_crank | mechanism | pychrono | verified | 100 | closed loop; piston stroke 0.8 = 2*crank_rad |
| gear | mechanism | pychrono | pending | | KEEP-port |
| mass_spring_damper | mechanism | pychrono | verified+3turn | 100 | DONE FULLY; 3 turns; oracle-grounded (oracle.py); derived period_d/zeta/ss_amp; good 100 / bad 40 |
| beam | FEA | pychrono | verified+3turn | 100 | DONE FULLY; 3 turns (tip load/self-weight/superposition); numpy-FE oracle; tip DERIVED from logged shape; good 100 / bad 40 |
| cable | FEA | pychrono | pending | | KEEP-port; ANCF cable |
| buckling | FEA | pychrono | pending | | KEEP-port |
| rotor | FEA | pychrono | pending | | KEEP-port; IGA rotor |
| tablecloth | FEA | pychrono | pending | | KEEP-port; BST shells |
| fea_ancf_beam (new) | FEA | pychrono | pending | | ADD; ANCF cantilever, analytic static deflection |
| solver_nsc_smc (new) | solver/contact | pychrono | pending | | ADD; NSC vs SMC restitution invariant |
| coupling_rigid_flex (new) | coupling | pychrono | pending | | ADD; rigid body on flexible beam |
| swig_contact_reporter (new) | swig-extension | pychrono | verified | 100 | callback+lifecycle; 4 contacts, normal_force 39.26~=N*m*g |
| import_urdf (new) | data-import | pychrono | pending | | ADD; URDF load + named-joint actuation |
| yaml_mbs (new) | data-import | pychrono | pending | | ADD; YAML declarative model |
| checkpoint (new) | state-mgmt | pychrono | pending | | ADD; checkpoint/restart determinism |
| hmmwv | vehicle | pychrono | pending | | KEEP-port; driver+datapath+VSG deltas |
| m113 | vehicle(tracked) | pychrono | pending | | KEEP-port |
| (1 car: sedan or citybus) | vehicle | pychrono | pending | | KEEP-port; Dan picks which |
| gps_imu | sensor | pychrono | pending | | KEEP-port; GPS/IMU (no OptiX) |
| turtlebot | robotics | pychrono | pending | | KEEP-port; rover (keep only for URDF/control) |

Ground-locomotion cap (<=3-4 total): hmmwv + m113 + one car + at most one rover. `scm`/`rigid_multipatches`/
`curiosity` are candidates to drop or re-tag under the tightened cap; decide with the task-list approval.

## DEFERRED (infra-gated) -- NOT authored, need infra (see SUITE_DESIGN.md + PANEL_REDTEAM.md)

OptiX sensors (camera/lidar/radar), ROS (vehros/sensros/multi-agent), FSI/SPH, PyDEME/DEM
(pile/cone-penetration/mixer). Designed; await GPU/ROS/PyDEME env.

## Gates awaiting Dan (see HANDOFF.md)

1. Approve/adjust the finalized task list (which car; final ground-locomotion set; CAD include/defer).
2. Expert sign-off on a per-axis sample of references.
3. Recalibration of the de-scoped judge vs human judgment.
