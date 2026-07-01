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
4. **Discrimination demonstrated** on real generations (turn 1): good sample 100 (pass) / bad sample 75
   (invariant-fail, period 1.42 measured from CSV) / gpt-4o 0 and gpt-4o-mini 0 (L1 run:exception, both
   emitted PyChrono 9.0 API `Set_G_acc`/`ChVectorD` that does not exist in 10.0).

The other four verified tasks have their `input1.txt` but are NOT yet multi-turn and their contracts still
check the model's self-reported JSON (not yet CSV-derived). Applying the derived-observable + multi-turn
template to them is the next replication step.

## Tasks

| Task | Axis | Sim | State | Self-score | Notes |
|------|------|-----|-------|-----------|-------|
| pendulum | mechanism | pychrono | verified+3turn | 100 | PILOT COMPLETE; 3 turns; input1-3.txt; CSV-derived invariants; end-to-end generate->judge proven |
| slider_crank | mechanism | pychrono | verified | 100 | closed loop; piston stroke 0.8 = 2*crank_rad |
| gear | mechanism | pychrono | pending | | KEEP-port |
| mass_spring_damper | mechanism | pychrono | verified | 100 | period_d 0.6315 + zeta 0.1 match analytic |
| beam | FEA | pychrono | verified | 100 | clean cantilever; tip deflection 0.0048 = FL^3/3EI (static solve) |
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
