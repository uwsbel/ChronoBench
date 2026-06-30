# demo_data_10 build ledger (v2.0 / PyChrono 10.0 redesign)

Resumable status of the redesigned task suite on branch `chrono10-redesign`. A fresh session resumes
from this file. State per task: `pending` (planned) / `authored` (files written) / `verified` (passes
the de-scoped gate: L1 execution on pychrono10 + minimal L2 + L3 invariant via `scoring/judge_v2.py`)
/ `blocked` (infra-gated or unresolved). Gate self-check command:
`conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/<task>`

## Tasks

| Task | Axis | Sim | State | Self-score | Notes |
|------|------|-----|-------|-----------|-------|
| pendulum | mechanism | pychrono | verified | 100 | pilot; period 2.007 vs 2.006; reference + 2 samples |
| slider_crank | mechanism | pychrono | pending | | KEEP-port |
| gear | mechanism | pychrono | pending | | KEEP-port |
| mass_spring_damper | mechanism | pychrono | verified | 100 | period_d 0.6315 + zeta 0.1 match analytic |
| beam | FEA | pychrono | pending | | KEEP-port; Euler cantilever tip-deflection |
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
