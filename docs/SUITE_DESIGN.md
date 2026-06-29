# ChronoBench task-suite redesign: blueprint (DRAFT for review)

Status: Phase 1/2 draft on branch `chrono10-redesign`. This document is the design + coverage record
for the redesigned, multi-simulator (PyChrono 10.0 + PyDEME) task suite. It records *why* each task
is in the suite (axis, simulator, source, rationale), fixing the provenance void of the original 34.
Nothing here is authored or frozen yet; this is the plan for the panel red-team (Phase 3) and
authoring/porting (Phase 4) to execute against.

## Method (bias-aware)

1. Coverage is defined by the authoritative capability taxonomy (14 axes derived from Chrono 10.0.0's
   own `src/demos/python`, ~113 demos), not one author's taste. Every task maps to an axis.
2. Each task's reference is sourced from / cross-checked against an authoritative upstream demo or
   `uwsbel/sbel-reproducibility` scenario; correctness by execution + structural checks, not opinion.
3. Balance: VEH capped at <= ~20-25% of feasible tasks; every in-scope axis gets coverage; difficulty
   is staged across the 3-turn structure (turn 1 basic, turns 2-3 add genuinely new capability).
4. Visualization standardizes on VSG; Irrlicht is legacy (migration is 1:1, see below).
5. Feasibility is honest: this machine has no NVIDIA GPU / CUDA and the `ros` module is not built, so
   GPU/ROS axes are designed-but-deferred (infra-gated), not dropped.

## Feasibility (this machine: pychrono10, CPU-only, VSG ok, no GPU/CUDA, no ROS)

| Class | Examples | Status |
|---|---|---|
| CPU PyChrono | MBS, FEA, VEH (rigid + SCM), rigid robots + IK, GPS/IMU, URDF/YAML import, checkpoint | Feasible now |
| GPU PyChrono | OptiX sensors (camera/lidar/radar), FSI/SPH, CRM terrain | Infra-gated |
| ROS | vehros, sensros, multi-agent | Infra-gated (`ros` not built) |
| PyDEME (separate sim) | granular: ball-drop, cone-penetration, mixer, wheel-DP | Infra-gated (Linux/WSL+CUDA+GPU) |

## Capability axes (authoritative) and target coverage

Legend: F = feasible now, G = GPU-gated, R = ROS-gated, D = PyDEME/GPU-gated.

| # | Axis | Tier range | Target tasks | Status |
|---|------|-----------|--------------|--------|
| 1 | Mechanism kinematics & constraints | basic->adv | 4-5 | F |
| 2 | Collision & contact (NSC/SMC, trimesh, custom) | basic->adv | 2 | F |
| 3 | FEA structural (Euler/ANCF/IGA/shells/tetra/cable/contact/rotor) | basic->adv | 6-7 | F |
| 4 | Vehicle dynamics & terrain (rigid/SCM; CRM=G) | basic->adv | 5 (capped) | F (CRM=G) |
| 5 | Sensors & perception (GPS/IMU=F; camera/lidar/radar=G) | basic->adv | 1 F + 3 G | mixed |
| 6 | Robotics & locomotion (mobile/rover/quadruped/industrial-IK) | basic->adv | 3-4 | F |
| 7 | Multi-agent / ROS | inter->adv | 2 | R |
| 8 | FSI / SPH | adv | 1-2 | G |
| 9 | DEM / granular (PyDEME) | basic->adv | 2-3 | D |
| 10 | Data-driven import (URDF/YAML; CAD=caveat) | inter | 2 (+CAD?) | F (CAD caveat) |
| 11 | Visualization / postprocess | basic | 0-1 | F (low value) |
| 12 | Control / RL | adv | 0 (separate track) | deferred |
| 13 | State mgmt (checkpoint/output) | inter | 1 | F |
| 14 | Solvers / integrators | basic->adv | folded into axis 2 | F |

Feasible-now target ~= 24-27 tasks; infra-gated designed-but-deferred ~= 8-10. VEH share ~= 5/26 ~= 19%
(under cap), down from 41% in v1.

## Keep / drop / add (per system)

### KEEP (port to 10.0 + Irrlicht->VSG), the genuinely-distinct existing tasks

| System | Axis | Why kept |
|---|---|---|
| pendulum, slider_crank, gear, mass_spring_damper | 1 | distinct constraint/force mechanics |
| particles | 2/9-ish | particle emitter (PyChrono); note real granular is the new PyDEME track |
| beam, buckling, rotor, tablecloth, cable | 3 | 5 distinct FEA element families (Euler, IGA-buckling, IGA-rotor, BST shell, ANCF cable) |
| hmmwv | 4 | baseline wheeled; turns add path controller + custom driver |
| rigid_multipatches | 4 | complex/multi-patch terrain |
| scm | 4 | SCM deformable terrain (+ sensors at turn 3); CPU-feasible |
| m113 | 4 | tracked vehicle (distinct dynamics) |
| sedan (or citybus) | 4 | one representative wheeled passenger vehicle |
| gps_imu | 5 | GPS/IMU sensor fusion (no ray-tracing; feasible) |
| turtlebot, curiosity | 6 | mobile + rover archetypes |

### DROP (redundant; preserved at tag `paper-ieee-access-2026`)

citybus/feda/gator/kraz/art/uazbus/man (redundant wheeled "drive model X"), scm_hill (param-variant of
scm), viper (rover redundant with turtlebot+curiosity), handler (minimal ROS stub), veh_app (vehicle+
sensor dup). camera/lidar/sensros/vehros are NOT dropped but move to infra-gated (below).

### ADD (new tasks for uncovered axes; sourced from authoritative demos / sbel-reproducibility)

| New task | Axis | Simulator | Source | Status |
|---|---|---|---|---|
| NSC-vs-SMC collision + trimesh/custom-contact | 2 | PyChrono | demo_MBS_collision{NSC,SMC,trimesh}, demo_MBS_custom_contact | F |
| ANCF cantilever beam | 3 | PyChrono | demo_FEA_beamsANCF | F |
| 3D-solid FEA (tetra/brick) | 3 | PyChrono | demo_FEA_tetra / demo_FEA_brick | F |
| FEA-rigid contact | 3 | PyChrono | demo_FEA_contacts | F |
| Industrial robot IK | 6 | PyChrono | demo_ROBOT_Industrial | F |
| URDF model import | 10 | PyChrono | demo_PARSER_URDF | F |
| YAML declarative model | 10 | PyChrono | demo_YAML_mbs(+controller) | F |
| Checkpoint / structured output | 13 | PyChrono | demo_MBS_checkpoint + ChOutput | F |
| Physics-based camera / path-tracing | 5 | PyChrono | demo_SEN_camera | G |
| LiDAR point cloud | 5 | PyChrono | demo_SEN_lidar | G |
| Multi-agent ROS (two-manager) | 7 | PyChrono | demo_ROS_two_managers | R |
| FSI/SPH object drop | 8 | PyChrono | demo_FSI_ObjectDrop | G |
| Granular settling/pile | 9 | PyDEME | pyDEME_BallDrop | D |
| Cone penetration | 9 | PyDEME | pyDEME_ConePenetration | D |
| Bladed mixer (arbitrary mesh motion) | 9 | PyDEME | pyDEME_Mixer | D |

## VSG migration (applies to all KEEP + new PyChrono tasks)

Replace `chronoirr.ChVisualSystemIrrlicht` -> `vsg.ChVisualSystemVSG`;
`veh.ChWheeledVehicleVisualSystemIrrlicht` -> `veh.ChWheeledVehicleVisualSystemVSG`;
`veh.ChTrackedVehicleVisualSystemIrrlicht` -> `veh.ChTrackedVehicleVisualSystemVSG` (all present in 10.0).
Minor deltas: window size as `chrono.ChVector2i(w,h)`; `AddSkyBox()` -> `SetSkyBoxTexture(path)`;
`AddTypicalLights()` -> `SetLightIntensity()`+`SetLightDirection()`. Verification stays headless, so the
Vulkan runtime is not required to gate a task. Sensor/ROS systems have no main vis system (no change).

## Open design questions (for the panel red-team / Dan)

1. Target total: ~26 feasible-now + ~10 infra-gated. Comfortable, or trim/expand?
2. VEH cap: keep 5 (hmmwv, rigid_multipatches, scm, m113, + one car) at ~19%? Which car, sedan or citybus?
3. Do we keep `particles` (PyChrono emitter) once the PyDEME granular track exists, or retire it as
   superseded?
4. Difficulty staging: keep the 3-turn create/modify/extend structure for all axes, or vary it?
5. CAD/STEP import (CASCADE): include despite the conda-packaging caveat, or defer?
6. Should infra-gated tasks be authored now (designed, marked pending-verify) or deferred until the
   GPU/ROS/PyDEME envs exist?
