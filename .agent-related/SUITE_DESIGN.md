# ChronoBench task-suite redesign: blueprint (DRAFT for review)

> **WORK IN PROGRESS.** The suite is being redesigned. Only the feasible-now (CPU PyChrono) tasks
> are authored + verified first; the GPU/ROS/PyDEME tracks are a DEFERRED BACKLOG (see "ACTION
> NEEDED" below) and are NOT yet done, NOT verified, and NOT in the contract. The suite is not
> complete until that backlog is cleared. v1.0/paper stays frozen at tag `paper-ieee-access-2026`.

Status: Phase 1/2 draft on branch `chrono10-redesign`. This document is the design + coverage record
for the redesigned, multi-simulator (PyChrono 10.0 + PyDEME) task suite. It records *why* each task
is in the suite (axis, simulator, source, rationale), fixing the provenance void of the original 34.
Nothing here is authored or frozen yet; this is the plan for the panel red-team (Phase 3) and
authoring/porting (Phase 4) to execute against.

## Post-panel architecture (ADOPTED 2026-06-29) -- governs over the per-axis draft below

The Phase 3 red-team (see `PANEL_REDTEAM.md`) reshaped the design. Where this section conflicts with
the per-axis draft further down, THIS governs. Net: the benchmark becomes a failure-mode-probing,
contracted-virtual-experiment suite with a layered oracle, not a per-axis demo-coverage set.

### 1. Every task is a contracted virtual experiment
Each task ships an executable contract: prompt parameters (masses, rates, seeds, asset paths, final
time); required API/topology features; execution requirements (headless, no blocking vis, no absolute
paths, bounded runtime, pinned 10.0); an output schema (JSON/CSV with units); a behavioral oracle
(analytic checks, tolerances, inequalities, metamorphic relations); and per-task failure-triage
(import / construct / run / missing-output / contract-violation / required-feature-absent).

### 2. Judge evolution -- DE-SCOPED (post-panel honest review; NOT the full layered oracle)
On review, the full three-layer oracle was de-scoped: a full L2 "required-API/topology" layer is
itself a new rigid idiom-bias vector, and a full rewrite forfeits the v1.0 judge's human calibration.
Adopted instead:
- **L1 Execution integrity (now):** imports, constructs, runs to horizon under timeout, no NaN/segfault.
  Objective, low-bias, hard-to-game, and the main win the text-only judge lacks.
- **L3 Behavioral invariants (selective):** only where physics gives clean checks (mechanism
  period/energy, FEA static displacement/frequency, monotone slip/sinkage); skip chaotic/contact.
  Parameters varied via the PROMPT, never by editing generated code.
- **L2 Semantic checks (MINIMAL):** "necessary capability present" only (e.g. a sensor was attached),
  never "preferred idiom used".
- **Keep the existing reference+api rubric LLM** for residual / partial-credit scoring.
- **Recalibrate** the combined judge vs human expert judgment (as v1.0 was), else more thorough but
  less validated.
Validate this de-scoped judge on the Phase 0 pilot before scaling.

### 3. Axes reframed around failure modes (supersedes the 14-axis coverage table below)
First-class, separately scored: (a) mechanism/constraint + frame/marker reasoning; (b) collision/
contact NSC vs SMC + trimesh/custom; (c) FEA incl. ANCF/shells/3D-solid + FEA-contact; (d) SOLVER/
integrator/timestepper policy (NEW; no longer folded into contact); (e) SWIG Python/C++ extension --
callbacks, ChFunction subclassing, GC/lifecycle (NEW; >=1 mandatory task); (f) cross-domain COUPLING
(rigid-flex, FEA-contact, vehicle-terrain-sensor, robot-env) -- REQUIRED, not optional; (g) data-driven
import + POST-import actuation (URDF/YAML); (h) sensors GPS/IMU (camera/lidar deferred-GPU); (i)
STATE-management / reproducible logging -- seed, checkpoint/restart, schema (NEW first-class);
(j) ground locomotion. Scale is tested via config-on-small-N (require `ChSystemMulticore` /
broadphase / solver caps), NOT large-N execution. Solvers/callbacks/instrumentation also act as
cross-cutting subscores.

### 4. Axis-specific staging (supersedes uniform create->modify->extend); may include repair
E.g. mechanisms: build topology -> validate an analytic quantity -> alter frame preserving the metric;
contact/numerics: build or REPAIR a degraded model -> compare NSC/SMC -> couple/scale-config; FEA:
build -> tune solver/timestep + validate displacement/frequency -> couple to rigid/contact;
import/robotics: import -> actuate named joints + validate frames -> task motion + logging; callbacks:
implement -> prove invocation/lifecycle -> use for state-dependent control. Repair stages (diagnose +
fix a degraded script) are included for contact/numerics/import/state, kept focused so they probe
domain knowledge, not generic code-editing.

### 5. Redundancy cap tightened
ALL ground locomotion (vehicles + rovers + mobile robots) <= 3-4 tasks TOTAL (not VEH-only at ~19%).
At most ~2 Chrono::Vehicle wrappers; a rover only if it probes URDF/import/control/sensors, not
rolling dynamics. Freed slots go to solvers, coupling, SWIG, and state-management. Redundancy is judged
on PROBE VECTORS (API family, numerics risk, coupling, observables, extension mode), not scenario
names: two tasks with the same probe vector collapse even if the stories differ.

## Phase 0 pilot result (PASSED 2026-06-29)

The de-scoped judge was validated end-to-end on a pendulum task (`demo_data_10/pendulum/`, judge at
`scoring/judge_pilot.py`): the reference and a correct-but-DIFFERENT-style candidate both score 100
(idiom divergence is not penalized), while a runs-but-WRONG candidate (L=0.5 m -> period 1.42 s vs
analytic 2.01 s) that passes L1 execution and every L2 capability check is caught by L3
(`invariant-fail`, 65). This confirms the architecture is both more thorough (L3 catches finite-but-
wrong physics a text-only judge would miss) and unbiased on the execution/behavioral axis
(style-divergent-but-correct scores identically to the reference). Cleared to scale to Phase 4.

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

## Decisions (Dan, this session)

1. Infra-gated tasks: **DEFER**. Author + gate-verify the feasible-now suite and cut the contract on
   those only; the infra-gated axes are a designed backlog (below) to author + verify when the env
   exists. Clear in-progress signage stays so a future visitor knows the suite is not yet complete.
2. `particles`: **KEEP** (the PyChrono emitter / N-body path is distinct from the PyDEME GPU-DEM track).

## DEFERRED BACKLOG -- ACTION NEEDED (suite is NOT complete until cleared)

These axes are part of the intended suite but are NOT yet authored or verified: this machine has no
NVIDIA GPU / CUDA and no ROS build. They are DESIGNED only, NOT in the v2.0 contract, until each is
stood up, executed, and gated. `STATUS.md` flags them `deferred(infra)`.

| Axis / track | Tasks | Infra required |
|---|---|---|
| Sensors (OptiX) | physics camera/path-tracing, LiDAR, radar | NVIDIA GPU + Chrono::Sensor built with OptiX |
| Multi-agent / ROS | two-manager ROS, vehros, sensros | pychrono built with the `ros` module + a ROS env |
| FSI / SPH | object drop, CRM terrain | NVIDIA GPU build |
| DEM / granular (PyDEME) | granular pile, cone-penetration, mixer | Linux/WSL + CUDA 12.x + NVIDIA GPU + `pip install DEME` |

## Still-open design questions (for the panel red-team)

1. Target total ~26 feasible-now: comfortable, or trim/expand?
2. VEH cap at 5 (~19%): which representative car, sedan or citybus?
3. Difficulty staging: keep the 3-turn create/modify/extend structure for all axes, or vary it?
4. CAD/STEP import (CASCADE): include despite the conda-packaging caveat, or defer?
