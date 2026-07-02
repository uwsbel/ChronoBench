# Scan of `uwsbel/sbel-reproducibility` for ChronoBench task sources

Read-only inventory produced via the `dan-github` access (GitHub API, no clone), 2026-07-02. Purpose:
harvest REAL, paper-backed scenarios as task *sources* (the scenario/prompt), from which we author clean,
gate-verified references, rather than authoring only artificial toys.

## Repo shape

- Year-organized (2020-2026), ~4,500 files, 520 MB. Mostly DATA (1,493 csv, 212 npy, images) and ANALYSIS
  (632 py, 92 MATLAB). Only ~45 `.py` actually `import pychrono`.
- **Key structural fact:** in the older projects the Chrono *simulations* are C++ (`chrono_scripts/*.cpp`),
  and the Python is calibration/plotting. This is what makes the **C++ -> PyChrono translation** task shape
  so well-supported here.
- Module footprint (code-search counts): vehicle 16, SCM deformable terrain ~19, sensor 7, FSI 4, FEA 1,
  robot 2. Dominant real-world domain = **terramechanics** (SBEL signature).
- `2026/chrono-code` (80 py) is the lab's own **agentic PyChrono code-generation system** (a peer S-LLM to
  what ChronoBench evaluates), NOT task source. Context only.

## Feasibility split (per the GPU policy)

- **CPU-now:** SCM deformable terrain (Bekker-Wong, CPU), rigid-terrain vehicles, MBS/FEA, small granular,
  and all C++/MJCF **translation** tasks (the target runs on CPU).
- **gpu-pending (author fully now incl. the CPU oracle; run later on the AMD/ROCm build):** CRM/SPH/FSI
  terrain, large DEM, ray-traced (OptiX) sensors.

## Publish vs. reserve (contamination policy)

Public = dev set; a subset is **RESERVED** (never committed to the public repo) as held-out eval material.
Rule of thumb: publish the SCM/CPU exemplars; reserve the FSI/CRM counterparts and a few whole scenarios.

## Candidate inventory (Chrono-simulation sources only)

| # | Source | Domain / axis | Lang | Feasible | Task shape | Oracle strategy | Publish/Reserve |
|---|--------|---------------|------|----------|-----------|-----------------|-----------------|
| 1 | (authored) plate-sinkage / bevameter on SCM | terramechanics: pressure-sinkage | PyChrono | CPU-now | create/modify/extend (load & soil sweeps) | Bekker-Wong pressure-sinkage (independent) + monotonic (softer/heavier -> deeper) | Publish (first task) |
| 2 | `2022/CRM2SCM_Paper/chrono_scripts/demo_ROBOT_Viper_SCM.cpp` | rover on SCM soft soil | C++ | CPU-now | C++ -> PyChrono + modernize (`SCMDeformableTerrain` -> `SCMTerrain`) | sinkage>0, advances; sinkage monotonic in soil softness | Publish (translation exemplar) |
| 3 | `2025/multi-terrain-RL` (Go2, Unitree A1 + URDF) | robotics: quadruped, URDF import | PyChrono | CPU-now (rigid/SMC) | direct port (URDF load + stand) | stands under gravity; feet in contact; base-height stable | Publish |
| 4 | `2022/HalfImplicit_JCND` | integrator/solver benchmark (MBS) | PyChrono | CPU-now | create + numerical-method | analytic MBS / energy invariants | Publish (candidate) |
| 5 | `2024/PathFollowingSim2real` | wheeled vehicle path following | PyChrono | CPU-now (rigid) | port | path-tracking error bound; reaches waypoint | Publish (candidate) |
| 6 | `2023/Unjhawala-IEEE-ExpressiveVM` | reduced-order vehicle model | py/C++ | CPU-now (rigid) | port/translation | steady-state / trajectory invariants | Reserve (candidate) |
| 7 | `2022/CRM2SCM_Paper/.../demo_FSI_SingleWheelTest.cpp` | single-wheel drawbar vs slip | C++ | gpu-pending | C++ -> PyChrono | drawbar-pull-vs-slip monotonic shape | Reserve (held-out) |
| 8 | `2022/CRM2SCM_Paper/.../demo_FSI_Bevameter.cpp` | bevameter (SPH) | C++ | gpu-pending | translation | Bekker band | Reserve |
| 9 | `2025/multi-terrain-RL/playground_crm.py`, `chrono_crmenv.py` | quadruped on CRM terrain | PyChrono | gpu-pending (CRM) | port | qualitative locomotion / stays upright | Reserve |
| 10 | `2022/M113_SPH_Terrain`, `2022/Polaris_SPH_Terrain` | tracked/wheeled vehicle on SPH | C++/py | gpu-pending | translation/port | momentum/energy + qualitative | Reserve |
| 11 | `2021/NASA-Project-Sims` (46 py + 33 cpp) | terramechanics (SCM + SPH) | C++/py | mixed | translation | sinkage / drawbar | Reserve (mine for pieces) |

Not candidates (Python but not Chrono simulation): ML projects (`FNODE`, `MNODE-code`, Bayesian
calibration, RL policies), all plotting/`plot_*.py`, and the `chrono-code` agent.

## Recommended order

1. **Plate-sinkage / bevameter on SCM** (row 1): first realistic task; CPU; cleanest independent oracle
   (Bekker-Wong) and the purest match to "lower bulk density -> more sinkage".
2. **Viper-on-SCM C++ -> PyChrono** (row 2): the translation+modernization exemplar, from a real repo file.
3. **Quadruped URDF import + stand** (row 3): robotics axis, real robot descriptions.

The `gpu-pending` rows (7-11) are authored later WITH their CPU oracles and run once the AMD/ROCm +
Vulkan-sensor track lands.
