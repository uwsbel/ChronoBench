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
damped/forced oscillator, and FEA/static axes.

## Breadth complete (2026-07-02): all 5 originally-verified tasks fully template-complete

`slider_crank` and `swig_contact_reporter` are now full 3-turn, oracle-grounded tasks matching the
pattern, so all five verified tasks are done:
- `slider_crank`: analytic slider-crank kinematics oracle; turns stroke(r=0.4) / stroke(r=0.6) /
  drive-speed(omega=2*pi)+peak piston speed; derived stroke (range) + peak_speed (max|dx/dt|). Added
  judge derive kind `max_speed`.
- `swig_contact_reporter`: static-equilibrium oracle; turns N=4 / N=6 / heavier(m=2)+per-contact; the
  callback logs each contact's force to out.csv and the judge derives count + sum + per-contact max.
  Added judge derive kinds `count` and `sum`.
All five tasks: references gate-verify 100 across their turns; good sample ~100, bad sample capped at 40.

The v2 suite is now runnable through ONE main-package command (wraps `judge_v2` over every task x turn):
`conda run -n pychrono10 python -m chronobench.score_v2` self-checks the references (currently 15/15
turns pass); `--candidates DIR` scores an agent's `DIR/<task>/turn{N}.py` outputs; `--task NAME` limits to
one task; `--json FILE` dumps full results. Remaining: the ~12 pending tasks (breadth).

## Realistic-task phase started (2026-07-02): sbel-reproducibility sourcing + contamination insurance + first terramechanics task

Shift from artificial tasks to REAL, paper-backed scenarios (see `.agent-related/REPRO_SCAN.md`, the classified scan
of `uwsbel/sbel-reproducibility`). Contamination insurance in place (cheap-now posture): `CANARY.md` (fixed
GUID) + a README curator marker; tasks authored PARAMETER-FIRST (each turn's `params` block is the single
declared source; the oracle is a pure function of them, so eval-time randomization stays free); a
publish-vs-reserve column in the scan. Deferred: the randomized-instance eval harness, contamination
detection, private leaderboard.

First realistic task done: **`plate_sinkage_scm`** (bevameter / plate-sinkage on SCM deformable terrain,
CPU). It is the template's first outing on a domain with NO tight closed-form oracle: the independent
Bekker-Wong oracle sets a COARSE band [0.5x, 2.5x] (SCM sinkage measured ~1.0-1.5x ideal Bekker), and the
"softer soil / heavier load -> deeper" law is encoded ACROSS turns via the shifting band. Confirmed SCM
works on this CPU (`veh.SCMTerrain`; `SetCollisionSystemType` required before constructing it, a real 10.0
gotcha). Added judge derive kind `monotonic` (a stability check; not used in the final contract because a
settled plate jitters at equilibrium under SMC contact, so within-run monotonicity is ~0.5 and invalid;
the min/max band + cross-turn shifts do the work). Full suite now 18/18 turns pass.

## Breadth batch 1 (2026-07-03): gear + fea_ancf_beam + solver_nsc_smc done fully

Three more artificial keepers to the full template (suite 6 -> 9 tasks; self-check 27/27), driven
by the paper_plan.md sequencing (authoring precedes the E1 ranking run):
- `gear`: closed-form rigid-transmission kinematics oracle (external mesh reverses sense; two-way
  agreement ~13 digits); turns 2:1 / 3:1 / two-stage where the POSITIVE w3 sign discriminates.
  TWO verified API findings: `ChLinkLockPulley` enforces w_out/w_in = tau + 2 (NOT the textbook
  rp1/rp2; the belt turn was redesigned to a second gear mesh), and `ChLinkLockGear` phase capture
  can take ~1 s (measure a t >= 2 s tail; the 0.5 s window mis-graded a correct mirror-mounted
  control at 40). Both in `docs/DELTAS_10.md`.
- `fea_ancf_beam`: ANCF gradient-deficient cable element (nonlinear static solve) vs the
  independent Euler-Bernoulli Hermite-FE oracle; references agree 0.002%/0.03%/0.003% across
  tip-load / self-weight / tip+mid-span turns. Complements `beam` (same physics discipline,
  different element technology). `DoStaticNonlinear(100)` + Pardiso converged; no fallback needed.
- `solver_nsc_smc`: solver/contact-method axis; exact impact-kinematics oracle (apex = e^2*h0);
  turns NSC e=0.7 / NSC e=0.9 / switch-to-SMC. TWO verified solver findings: NSC restitution has
  an isolated bad dt pocket (1e-4 rebounds at 54% of ideal while 1e-3...2e-4 and 2e-5 are exact;
  the task pins dt=2e-4), and the SMC default contact stiffness lets the ball sink ~9 cm (the
  reference sets E_contact=1e8; a penetration invariant catches the soft default). The bad control
  calls SetRestitution on an UNUSED material: passes the L2 text check, only measured L3 catches it.

## Pilot A/B (2026-07-04): chrono-rag retrieval A/B x 3 emitters, with timing (dev-set dry run)

First end-to-end run of the full generate -> extract -> judge loop at suite scale, following the
A/B brief (same agent scored twice, base vs chrono-rag-augmented prompt; ONLY the RAG variable
changes; 10.0 suite + judge_v2 only; k = 3 reps; turn 1 x 9 tasks; dev-set caveat per CANARY.md).
Driver: `scoring/generate_suite.py` (arms x agents x reps, cached per-task retrieval, per-call
timing manifest, parallel gen + judge). Artifacts: `runs/pilot-2026-07-04/` (untracked;
`report.md`, `manifest.jsonl`, all candidates + verdicts); ARCHIVED wholesale to Box:
`Box\ChronoBench\runs\pilot-2026-07-04.zip` (0.6 MB, SHA256-verified copy via the Box Drive
mount; the lab convention for benchmark-run archives going forward).

| agent | arm | mean score | pass | dead-9.0-API hits |
|---|---|---|---|---|
| gpt-5.5 | base | 41.5 | 10/27 | 3 |
| gpt-5.5 | rag | 75.6 | 18/27 | 3 |
| gpt-4o | base | 0.0 | 0/27 | 27 |
| gpt-4o | rag | 0.0 | 0/27 | 25 |
| claude-opus-4-8 (subscription CLI) | base | 74.8 | 19/27 | 1 |
| claude-opus-4-8 (subscription CLI) | rag | 74.8 | 19/27 | 0 |

Findings:
1. **RAG value is strongly model-dependent.** gpt-5.5 gains +34.1 points (run-failures 14 -> 3:
   retrieval fixes its API errors). Opus 4.8 is RAG-neutral on score (already writes ~correct 10.0
   API; its one dead-API slip disappears with RAG). gpt-4o is NOT rescued: 0.0 both arms, dead
   9.0 API in 25-27/27 candidates EVEN WITH the correct 10.0 excerpts in context; a strong stale
   prior beats reference material.
2. **Multi-turn shakeout (rider):** gpt-5.5 base on turns 2-3 (given the correct prior-turn
   script): 16/18 pass vs 10/27 on de-novo turn 1: modify/extend from working 10.0 code largely
   neutralizes version drift. The full 3-turn machinery (pyinput chaining, --turn judging) works.
3. **Harness lessons (fixed in this commit):** candidates ignore "headless" and trigger GUI/DLL
   error DIALOGS that block judging: judge_v2 now suppresses Windows loader/crash dialogs, spawns
   candidates with no window, and sets SDL's dummy video driver. `pychrono.vehicle` imports only
   under the ACTIVATED conda env, so judging must go through `conda run` (see docs/DELTAS_10.md).
4. **Timing (planning info):** generation mean/call: gpt-5.5 27 s, gpt-4o 9 s, Opus-via-CLI 287 s
   (the subscription path is the wall-clock bottleneck; 54 calls ~ 4.3 h at 2-way concurrency).
   Judging: 5.3 s/candidate mean, 14.3 min compute for 162 candidates (4 workers).
5. Decoding: temperature 0 for gpt-4o; gpt-5.5 (reasoning API) and the Opus CLI expose no
   temperature control, so those arms rely on the 3 reps (documented protocol deviation).

## Breadth batch 2 (2026-07-04): import_urdf + yaml_mbs + hmmwv_scm done fully

Dan's picks (URDF, YAML, HMMWV-on-SCM; the vehicle gate is settled: HMMWV). Suite 9 -> 12 tasks:
- `import_urdf`: ships its own parameter-first asset (`assets/pendulum.urdf`); judge upgrade:
  `judge_v2` stages a task's `assets/` into the run dir (generic). Turns: named-joint POSITION
  sine actuation (closed-form amplitude/period) / actuation change / free swing whose period
  tests the IMPORTED inertials (1.638 s vs oracle 1.6420). The shipped `demo_PARSER_URDF.py`
  uses a stale 3-arg ChFunctionSine signature (noted); the bad control swaps the two args.
- `yaml_mbs`: the candidate script AUTHORS the model/simulation/solver YAML inline and drives it
  through `ChParserMbsYAML`; physics = inline slider-crank identities (stroke 2r; peak speed).
  Schema findings in `docs/DELTAS_10.md`: wrapper takes (sim_yaml[, verbose]), refs resolve
  against the sim file's dir, DISTANCE points are GLOBAL assembly coordinates (the body-local
  misreading = frozen slider = the shipped bad control).
- `hmmwv_scm`: vehicle-terramechanics coupling; turns rigid baseline / SCM firm / SCM soft.
  Rut bands Bekker-anchored (static cylinder solve 0.043/0.103 m; measured 0.116/0.242, i.e.
  2.3-2.7x static, slip-sinkage + multi-pass); speed carries the motion-resistance law (6.58 /
  4.54 / 4.05 m/s; DISTANCE does not discriminate rigid-vs-firm, 11.39 vs 11.79 m, less
  wheelspin on SCM). Monotonic-progress derive is windowed past the launch transient (settle
  jitter gave 0.933; a full-window check mis-graded correct runs).

## AMD GPU unblock (2026-07-04): FSI + CRM now run from Python on this machine; ROS investigated

Upstream PRs #759 (HIP backend on native Windows; Dan's) and #760 (VSG+HIP fix) merged today; no
conda package carries them yet, so PyChrono was SOURCE-BUILT here (recipe: `docs/BUILD_HIP.md`):
`WinRepos/chrono` main @ 178fb99f61, ROCm clang toolchain (HIP SDK 7.1, gfx1151 Strix Halo),
existing `build_hip` tree + `CH_ENABLE_MODULE_PYTHON=ON` (SWIG 4.4.1 from the new `chrono-build`
conda env). One portability fix was needed and committed locally in the chrono clone as an
upstream-PR candidate (`/DWNT` -> `WNT` in the SWIG targets, 5623ad893b). VALIDATED headless from
Python on the GPU: FSI-SPH ObjectDrop (1 s sim / 76.6 s wall, 8,003 steps) and Viper-on-CRM
(3 s sim / 50.6 s wall; needed a one-line demo fix, the stale `EnableCudaErrorCheck` name, see
DELTAS). Consequences:
1. The fsi_*/crm_* task family is UNBLOCKED (next authoring batch); those tasks will pin the NEW
   environment (chrono-build + build_hip), while the 12 existing tasks stay on `pychrono10`
   (36/36 unaffected).
2. Chrono::Sensor remains the only gated module on this machine.
3. ROS on Windows investigated (`docs/ROS_WINDOWS.md`): no ROS on this box; Windows conda
   packages ship ROS demos without the pychrono.ros module (upstream packaging note); recommend
   keeping ROS tasks deferred to the Linux/NVIDIA machine (RoboStack is the Windows fallback).
4. Pilot archive: `runs/pilot-2026-07-04/` zipped to `Box\ChronoBench\runs\pilot-2026-07-04.zip`
   (the standing home for benchmark-run archives).

## First GPU tasks (2026-07-06): `fsi_object_drop` + `crm_tire_rig` through the env registry

The suite now spans TWO execution environments in one gate: the pinned `pychrono10` conda env
(12 tasks, unchanged) and the HIP source build (docs/BUILD_HIP.md) selected per task by
`run.env_id = "hip"` in the contract, resolved through the machine-local
`scoring/envs.local.json` (git-ignored; committed template `scoring/envs.template.json`).

1. **Harness: per-task env registry in `judge_v2`.** Contracts stay portable (only the id is
   committed); triage codes `env:registry-missing(id)` / `env:unknown-id(id)`; tasks without
   `env_id` run exactly as before (pendulum regression re-verified at 100).
2. **`fsi_object_drop` (FSI-SPH axis).** Sphere into a water tank (ChFsiProblemCartesian,
   spacing 0.025); Archimedes spherical-cap oracle; turns density 500 (center at surface) /
   900 (0.073 m draft) / 2500 (sinks, rests on floor). Calibration exposed and the contract
   documents the SPH BCE-skin float bias (~+0.04 m, ~+35% effective buoyant volume; a 1.2
   density ratio still floats, hence 2500 for the sink turn) and the
   `CreateCollisionShapes` requirement (without it the sinking sphere falls THROUGH the floor
   to z = -99). Truths 100/100/100; the settled sink case re-measures -0.3800 exactly.
3. **`crm_tire_rig` (CRM terramechanics axis).** Polaris tire on CRM soil in a ChTireTestRig;
   turns load 2500 N / 5000 N (4.3 cm deeper) / 30 RPM at 5000 N (slip-sinkage, deeper still).
   The slip observable is KINEMATIC (both rig speeds imposed; omega*R/v - 1 = 0.7291 / 4.1873,
   matched to 4 decimals), so it pins the wheel-speed change exactly; z and drawbar bands
   calibrated-and-frozen. Reproducibility split: z repeats to <1 mm, drawbar swings tens of
   percent (bands sized accordingly). Truths 100/100/100.
4. Runtimes on gfx1151: FSI ~70 s (3 s turns) / ~152 s (5 s sink turn); CRM ~113-134 s per
   turn. New DELTAS entries: BCE-vs-contact geometry, BCE-skin buoyancy bias, rig
   self-sequencing (measurements enable at t = 3; `GetPos` vs `GetSpindle`), slip definition,
   `ReportTireForce` reads 0 in this rig mode.

## Shape pilots, solo half (2026-07-08): three task SHAPES land in one batch (+7 tasks, suite 21 / 63 turns)

The suite now covers all three conversion/translation shapes that were previously only designed
(the interactive modernize-broken pair with Dan still pending). Every prompt embeds the COMPLETE
source artifact (C++ file, MJCF XML, or PyBullet script), since generation-time translation is
the skill under test; verbatim sources kept per task in `source/` for provenance.

1. **C++ -> PyChrono translation** (`cpp_ujoint`, `cpp_fourbar`, `cpp_car_suspension`): Cardan
   closed form (signed bands: the motor body-order finding); Freudenstein loop-closure oracle
   (4-5 decimal agreement; the demo starts AT the extended collinear extreme); and the
   PRACTICAL 718-line MySimpleCar (9 bodies, distance-rod wishbones, TSDA springs, a
   throttle/differential/DC-motor drivetrain law to translate; force-balance anchor closes to
   2-3% at two stiffnesses). Authoring findings: a rod-anchor mirroring bug in MY OWN draft
   reference (caught and fixed against the C++ coordinates); a designed side-drift invariant
   REMOVED after it proved to grade OpenMP contact-ordering noise (free-rolling creep 0.1-1.2 m
   across identical runs) rather than physics; the 60 m demo ground plate is a run-length trap
   for driven scenarios (the car drives off the edge at ~73 m and falls; turn 3 enlarges it,
   stated in the prompt).
2. **MJCF -> PyChrono conversion** (`mjcf_cartpole`, `mjcf_double_pendulum`,
   `mjcf_robot_arm`): no MJCF importer exists in PyChrono (verified), so semantics are
   hand-translated. Closed-form coupled cart-pole oscillation (recoil mass-ratio; COM-x
   conserved to machine zero), RK4 double pendulum (4-5 decimal agreement; nested-body pos is
   the graded trap), and the PRACTICAL 3-DOF position-actuated arm (FK oracle; home return
   exact to 1e-13; bad control = dropped hinge axes).
3. **PyBullet -> PyChrono conversion** (`pyb_arm_motor`): the IMPERATIVE-source counterpart,
   TRIPLE-validated (RK4 oracle / PyBullet source run in a disposable `pyb-src` conda env /
   Chrono reference agree to 4 decimals). Findings: velocity-motor step starts are impulsive
   (3.6x amplitude; soft-start ramp is part of the spec); PyBullet `linkPositions` are
   parent-JOINT-frame relative (the author's own slip became the bad control); PyBullet
   `jointDamping` is not a plain viscous torque (source applies -b*qdot explicitly).
4. All seven: truths 100/100/100, good ~100, bad capped 40. Suite self-check 63/63 after two
   variance-driven hardening passes that full-suite sweeps exposed: cpp_car_suspension turn 3
   flaked once under concurrent system load (threaded-contact nondeterminism is LOAD-conditioned;
   quiet runs are bit-identical: ride band widened to contention variance, monotonic window moved
   past the bounce), and crm_tire_rig turn 2's drawbar check was REMOVED after the GPU drawbar
   proved to spread 2.6x across identical runs (105.9-277.5 N; sinkage, <1 mm repeatable,
   carries the load law). Sweep-level flakes are themselves calibration data.

## Isaac Sim / USD batch (2026-07-08): the robotics heavy-hitter triad is complete (+2 tasks, suite 23 / 69 turns)

The conversion shape now spans Isaac Sim, MuJoCo, and PyBullet. Both tasks source compact
hand-authored USDA stages in Isaac's exact idiom (UsdPhysics joints with per-body localPos
frames, MassAPI inertials, PhysX DriveAPI; world-anchored joints; explicit stage metadata),
structurally validated with usd-core (PyPI, in the disposable `usd-src` env; all five stages
LINT PASS with intended metadata). Isaac itself cannot run on this AMD machine: the Isaac-side
replay of the stages is the explicitly recorded PENDING provenance leg for the NVIDIA machine.

1. **isaac_crank_pendulum, the MATCHED TRIPLE**: the pyb_arm_motor physics, constant for
   constant, expressed declaratively in USD: the same system now exists as an imperative
   PyBullet program, a declarative Isaac/USD stage, and the Chrono references, so score
   differences between the two conversion tasks isolate pure REPRESENTATION effects (the
   controlled instrument for the pre-registered C15 prediction). USD semantics graded: angular
   drives in the schema's DEGREE units (85.9437 = 1.5 rad/s; reading degrees as radians builds
   a 57x crank); stiffness-0 + damping-only DriveAPI = a NATIVE declarative viscous damper
   (-> ChLinkRSDA); controller timelines live outside the stage. Bad control = damper converted
   onto the wrong joint (undamped tail 0.3551, oracle-predicted, vs required 0.2015).
2. **isaac_robot_arm, the unit-semantics practical**: a 3-DOF arm authored in CENTIMETERS
   (metersPerUnit 0.01; gravityMagnitude reads 981) with position drives and a decreed
   reach-out pick arc; FK oracle exact (home 1.3/1.5 to 1e-12). Turn 3 re-exports the SAME arm
   at meters, Y-up: the graded truth is INVARIANCE (turn-2 values reproduced to 12 digits by
   the Y-up truth). Authoring finding: the first trajectory draft folded the arm onto itself
   and collapsed the yaw-sweep observable to ~1 cm; redesigned so the yaw peak rides full
   extension. Bad control = metersPerUnit ignored (a happily simulating 130 m robot).
3. Both tasks: truths 100/100/100, good ~100, bad capped 40; suite self-check 69/69.

## Newton batch (2026-07-08): the matched set becomes a QUADRUPLE (+1 task, suite 24 / 72 turns)

`newton_crank_pendulum` sources a complete program in Newton (the NVIDIA + Google DeepMind +
Disney Research engine, Linux Foundation, v1.3.0 on warp 1.15.0, GTC 2026's centerpiece): the
same crank-and-pendulum system as pyb_arm_motor and isaac_crank_pendulum, now from a PRIOR-POOR
ecosystem (public for about a year, barely present in training corpora). Design intent:
PyBullet-vs-Newton isolates ecosystem PRIOR STRENGTH at fixed representation style (the newly
pre-registered claim C16); PyBullet-vs-USD isolates representation (C15); constants, oracle,
and bands identical across all four legs.

1. **Validation is the strongest in the suite**: the Newton sources EXECUTE on this machine
   (Warp CPU device, no GPU needed) and match the shared RK4 oracle to sub-percent on every
   graded value (tails 0.1999/0.3250/0.0431 vs 0.2015/0.3267/0.0434); the residual is the
   finite drive gain vs the oracle's ideally prescribed pivot. No pending replay leg.
2. **Authoring findings (all in DELTAS)**: eval_ik joint coordinates wrap at +-2 pi (a
   spurious 4-pi jump masqueraded as a blow-up until the body-quaternion observable replaced
   the chain-coordinate sum); explicit velocity drives have a gain-times-step stability limit
   on Featherstone (kd 1000: NaN at dt 1e-4, clean at 2e-5); Newton's native joint `damping`
   IS plain viscous (unlike PyBullet's); add_link separates body frame from COM (the
   imperative twin of the declarative inertial-offset trap: the bad control drops it and the
   pendulum winds to the 2-pi wrap boundary, gate-measured 6.283).
3. Truths 100/100/100, good 100, bad 40; suite self-check 72/72; disposable newton-src env
   (conda + pip newton, recipe in CONTRACT.md) removed after validation.

## Tasks

| Task | Axis | Sim | State | Self-score | Notes |
|------|------|-----|-------|-----------|-------|
| pendulum | mechanism | pychrono | verified+3turn | 100 | PILOT COMPLETE; 3 turns; input1-3.txt; CSV-derived invariants; end-to-end generate->judge proven |
| plate_sinkage_scm (new) | terramechanics | pychrono | verified+3turn | 100 | FIRST REALISTIC; SCM deformable soil (CPU); Bekker-Wong coarse-band oracle; turns baseline/softer/heavier; param-first; good 100 / bad 40 |
| slider_crank | mechanism | pychrono | verified+3turn | 100 | DONE FULLY; 3 turns (stroke/stroke/speed); analytic-kinematics oracle; stroke+peak_speed derived; good 100 / bad 40 |
| gear | mechanism | pychrono | verified+3turn | 100 | DONE FULLY; 3 turns (2:1/3:1/two-stage); closed-form kinematics oracle; SIGN of w3 discriminates; good 100 / bad 40; pulley + phase-capture findings |
| mass_spring_damper | mechanism | pychrono | verified+3turn | 100 | DONE FULLY; 3 turns; oracle-grounded (oracle.py); derived period_d/zeta/ss_amp; good 100 / bad 40 |
| beam | FEA | pychrono | verified+3turn | 100 | DONE FULLY; 3 turns (tip load/self-weight/superposition); numpy-FE oracle; tip DERIVED from logged shape; good 100 / bad 40 |
| cable | FEA | pychrono | pending | | KEEP-port; ANCF cable |
| buckling | FEA | pychrono | pending | | KEEP-port |
| rotor | FEA | pychrono | pending | | KEEP-port; IGA rotor |
| tablecloth | FEA | pychrono | pending | | KEEP-port; BST shells |
| fea_ancf_beam (new) | FEA | pychrono | verified+3turn | 100 | DONE FULLY; 3 turns (tip/self-weight/superposition); ANCF cable + nonlinear static; numpy-EB oracle agrees <=0.03%; good 100 / bad 40 |
| solver_nsc_smc (new) | solver/contact | pychrono | verified+3turn | 100 | DONE FULLY; 3 turns (NSC e=0.7/e=0.9/SMC switch); impact-kinematics oracle; dt-pocket + soft-SMC findings; good 100 / bad 40 |
| coupling_rigid_flex (new) | coupling | pychrono | pending | | ADD; rigid body on flexible beam |
| swig_contact_reporter (new) | swig-extension | pychrono | verified+3turn | 100 | DONE FULLY; 3 turns (N=4/N=6/heavier+per-contact); equilibrium oracle; count+sum+per-contact derived; good 100 / bad 40 |
| import_urdf (new) | data-import | pychrono | verified+3turn | 100 | DONE FULLY; ships assets/pendulum.urdf (judge stages assets/); sine actuation + free-swing inertial test; good 100 / bad 40 |
| yaml_mbs (new) | data-import | pychrono | verified+3turn | 100 | DONE FULLY; script authors MBS-YAML + ChParserMbsYAML; stroke/peak-speed oracle; world-frame constraint-point gotcha; good 100 / bad 40 |
| checkpoint (new) | state-mgmt | pychrono | pending | | ADD; checkpoint/restart determinism |
| hmmwv_scm | vehicle-terramech | pychrono | verified+3turn | 100 | DONE FULLY (vehicle pick SETTLED); rigid/SCM-firm/SCM-soft; Bekker-anchored rut bands + calibrated speed bands; good 100 / bad 40 |
| fsi_object_drop (new) | FSI-SPH | pychrono (hip env) | verified+3turn | 100 | DONE FULLY; FIRST GPU TASK (env registry); Archimedes oracle + documented BCE-skin bias; float/deeper-draft/sinks-to-floor; good 100 / bad 40 |
| crm_tire_rig (new) | tire-terramech-CRM | pychrono (hip env) | verified+3turn | 100 | DONE FULLY; CRM counterpart to the SCM tasks; kinematic-slip anchor + load/slip-sinkage laws; good 100 / bad 40 |
| cpp_ujoint (new) | translation (C++) | pychrono | verified+3turn | 100 | DONE FULLY; SHAPE PILOT; embeds demo_MBS_ujoint.cpp; signed Cardan bands (motor body-order finding); good 100 / bad 40 |
| cpp_fourbar (new) | translation (C++) | pychrono | verified+3turn | 100 | DONE FULLY; SHAPE PILOT; embeds demo_MBS_fourbar.cpp (GUI class to strip); loop-closure oracle 4-5 decimals; good 100 / bad 40 |
| cpp_car_suspension (new) | translation (C++, practical) | pychrono | verified+3turn | 100 | DONE FULLY; SHAPE PILOT; 718-line MySimpleCar incl. drivetrain law; force-balance anchor 2-3%; drift-check-removed finding; good 100 / bad 40 |
| mjcf_cartpole (new) | conversion (MJCF) | pychrono | verified+3turn | 100 | DONE FULLY; SHAPE PILOT; recoil mass-ratio closed form + exact COM conservation; COM-offset trap; good 100 / bad 40 |
| mjcf_double_pendulum (new) | conversion (MJCF) | pychrono | verified+3turn | 100 | DONE FULLY; SHAPE PILOT; RK4 oracle 4-5 decimals; nested-body-pos trap; energy instrumentation; good 100 / bad 40 |
| mjcf_robot_arm (new) | conversion (MJCF, practical) | pychrono | verified+3turn | 100 | DONE FULLY; SHAPE PILOT; 3-DOF position-actuated arm; FK oracle, home return 1e-13; hinge-axis trap; good 100 / bad 40 |
| pyb_arm_motor (new) | conversion (PyBullet) | pychrono | verified+3turn | 100 | DONE FULLY; SHAPE PILOT (3rd ecosystem, imperative source); TRIPLE-validated to 4 decimals; impulsive-start + linkPositions findings; good 100 / bad 40 |
| isaac_crank_pendulum (new) | conversion (Isaac/USD) | pychrono | verified+3turn | 100 | DONE FULLY; MATCHED TRIPLE with pyb_arm_motor (same physics, declarative USD); degree-units + native-damper-drive semantics; usd-core linted, Isaac replay PENDING (NVIDIA); good 100 / bad 40 |
| isaac_robot_arm (new) | conversion (Isaac/USD, practical) | pychrono | verified+3turn | 100 | DONE FULLY; metersPerUnit (cm!) + upAxis re-export invariance turns; FK exact, turn-3 invariance to 12 digits; usd-core linted, Isaac replay PENDING (NVIDIA); good 100 / bad 40 |
| newton_crank_pendulum (new) | conversion (Newton, prior-poor) | pychrono | verified+3turn | 100 | DONE FULLY; 4th leg of the matched set (C16 prior-strength instrument); sources EXECUTED on Warp CPU, sub-percent vs oracle, no pending leg; eval_ik wrap + drive-stability findings; good 100 / bad 40 |
| m113 | vehicle(tracked) | pychrono | pending | | KEEP-port |
| (1 car: sedan or citybus) | vehicle | pychrono | pending | | KEEP-port; Dan picks which |
| gps_imu | sensor | pychrono | pending | | KEEP-port; GPS/IMU (no OptiX) |
| turtlebot | robotics | pychrono | pending | | KEEP-port; rover (keep only for URDF/control) |

Ground-locomotion cap (<=3-4 total): hmmwv + m113 + one car + at most one rover. `scm`/`rigid_multipatches`/
`curiosity` are candidates to drop or re-tag under the tightened cap; decide with the task-list approval.

## DEFERRED (infra-gated) -- NOT authored, need infra (see `.agent-related/SUITE_DESIGN.md` + `.agent-related/PANEL_REDTEAM.md`)

OptiX sensors (camera/lidar/radar), ROS (vehros/sensros/multi-agent), PyDEME/DEM
(pile/cone-penetration/mixer). Designed; await GPU/ROS/PyDEME env. (FSI/SPH and CRM came OFF
this list 2026-07-06: covered on the AMD HIP build via the env registry.)

## Gates awaiting Dan (see `.agent-related/HANDOFF.md`)

1. Approve/adjust the finalized task list (which car; final ground-locomotion set; CAD include/defer).
2. Expert sign-off on a per-axis sample of references.
3. Recalibration of the de-scoped judge vs human judgment.
