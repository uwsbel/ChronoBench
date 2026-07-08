# isaac_crank_pendulum -- task contract (v2.0, PyChrono 10.0)

The Isaac Sim leg of the CONVERSION shape, and the suite's MATCHED TRIPLE: the prompt embeds a
compact hand-authored USDA stage in Isaac's exact idiom (`source/crank_pendulum_v1..2.usda`:
UsdPhysics revolute joints with per-body localPos frames, MassAPI mass/COM/diagonalInertia,
PhysX DriveAPI, a world-anchoring fixed joint, Z-up, metersPerUnit 1), and the physics is
IDENTICAL, constant for constant, to pyb_arm_motor: the same crank-and-pendulum system now
exists as an imperative PyBullet program, a declarative Isaac/USD stage, and the Chrono
references. Score differences between the two conversion tasks therefore isolate pure
REPRESENTATION effects: the controlled instrument for the pre-registered C15
difficulty-ordering prediction. With MJCF and PyBullet already covered, the conversion shape
now spans the three robotics heavy hitters (Isaac Sim, MuJoCo, PyBullet).

## Oracle posture (inherited three-way validation + USD-side honesty)

`oracle.py` (stdlib math): the same RK4 pendulum-with-prescribed-moving-pivot integrator as
pyb_arm_motor, same trapezoidal drive profile, same bands (tail 0.2015 / 0.3267 / 0.0434, run
max 0.3057 / 0.4778, ring-down period 1.1785 = the closed-form damped value). The PHYSICS
inherits the matched task's three-way validation (RK4 / PyBullet source executed in its own
engine / Chrono, all to 4 decimals). The USD ARTIFACT itself is validated in two stages:
structurally here (usd-core in a disposable `usd-src` env: stage parses; 3 rigid bodies,
3 joints, 2 drives; every physics attribute readable; unit metadata as intended), and its
Isaac Sim replay is PENDING: this AMD machine cannot run Isaac, so replaying the stage in
Isaac and matching the oracle is an explicitly recorded to-do for the NVIDIA machine.

## USD/Isaac semantics the task grades (and the stage documents)

1. Angular drives are authored in the USD Physics schema's DEGREE units: the stage says
   targetVelocity 85.9437 (v1) / 114.5916 (v2), meaning 1.5 / 2.0 rad/s. Reading degrees as
   radians builds a 57x-faster crank.
2. A DriveAPI with stiffness 0 and a velocity target is a VELOCITY drive (-> Chrono speed
   motor); one with stiffness 0, target 0, and only damping is a pure viscous rotary damper
   (-> ChLinkRSDA), declared natively in the stage (no controller needed).
3. Joint localPos0/localPos1 are per-body-frame (the elbow's localPos0 = (0,0,-0.4) is the
   crank TIP in the crank's frame): the same frame-attribution class of trap as PyBullet's
   linkPositions, here declarative.
4. Time-varying retargeting is NOT expressible in the stage: the soft-start/brake controller
   timeline is decreed in the prompt, exactly as an Isaac Python controller would act on the
   same stage at run time.

## Turns

1. Convert: drive 1.5 rad/s. Tail swing in [0.17, 0.23]; run max in [0.27, 0.34]; crank cruise
   in [1.44, 1.56].
2. Modify: the stage retargets to 114.5916 deg/s = 2.0 rad/s. Tail in [0.29, 0.37]; max in
   [0.43, 0.53]; cruise in [1.92, 2.08].
3. Extend: controller ramps the target to zero at t = 5 s. Tail <= 0.08; tail crank rate
   <= 0.05; ring-down period in [1.12, 1.24]; pre-brake overshoot still in [0.43, 0.53].

## Controls

Good = stylistic variant (~100). Bad = the damping DriveAPI converted onto the WRONG joint
(crank instead of elbow): the velocity motor overpowers it, the pendulum is undamped, and the
tail swings at the oracle-predicted 0.3551 instead of 0.2015 (runs clean, fails the band).

## Gate (de-scoped judge)

1. L1: runs headless on pinned pychrono10 (timeout 120 s); emits `out.csv` (`t,w,theta`) + one
   JSON line.
2. L2 (minimal): a rotation speed/angle motor, a free revolute, a `ChLinkRSDA` damper.
3. L3 (measured): the tail/max/cruise bands (+ ring-down and brake-hold on turn 3), derived
   from the logged CSV.

## Verify

```
conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/isaac_crank_pendulum --turn N            # 100
conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/isaac_crank_pendulum --turn 1 demo_data_10/isaac_crank_pendulum/samples/good_candidate.py   # 100
conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/isaac_crank_pendulum --turn 1 demo_data_10/isaac_crank_pendulum/samples/bad_candidate.py    # 40 (invariant-fail, damper on the wrong joint)
```
