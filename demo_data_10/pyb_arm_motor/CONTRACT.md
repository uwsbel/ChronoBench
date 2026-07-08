# pyb_arm_motor -- task contract (v2.0, PyChrono 10.0)

The THIRD-ECOSYSTEM conversion task (PyBullet -> PyChrono): where the MJCF tasks convert a
DECLARATIVE model file, this one converts an IMPERATIVE program: a complete PyBullet script
(`source/pyb_arm_v1..3.py`) that builds a two-link arm with `createMultiBody` link arrays, sets
rod inertias through `changeDynamics`, frees a joint the PyBullet way (velocity motor commanded
with force = 0), applies an explicit viscous elbow torque every step, and velocity-controls the
crank with per-step retargeting. The candidate must translate API semantics, not just physics.

## Oracle posture (triple validation)

`oracle.py` (stdlib math, NO Chrono, NO PyBullet): the pendulum on a whirling crank is a
compound pendulum with a PRESCRIBED MOVING PIVOT; exact RK4 at dt = 2e-5 with the same
trapezoidal drive profile. THREE independent implementations agree on every graded value to 4
decimals: RK4 0.3057/0.2015 (turn-1 max/tail), PyBullet source 0.30569/0.2015 (run in the
disposable `pyb-src` conda env: `conda create -n pyb-src -c conda-forge python=3.11 pybullet`),
PyChrono reference 0.30567/0.2015. Turn 3's ring-down period lands on the closed-form damped
value 1.1582/sqrt(1 - zeta^2) = 1.1785 s.

## Turns

1. Convert: soft-started drive to 1.5 rad/s. Steady tail swing in [0.17, 0.23] (ref 0.2015);
   run max in [0.27, 0.34] (ref 0.3057); crank cruise in [1.44, 1.56].
2. Modify: the source retargets to 2.0 rad/s. Tail in [0.29, 0.37] (ref 0.3267; unmodified
   0.2015 fails low); max in [0.43, 0.53]; cruise in [1.92, 2.08].
3. Extend: the source ramps the motor back to ZERO at t = 5 s. Tail swing <= 0.08 (ref 0.0434;
   un-braked 0.3267 fails 4x over), tail crank rate <= 0.05, ring-down period in [1.12, 1.24]
   (closed form 1.1785; a still-driven candidate measures the drive-locked 3.26 s), pre-brake
   overshoot still in [0.43, 0.53].

## Calibration findings (probe-derived, worth knowing)

1. A velocity-motor STEP start is an impulsive constraint: it kicked the free pendulum to 3.6x
   the smooth-oracle amplitude. The 0.5 s soft-start ramp is therefore part of the task spec in
   all three implementations (and the honest reason it exists is stated in the prompt).
2. PyBullet's `linkPositions` are relative to the parent's JOINT frame, not its COM frame: the
   author's own first source placed the elbow at the crank's COM (half the whirl radius, 25%
   low amplitude) before the three-way validation caught it. That authentic slip IS the bad
   control, with its measured failure values.
3. PyBullet's built-in `jointDamping` does NOT behave like a plain viscous torque (25% lower
   response than -b*qdot at the same coefficient); the source applies the damper explicitly via
   TORQUE_CONTROL instead, which matches Chrono's ChLinkRSDA and the oracle exactly.
4. Undamped, the startup transient rings forever and pollutes every tail observable; the elbow
   damper (zeta = 0.18) is what makes steady-state bands meaningful.
5. Runtime: ~4 s per turn. Timeout 120.

## Gate (de-scoped judge)

1. L1: runs headless on pinned pychrono10 (timeout 120 s); emits `out.csv` (`t,w,theta`) + one
   JSON line.
2. L2 (minimal): a rotation speed/angle motor, a free revolute, a `ChLinkRSDA` damper.
3. L3 (measured): the tail/max/cruise bands (+ ring-down period and brake-hold on turn 3), all
   derived from the logged CSV.

## Verify

```
conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/pyb_arm_motor --turn N            # 100
conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/pyb_arm_motor --turn 1 demo_data_10/pyb_arm_motor/samples/good_candidate.py   # 100
conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/pyb_arm_motor --turn 1 demo_data_10/pyb_arm_motor/samples/bad_candidate.py    # 40 (invariant-fail, linkPositions misread as COM-relative)
```
