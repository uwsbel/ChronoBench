# newton_crank_pendulum -- task contract (v2.0, PyChrono 10.0)

The PRIOR-POOR ecosystem leg of the conversion shape, and the FOURTH leg of the matched set:
the prompt embeds a complete Newton program (the NVIDIA + Google DeepMind + Disney Research
engine, Linux Foundation, built on Warp; public for about a year, pinned here at newton 1.3.0 /
warp 1.15.0) building the same crank-and-pendulum system that already exists as an imperative
PyBullet program, a declarative Isaac/USD stage, and the Chrono references. The design intent:

1. PyBullet vs Newton: SAME representation style (imperative Python), maximally different
   ECOSYSTEM PRIOR (PyBullet saturates training corpora; Newton barely exists in them): this
   cell isolates prior strength (the pre-registered claim C16).
2. PyBullet vs USD: same physics, different representation style (claim C15).
3. Constants, oracle, and graded bands are identical across all legs.

## Oracle posture (executed source, sub-percent agreement)

`oracle.py` (stdlib math): the shared RK4 pendulum-with-prescribed-moving-pivot integrator and
trapezoidal drive profile. The Newton sources were EXECUTED on this machine (Warp CPU device,
no GPU required) and match the oracle to better than 1% on every graded value: v1 tail 0.1999 /
max 0.3051 (oracle 0.2015 / 0.3057), v2 tail 0.3250 (0.3267), v3 tail 0.0431 (0.0434) with the
braked crank at 0.001 rad/s and ring period 1.149 (band anchor 1.1785). The sub-percent
residual is physical: the finite velocity-drive gain (target_kd = 1000) versus the oracle's
ideally prescribed pivot.

## Newton/Warp semantics the task grades (found empirically during authoring)

1. `add_link` separates the BODY FRAME (xform, typically at the joint) from the center of mass
   (`com`, an in-frame offset): the imperative twin of the MJCF/USD inertial-offset trap, and
   the bad control drops it.
2. A joint actuator in `JointTargetMode.VELOCITY` uses `target_kd` as its tracking GAIN, and
   the target lives on `Control.joint_target_qd`, retargeted per step by the control loop.
3. The joint's native `damping` parameter IS a plain viscous torque (validated against the
   oracle; contrast PyBullet's jointDamping, which is not).
4. Newton's joint coordinates from `eval_ik` WRAP at +-2 pi: the chain-coordinate sum q0 + q1
   is not a safe absolute angle once the crank has whirled past a revolution (the authoring
   probe measured a spurious 4-pi jump); the pendulum's body quaternion is the wrap-free
   observable, and both the source and the prompt use it.
5. Explicit velocity drives have a gain-times-step stability limit on the Featherstone solver:
   target_kd = 1000 diverged (NaN) at a 1e-4 substep and is clean at 2e-5. The source pins
   SUBSTEPS accordingly.

## Turns

1. Convert: drive to 1.5 rad/s. Tail swing in [0.17, 0.23]; run max in [0.27, 0.34]; crank
   cruise in [1.44, 1.56].
2. Modify: OMEGA retargeted to 2.0. Tail in [0.29, 0.37]; max in [0.43, 0.53]; cruise in
   [1.92, 2.08].
3. Extend: the control loop ramps the target to zero at t = 5 s. Tail <= 0.08; tail crank rate
   <= 0.05; ring-down period in [1.12, 1.24]; pre-brake overshoot still in [0.43, 0.53].

## Controls

Good = stylistic variant (~100). Bad = the com-offset drop: the pendulum body placed at the
elbow, mass at its own hinge, no gravity lever arm; the damper then drags it around WITH the
crank and its absolute angle winds to the quaternion-angle wrap boundary (gate measured
theta_max = 6.283 = 2 pi, tail 2.63, vs the required 0.31 / 0.20: runs clean, fails decisively).

## Gate (de-scoped judge)

1. L1: runs headless on pinned pychrono10 (timeout 120 s); emits `out.csv` (`t,w,theta`) + one
   JSON line.
2. L2 (minimal): a rotation speed/angle motor, a free revolute, a `ChLinkRSDA` damper.
3. L3 (measured): the tail/max/cruise bands (+ ring-down and brake-hold on turn 3), derived
   from the logged CSV.

## Env provenance

Disposable validation env: `conda create -n newton-src -c conda-forge python=3.11` +
`pip install "newton[examples]"` (PyPI-only; Dan-approved exception), removed after use.
Newton executes on the Warp CPU device on plain Windows hardware: no NVIDIA GPU was needed for
validation, so this task carries NO pending replay leg.

## Verify

```
conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/newton_crank_pendulum --turn N            # 100
conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/newton_crank_pendulum --turn 1 demo_data_10/newton_crank_pendulum/samples/good_candidate.py   # 100
conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/newton_crank_pendulum --turn 1 demo_data_10/newton_crank_pendulum/samples/bad_candidate.py    # 40 (invariant-fail, com offset dropped)
```
