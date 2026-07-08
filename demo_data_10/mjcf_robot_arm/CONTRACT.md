# mjcf_robot_arm -- task contract (v2.0, PyChrono 10.0)

The PRACTICAL task of the CROSS-SIMULATOR CONVERSION shape (MuJoCo MJCF -> PyChrono): a 3-DOF
serial arm (base yaw + shoulder/elbow pitch), MuJoCo's home turf. The MJCF carries position
actuators whose time-varying reference lives in the CONTROLLER, exactly as in real MuJoCo
setups, so the prompt states the joint-space pick trajectory and the sanctioned mapping: impose
the angles with `ChLinkMotorRotationAngle` driven by custom `ChFunction` subclasses (which also
exercises PyChrono's Python-director machinery). Under imposed motion the end effector follows
closed-form forward kinematics.

## Oracle posture (closed-form FK, tight)

`oracle.py` (stdlib math): z = H + L2 cos(q2) + L3 cos(q2+q3), r = L2 sin(q2) + L3 sin(q2+q3),
x/y by the yaw, swept numerically over the stated trajectories. Two-way validation on the
pinned build: every extremum reproduces to 4 decimals (z_min 1.2601 / 1.4131, |y| 0.0836 /
0.1454, peak |dz/dt| 0.1138 / 0.152 / 0.304) and the home return is exact to 1e-13. The
trajectory is chosen to START and END at zero joint angles, so motors impose angles directly
(no initial-offset bookkeeping) and "returns home" is a free, exact invariant.

## Turns

1. Convert: L2 = L3 = 0.5. Deepest reach z_min in [1.235, 1.285] (FK 1.2601); yaw sweep
   |y| in [0.075, 0.092]; home return z_final in [1.39, 1.41] and r_final <= 0.02.
2. Modify: the MJCF's upper arm grows to 0.7 m (mass/inertia/COM scale; the elbow and forearm
   move up 0.2 m). z_min in [1.385, 1.440] (an unmodified candidate at 1.2601 fails low);
   |y| in [0.131, 0.160]; home now 1.6.
3. Extend: the trajectory RETIMES 2x (two pick cycles in 10 s). Geometry-driven extremes are
   speed-invariant (z_min band unchanged), but the peak vertical EE rate must double:
   dz_rate_max in [0.27, 0.34] (FK 0.3039; an un-retimed candidate at 0.152 fails low).

## Shape notes (worth knowing)

1. The bad control drops the MJCF hinge `axis` attributes: shoulder/elbow motors default to
   rotating about global z, spinning vertical links uselessly; the EE never dips and z_min
   fails. Axis mapping (frame z is a Chrono motor's rotation axis) is the conversion skill
   probed.
2. Motor body ORDER matters (the cpp_ujoint finding): the prompt says to pass the moving link
   first so the imposed angle rotates the link, not its parent.
3. The graded observables (z, |y|, r) are chosen even/sign-robust so a globally mirrored but
   self-consistent axis convention still passes; only genuinely wrong kinematics fail.
4. Runtime: ~4 s per turn. Timeout 120.

## Gate (de-scoped judge)

1. L1: runs headless on pinned pychrono10 (timeout 120 s); emits `out.csv` (`t,x,y,z,r`) + one
   JSON line.
2. L2 (minimal): `ChLinkMotorRotationAngle`, explicit gravity, a `ChFunction` (custom or
   built-in) driving the motors.
3. L3 (measured): the FK extremum/home bands (+ the EE-rate band on turn 3), all derived from
   the logged CSV.

## Verify

```
conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/mjcf_robot_arm --turn N            # 100
conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/mjcf_robot_arm --turn 1 demo_data_10/mjcf_robot_arm/samples/good_candidate.py   # 100
conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/mjcf_robot_arm --turn 1 demo_data_10/mjcf_robot_arm/samples/bad_candidate.py    # 40 (invariant-fail, hinge axes dropped)
```
