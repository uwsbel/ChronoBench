# cpp_ujoint -- task contract (v2.0, PyChrono 10.0)

First task of the SOURCE-TRANSLATION shape (C++ -> PyChrono): the prompt embeds the complete
`demo_MBS_ujoint.cpp` (from projectchrono/chrono `src/demos/mbs`; verbatim copy kept in
`source/` for provenance) and the candidate must emit an equivalent headless PyChrono script.
The mechanism: two shafts joined by a `ChLinkUniversal` (Cardan/Hooke joint) with a bend angle,
input shaft driven by a ramp-angle rotation motor, gravity off, pure deterministic kinematics.
Graded from the candidate's own logged local-z angular velocities of both shafts.

## Oracle posture (closed form, tight)

`oracle.py` (stdlib math, NO Chrono): omega2(t) = omega1 cos(b) / (1 - sin^2(b) cos^2(omega1 t)),
oscillating between omega1 cos(b) and omega1/cos(b) at twice the shaft frequency, cycle-mean
omega1, oscillation period pi/omega1. SIGN convention verified on the pinned build: the demo
initializes the motor as Initialize(ground, shaft), which spins both shafts in NEGATIVE local z,
so all graded values are negative and the bands are SIGNED (direction is part of a faithful
translation; the gear task set the precedent). Two-way validation: the PyChrono references
reproduce the closed-form extremes to 4 decimals on all three turns
(turn 1: -1.154709/-0.866021 vs -1.1547/-0.8660; turn 2: -1.414246/-0.707099; turn 3:
-2.828685/-1.414149).

## Turns

1. Translate: beta 30 deg, omega1 = 1. omega2 trough in [-1.21, -1.10] (ref -1.1547), crest in
   [-0.905, -0.83] (ref -0.8660), drive mean in [-1.03, -0.97].
2. Modify: beta -> 45 deg (everything the C++ derives from `angle` must follow). Trough
   [-1.48, -1.35], crest [-0.74, -0.675]; an unmodified 30-deg candidate fails BOTH.
3. Extend: drive rate -> 2 rad/s at 45 deg. Trough [-2.96, -2.70], crest [-1.48, -1.35], drive
   mean [-2.06, -1.94], plus the oscillation period pi/omega1 = 1.5708 s in [1.45, 1.70]
   (period derive about the drive level -2.0).

## Shape notes (worth knowing)

1. The translation-fidelity trap this task actually caught during authoring: the motor's BODY
   ORDER (`Initialize(ground, shaft)`) makes the ramp drive ground-relative-to-shaft, so the
   shafts spin in negative local z. The C++ demo's own console output has the same sign; a
   translator who "fixes" the sign has changed the experiment. The prompt states the expected
   sign, and the bands enforce it.
2. The C++ comment itself offers the ChLinkMotorRotationSpeed alternative; the L2 motor cap and
   the good control accept it (constant speed 1 = ramp slope 1).
3. Runtime: ~2 s per turn (2000 steps, kinematic). Timeout 120.

## Gate (de-scoped judge)

1. L1: runs headless on pinned pychrono10 (timeout 120 s); emits `out.csv` (`t,w1,w2`) + one
   JSON line.
2. L2 (minimal): `ChLinkUniversal`, `ChLinkMotorRotationAngle|ChLinkMotorRotationSpeed`,
   `ChLinkLockCylindrical|ChLinkLockRevolute`.
3. L3 (measured): the signed trough/crest/drive bands above (+ the period band on turn 3), all
   derived from the logged CSV.

## Verify

```
conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/cpp_ujoint --turn N            # 100
conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/cpp_ujoint --turn 1 demo_data_10/cpp_ujoint/samples/good_candidate.py   # 100
conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/cpp_ujoint --turn 1 demo_data_10/cpp_ujoint/samples/bad_candidate.py    # 40 (invariant-fail, bend angle mis-transcribed pi/60)
```
