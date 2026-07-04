# import_urdf -- task contract (v2.0, PyChrono 10.0)

Data-import axis: loading a robot description from URDF (`chrono::parsers::ChParserURDF`),
populating a system, fixing the root, actuating a NAMED joint with a position motor, and (turn 3)
trusting the imported inertial parameters, graded from the candidate's own logged joint angle.

The task ships its own asset, `assets/pendulum.urdf` (authored parameter-first: the URDF text IS
the declared parameter source). The judge stages `assets/` into the candidate's run directory, so
scripts load `pendulum.urdf` from the current working directory; the prompt also embeds the URDF
text so the model knows the joint and link names.

## Independent oracle (anti-circularity)

`oracle.py` (stdlib math, NO Chrono):
1. Turns 1-2: the joint is POSITION-actuated with theta = A sin(2 pi f t), so amplitude = A and
   period = 1/f are exact closed forms of the commanded actuation, independent of dynamics.
2. Turn 3: free swing; the compound-pendulum period from the URDF-declared inertials
   (m = 2 kg, d = 0.5 m, Izz_com = m L^2/12) is T = 2 pi sqrt(I_pivot/(m g d)) = 1.63795 s
   (1.64204 s with the 0.2 rad finite-amplitude correction).
Two-way validation: references measure amp 0.50001 / 0.80000 / 0.19894 and period 1.000 / 2.000 /
1.638 (turn-3 period within 0.25% of the oracle).

## Turns

1. Create: load + populate + fix root + position-actuate "swing" with A = 0.5 rad, f = 1.0 Hz.
2. Modify: A = 0.8 rad, f = 0.5 Hz (longer run; catches unmodified actuation at 37%/50% error).
3. Extend: actuation removed (passive revolute), released from rest at 0.2 rad; the measured
   period tests that the parser truly imported the URDF's inertia (catches dropped/mangled
   inertials), and the ~0.2 amplitude catches "actuation left on".

## Gate (de-scoped judge)

1. L1: runs headless on pinned pychrono10 with `pendulum.urdf` staged into the run dir; emits
   `out.csv` (`t,theta`) + one JSON line.
2. L2 (minimal): `ChParserURDF`, `PopulateSystem`, plus actuation caps on turns 1-2
   (`ActuationType_POSITION|SetAllJointsActuationType`, `SetMotorFunction`, `ChFunctionSine`).
3. L3 (measured): amplitude (max|theta|) and period (zero crossings) vs the oracle, rel_tol
   0.05-0.08.

## Authoring notes

1. Judge upgrade shipped with this task: `judge_v2` now stages a task's `assets/` folder into the
   temp run directory (generic; any task can ship data files).
2. `ChFunctionSine(amplitude, frequency_Hz)` confirmed by evaluation; the shipped
   `demo_PARSER_URDF.py` passes three arguments (a 9.0-style signature), do NOT copy it blindly.
   The bad control swaps the two arguments and is caught by both invariants (amp 1.0, period 2.0).
3. `parser.GetChBody("arm")` / `parser.GetChMotor(...)` work for named lookup; quaternion
   components are attributes (`q.e0` ... `q.e3`); for the planar joint here
   theta = 2 atan2(e3, e0).

## Verify

```
conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/import_urdf --turn N            # 100
conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/import_urdf --turn 1 demo_data_10/import_urdf/samples/good_candidate.py   # 100
conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/import_urdf --turn 1 demo_data_10/import_urdf/samples/bad_candidate.py    # 40 (invariant-fail)
```
