# cpp_fourbar -- task contract (v2.0, PyChrono 10.0)

Second task of the SOURCE-TRANSLATION shape (C++ -> PyChrono): the prompt embeds the complete
`demo_MBS_fourbar.cpp` (from projectchrono/chrono `src/demos/mbs`; verbatim copy in `source/`),
which is a harder translation than cpp_ujoint because roughly a third of the file is an Irrlicht
GUI (a scrollbar-driven `MyEventReceiver` class, drawing helpers, a trajectory buffer) that the
candidate must RECOGNIZE as non-physics and drop. The mechanism: a Grashof crank-rocker
(crank pivot at the origin, pin (2,0); coupler to (8,0); rocker to the ground pivot (8,-8)),
crank driven at a constant CH_PI rad/s, EULER_IMPLICIT_LINEARIZED at 1e-3 s.

## Oracle posture (independent loop-closure solver)

`oracle.py` (stdlib math, NO Chrono): circle-intersection loop closure with branch continuity
over a full crank revolution, cross-checked against the closed-form collinear-extreme
configurations. A structural nicety: the demo STARTS at the extended (crank + coupler collinear)
configuration, so the rocker begins exactly at one swing extreme (alpha_min ~ 0 by
construction, a free invariant). Two-way validation on the pinned build: swing amplitude
0.549360 vs oracle 0.5494 (turn 1), 0.253382 vs 0.2534 (turn 2), coupler-midpoint height range
1.00769 vs 1.0077 (turn 3): 4-5 decimals across the board. Sign convention: the demo's
Initialize(truss, crank) motor order spins the crank in NEGATIVE z (the cpp_ujoint finding);
the drive band is signed.

## Turns

1. Translate: swing amplitude in [0.50, 0.60] (ref 0.5494); alpha_min in [-0.03, 0.03]; drive
   mean in [-3.30, -2.99].
2. Modify: crank pin (2,0) -> (1,0). The linkage stays Grashof (1 + 11.31 < 7 + 8); swing drops
   to [0.22, 0.29] (ref 0.2534; an unmodified candidate at 0.5494 fails high). Design note:
   LENGTHENING the crank to 3 was rejected because 3 + 11.31 > 5 + 8 makes it a non-Grashof
   double-rocker that fights the motor.
3. Extend: drive doubled to 2*CH_PI (drive band [-6.60, -5.97], rocker period [0.95, 1.05] s)
   plus a NEW logged observable: the coupler midpoint's height, whose range must land in
   [0.90, 1.11] (ref 1.0077).

## Shape notes (worth knowing)

1. The GUI-stripping requirement is graded implicitly: any Irrlicht import fails L1 headless
   (no display), and the physics that remains is checked by the loop-closure bands.
2. Swing amplitude is speed-independent (pure kinematics); the turn-3 discriminators for "rate
   not doubled" are the signed drive band and the period band, not the amplitude.
3. Bodies keep the demo's default mass/inertia (never set): the mechanism is motor-driven
   kinematics, so masses and gravity load the constraints without changing the trajectory.
4. Runtime: ~3 s per turn (10000 steps). Timeout 120.

## Gate (de-scoped judge)

1. L1: runs headless on pinned pychrono10 (timeout 120 s); emits `out.csv` (`t,alpha,wz`, plus
   `cmy` on turn 3) + one JSON line.
2. L2 (minimal): `ChLinkMotorRotationSpeed|ChLinkMotorRotationAngle`, `ChLinkLockRevolute`,
   `EULER_IMPLICIT_LINEARIZED`.
3. L3 (measured): the swing/start/drive bands above (+ period and coupler-midpoint range on
   turn 3), all derived from the logged CSV.

## Verify

```
conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/cpp_fourbar --turn N            # 100
conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/cpp_fourbar --turn 1 demo_data_10/cpp_fourbar/samples/good_candidate.py   # 100
conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/cpp_fourbar --turn 1 demo_data_10/cpp_fourbar/samples/bad_candidate.py    # 40 (invariant-fail, CH_PI mis-copied as CH_PI_2)
```
