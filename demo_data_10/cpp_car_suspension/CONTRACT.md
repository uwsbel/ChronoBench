# cpp_car_suspension -- task contract (v2.0, PyChrono 10.0)

The PRACTICAL task of the SOURCE-TRANSLATION shape (C++ -> PyChrono): the prompt embeds the
complete 718-line `demo_MBS_suspension.cpp` (projectchrono/chrono `src/demos/mbs`; verbatim copy
in `source/`), real automotive modeling code: a 9-body car whose four corners are double
wishbones built the classic Chrono way (four `ChLinkDistance` rods + a steer/lateral rod per
corner), `ChLinkTSDA` spring-dampers, and rear `ChLinkMotorRotationTorque` drives fed by a
throttle/differential/DC-motor drivetrain function that must itself be translated. Roughly half
the file is Irrlicht GUI; the prompt also explicitly excludes two nondeterministic elements
(six randomly placed obstacles; an asymmetric-friction contact callback), so the run is
deterministic and symmetric.

## Oracle posture (force balance + calibrated bands, stated honestly)

`oracle.py` (stdlib math, NO Chrono): the wishbone rods leave the spindle one near-vertical
motion relative to the chassis, and the angled spring resists it through the geometric motion
ratio 0.4/0.85. Linearized static force balance predicts the SPRING COMPRESSION under the
368 N corner load: 0.0276 m at k = 28300 (measured 0.0276, exact to 4 decimals) and 0.0069 m at k = 113200
(measured 0.0071, 3%): the compression carries the independent anchor at both stiffnesses. The
chassis RIDE HEIGHT couples compression to contact geometry (rigid estimate within ~3 cm), so
ride bands are calibrated-and-frozen (0.4205 / 0.4422 / 0.4369). The drive turn's terminal
physics is closed-form from the translated law (no-load wheel speed 48 rad/s -> 21.6 m/s at
r = 0.45; stall 200 Nm per rear wheel at throttle 0.3); the 8 s point (81.13 m, 17.01 m/s) is
calibrated-and-frozen.

## Turns

1. Translate (throttle 0): drop 0.55 m and settle. Spring settle in [0.810, 0.833] (ref
   0.8224); ride in [0.405, 0.432] (ref 0.4205); spawned-high check (max cy >= 0.9).
2. Modify: springs 4x stiffer (28300 -> 113200, all four; the demo's own GUI knob). Spring
   settle in [0.838, 0.848] (ref 0.8429; unmodified 0.8224 fails low); ride in [0.436, 0.452]
   (rides HIGHER; unmodified 0.4205 fails low).
3. Extend: throttle 0.3 for 8 s, ground enlarged 60 -> 400 m (deliberate, prompt-stated
   deviation: the calibration probe drove off the demo's plate at ~73 m and fell to y = -42).
   Final speed in [14, 20] (ref 17.01, approaching terminal 21.6); distance >= 60 m (ref
   81.13); forward progress monotonic; ride unchanged in [0.430, 0.456].

## Calibration findings (probe-derived, worth knowing)

1. The force-balance anchor closes to 2-3% at BOTH stiffnesses, so the suspension law (4x
   stiffer -> ~1/4 compression -> rides higher) is graded with real margins, not vibes.
2. The 60 m ground plate is a run-length trap for any driven scenario: at throttle 0.3 the car
   exits the plate inside 6 s. Ground enlargement is the one sanctioned deviation, stated in
   input3.txt.
3. A candidate side-drift invariant (max |cz| <= 0.3 at throttle 0) was DESIGNED, TRIED, AND
   REMOVED: it graded noise. A car on free-rolling wheels (revolute joints, zero motor torque,
   no brake) creeps 0.1 to 1.2 m in z during the bounce phase, differently on every run of the
   SAME script (the pinned build's contact-rich NSC runs are not bit-reproducible under OpenMP
   contact ordering). The settle observables are insensitive to the creep: a run that drifted
   1.07 m produced slen/cy identical to a 0.16 m run to 6 decimals, which is why they are what
   the contract grades. (Investigating the drift did surface a real rod-anchor mis-translation
   in the draft reference, (1 + 0.4*sz)*sz instead of the demo's mirrored 1.4*sz, fixed and
   verified against the C++ coordinates, but the drift itself could not have discriminated it.)
4. The nondeterminism is LOAD-CONDITIONED: back-to-back runs on a quiet machine are
   bit-identical, but under concurrent system load thread contention perturbs the bounce phase
   and the settle values move a few mm (a full-suite sweep, run alongside other work, failed
   the driven turn's original ride band once). Bands are therefore sized to contention-level
   variance, and the driven turn's monotonic-progress window starts after the bounce (t >= 2).
3. Runtime: ~8 s (turns 1-2) / ~15 s (turn 3) wall. Timeout 300.

## Gate (de-scoped judge)

1. L1: runs headless on pinned pychrono10 (timeout 300 s); emits `out.csv` (`t,cy,slen,cz`;
   turn 3 `t,cy,slen,dist,speed`) + one JSON line.
2. L2 (minimal): `ChLinkDistance`, `ChLinkTSDA`, `ChLinkMotorRotationTorque`, `PSOR`,
   `ChLinkLockRevolute`.
3. L3 (measured): the spring/ride/drive bands above, all derived from the logged CSV.

## Verify

```
conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/cpp_car_suspension --turn N            # 100
conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/cpp_car_suspension --turn 1 demo_data_10/cpp_car_suspension/samples/good_candidate.py   # 100
conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/cpp_car_suspension --turn 1 demo_data_10/cpp_car_suspension/samples/bad_candidate.py    # 40 (invariant-fail, chassis mass mis-transcribed 15)
```
