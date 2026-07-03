# gear -- task contract (v2.0, PyChrono 10.0)

Mechanism / gear-train axis: transmission-ratio constraints (`ChLinkLockGear`), motor actuation,
and the counter-rotation sign convention, graded from the candidate's own logged trajectory.

## Independent oracle (anti-circularity)

`oracle.py` (stdlib math, NO Chrono) fixes the targets from exact rigid-transmission kinematics:
an external spur mesh matches pitch-line velocities, so `w_out = -(r_in/r_out) * w_in` (the sign IS
physics: external gears counter-rotate); two external meshes restore the sense. No integration is
involved; the identities are closed form, so oracle values are exact. Two-way validation: the
Chrono references agree with the oracle to ~13 significant digits (the constraint is holonomic).

## Turns

1. Create: 2:1 train (r1=0.2, r2=0.4, motor +4 rad/s about global Z). Targets: w1 = +4.0,
   w2 = -2.0 rad/s.
2. Modify: driven gear enlarged to r2=0.6 (3:1). Targets: w1 = +4.0, w2 = -4/3 rad/s (catches
   "ratio not updated": an unmodified candidate sits at -2.0, a 50% error).
3. Extend: second stage, gear C (r3=0.2) meshing with B. Targets: w2 = -4/3 (unchanged) and
   w3 = +4.0 rad/s; the POSITIVE sign discriminates (one mesh or belt-like behavior lands at -4,
   a 200% error).

## Gate (de-scoped judge)

1. L1: runs headless on pinned pychrono10; emits `out.csv` (`t,w1,w2[,w3]`) + one JSON line.
2. L2 (minimal): `ChLinkLockGear`, `SetTransmissionRatio`, `ChLinkMotorRotationSpeed`,
   `ChLinkLockRevolute`.
3. L3 (measured): tail means (t >= 2.0 s) of each logged angular velocity vs the oracle targets,
   rel_tol 0.05, sign-enforcing.

## Authoring findings (recorded, also in docs/DELTAS_10.md)

1. `ChLinkLockPulley` (the original turn-3 belt design) with demo-style shaft frames enforces
   omega_out/omega_in = tau + 2 rather than the textbook tau = rp1/rp2 (measured; independent of
   shaft distance). A candidate implementing correct belt physics would be graded wrong, so the
   belt stage was replaced by a second gear mesh.
2. `ChLinkLockGear` can take up to ~1 s of simulated time to fully engage (phase capture) for a
   legitimately-built train whose initial phase differs from the reference (observed with the
   mirror-mounted control candidate). The measurement window is therefore the t >= 2.0 s tail;
   with the 0.5 s window the correct control was mis-graded at 40.

## Verify

```
conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/gear --turn N            # 100
conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/gear --turn 1 demo_data_10/gear/samples/good_candidate.py   # 100
conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/gear --turn 1 demo_data_10/gear/samples/bad_candidate.py    # 40 (invariant-fail)
```
