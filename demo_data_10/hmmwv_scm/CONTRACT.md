# hmmwv_scm -- task contract (v2.0, PyChrono 10.0)

Vehicle-terramechanics coupling (the panel's "coupling is the discriminator" axis): a full HMMWV
(rigid tires, SMC, AWD shafts powertrain) driven straight with a ramped throttle, first on rigid
ground, then on SCM deformable terrain, then on softened soil. Settles the suite's vehicle pick
(HMMWV). Graded from the candidate's own logged trajectory: distance, speed, and the rut depth
sampled at the rear-left wheel's location (`-terrain.GetHeight(...)` per step).

## Oracle posture (the plate_sinkage_scm pattern, stated honestly)

`oracle.py` (stdlib math, NO Chrono) anchors the RUT bands: a static Bekker-Wong solve for a rigid
cylinder (per-wheel load m*g/4 ~= 6009 N, R = 0.4699 m, b = 0.254 m) gives z = 0.043 m at
Kphi = 2e6 and z = 0.103 m at Kphi = 5e5; the graded bands are [0.5x, 4x] (firm) and [1.26x, 4x]
(soft; the lower edge is raised above the firm-soil reference so "soil not softened" fails). A
moving, slipping, multi-pass wheel cuts deeper than the static value, which the upper multiplier
absorbs (references measure 2.7x and 2.35x). The SPEED and DISTANCE bands are CALIBRATED on the
pinned build and frozen: rigid 11.39 m / 6.58 m/s; firm SCM 4.54 m/s; soft SCM 4.05 m/s.

## Turns

1. Create: rigid baseline. dist in [8.5, 14.5]; final speed in [5.2, 8.0]; forward progress
   monotonic.
2. Modify: swap to SCMTerrain (Kphi 2e6, n 1.1, active domain, 0.05 m grid). Rut in
   [0.0215, 0.172] m (ref 0.116); speed in [3.4, 5.2] (a still-rigid candidate at 6.58 fails);
   monotonic + a still-drives distance floor.
3. Extend: soften to Kphi 5e5. Rut in [0.13, 0.41] m (ref 0.242; an un-softened candidate at
   0.116 fails low); speed in [3.0, 5.0].

## Calibration findings (probe-derived, worth knowing)

1. DISTANCE does NOT discriminate rigid vs firm SCM here (11.39 vs 11.79 m): the SCM run
   accelerates with less wheelspin even though it ends slower. FINAL SPEED discriminates cleanly
   (6.58 / 4.54 / 4.05), so speed carries the motion-resistance invariant.
2. Wheel-center drop is polluted by spawn settle and pitch; the robust sinkage observable is the
   terrain's own deformed height under the wheel (rut), which is also what Bekker anchors.
3. `pychrono.vehicle` imports only under the ACTIVATED conda env (see docs/DELTAS_10.md).

## Gate (de-scoped judge)

1. L1: runs headless on pinned pychrono10 (timeout 300 s rigid / 600 s SCM); emits `out.csv`
   (`t,dist,speed,rut`) + one JSON line.
2. L2 (minimal): `pychrono.vehicle`, `HMMWV_Full|HMMWV_Reduced`, `SetThrottle`, plus
   `RigidTerrain` + `SetCollisionSystemType` (turn 1) or `SCMTerrain` + `SetSoilParameters`
   (turns 2-3).
3. L3 (measured): the bands above, all derived from the logged CSV.

## Verify

```
conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/hmmwv_scm --turn N            # 100
conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/hmmwv_scm --turn 1 demo_data_10/hmmwv_scm/samples/good_candidate.py   # 100
conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/hmmwv_scm --turn 1 demo_data_10/hmmwv_scm/samples/bad_candidate.py    # 40 (invariant-fail, throttle typo)
```
