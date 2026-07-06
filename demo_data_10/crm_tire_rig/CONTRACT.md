# crm_tire_rig -- task contract (v2.0, PyChrono 10.0, HIP build)

Tire-level terramechanics on CRM (the SPH continuum granular model): a Polaris wheel with a
rigid mesh tire in a `ChTireTestRig` on CRM soil (demo_VEH_TireTestRig_CRM idiom), carriage at
0.2 m/s, wheel speed imposed, graded from the candidate's own logged spindle height, drawbar
pull, and longitudinal slip. Runs through the judge's env registry (`run.env_id = "hip"`), and is
the CRM counterpart to the SCM tasks (plate_sinkage_scm, hmmwv_scm): same physical questions,
different terramechanics solver.

## Oracle posture (the hmmwv_scm pattern, stated honestly)

No closed form exists for CRM soil, so three things carry the physics:

1. Cross-turn terramechanics LAWS: doubling the load MUST sink the tire deeper (turn 1 -> 2:
   -0.5457 -> -0.5882, bands disjoint), and tripling the wheel speed at the same load MUST sink
   it deeper still via slip-sinkage (turn 2 -> 3).
2. A KINEMATIC slip anchor: the rig imposes BOTH the carriage speed and the wheel speed, so
   `GetLongitudinalSlip` = omega*R/v - 1 is set by pure kinematics, load-independent, and
   verifiable by hand: R = 0.330 m gives 0.7291 at 10 RPM and 4.1873 at 30 RPM (measured values
   match to 4 decimals). The slip band pins the wheel-speed change exactly.
3. Absolute sinkage/drawbar bands CALIBRATED on the pinned build and frozen (NOT re-derived at
   grade time): z tail means -0.5457 / -0.5882 (2500 / 5000 N at 10 RPM); drawbar 367 / 147 N
   (drawbar DROPS under the heavier load: more motion resistance at deeper sinkage).

## Turns

1. Create: rig at 2500 N, 10 RPM, 5 s. Settled spindle z (tail mean, t >= 4) in
   [-0.567, -0.525]; slip in [0.70, 0.76]; drawbar in [150, 600] N.
2. Modify: load doubled to 5000 N. z in [-0.610, -0.572] (an unmodified candidate at -0.5457
   fails high); slip unchanged (kinematic); drawbar in [60, 300] N.
3. Extend: wheel speed tripled to 30 RPM at 5000 N. Slip in [4.0, 4.4] (an unmodified 10 RPM
   candidate at 0.7291 fails far low); z in [-0.66, -0.60] (ref -0.6193, slip-sinkage: deeper
   than turn 2, whose -0.5882 fails high); drawbar in [400, 1300] N (ref 831).

## Calibration findings (probe-derived, worth knowing)

1. The rig SELF-SEQUENCES: drop phase ends at t = 2, motor activation and "enable measurements"
   at t = 3. Forces/DBP read 0 before that, so the graded tail window is the last second of a
   5 s run. `rig.GetPos()` reads 0 for height; the live observable is
   `rig.GetSpindle().GetPos().z`.
2. Slip is kinematic (both speeds imposed): identical 0.7291 at 2500 N and 5000 N. It cannot
   discriminate load, but it pins wheel-speed/RPM-conversion errors exactly; sinkage carries the
   load law instead.
3. Slip-sinkage is real and large here: at 2500 N, going 10 -> 30 RPM deepened the settled z
   from -0.5457 to -0.5881 (as deep as doubling the load), with drawbar 763 N; at 5000 N the
   same speed change gave -0.5882 -> -0.6193 with drawbar 831 N. Drawbar is NOT monotone in
   load at fixed slip (367 -> 147 N when the load doubles) but rises steeply with slip.
4. Run-to-run (GPU SPH) reproducibility is very different per observable: settled spindle height
   repeats to <1 mm and slip to ~1e-4 across independent runs, but drawbar pull swings by tens of
   percent (turn-2 config measured 147 N in calibration, 225 N at gate time). Hence tight z/slip
   bands and deliberately generous dbp bands.
5. Runtime on gfx1151: ~113-134 s per 5 s turn at spacing 0.02 (~91k SPH particles); timeout 900.

## Gate (de-scoped judge)

1. L1: runs headless through the `hip` env registry entry (timeout 900 s); emits `out.csv`
   (`t,z,dbp,slip`) + one JSON line.
2. L2 (minimal): `pychrono.vehicle`, `ChTireTestRig`, `SetTerrainCRM`, `SetNormalLoad`,
   `SetAngSpeedFunction`.
3. L3 (measured): the sinkage/slip/drawbar bands above, all derived from the logged CSV.

## Verify

```
conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/crm_tire_rig --turn N            # 100
conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/crm_tire_rig --turn 1 demo_data_10/crm_tire_rig/samples/good_candidate.py   # 100
conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/crm_tire_rig --turn 1 demo_data_10/crm_tire_rig/samples/bad_candidate.py    # 40 (invariant-fail, load typo 250 N)
```
