# mjcf_double_pendulum -- task contract (v2.0, PyChrono 10.0)

Second task of the CROSS-SIMULATOR CONVERSION shape (MuJoCo MJCF -> PyChrono): the prompt embeds
an authored two-rod MJCF whose defining feature is body NESTING: rod 2 is a child of rod 1 and
its `pos="0 0 -1"` places its hinge at rod 1's TIP. That one attribute is what couples the rods
into a double pendulum, and (with the inertial COM offsets) it is exactly what an imperative
conversion must turn into explicit body placement. Small-angle releases keep the motion
quasi-periodic: no chaos, tight bands.

## Oracle posture (independent RK4, tight)

`oracle.py` (stdlib math, NO Chrono, NO MuJoCo): the exact two-rod Lagrangian equations
integrated with RK4 at dt = 1e-5, reporting window maxima over the 10 s run. Two-way validation
on the pinned build: theta2_max 0.17016/0.13318/0.12741 vs oracle 0.1702/0.1332/0.1274, and
rel_max 0.1511/0.1385/0.208 matching to 4 decimals; the logged total energy drifts 1.7e-5 to
5.5e-5 relative (band 1e-3).

## Turns

1. Convert: equal rods (m = 1, L = 1), release (0.1, 0). Energy exchange pumps the SECOND arm
   to 0.1702 rad: theta2_max in [0.155, 0.185]; theta1_max in [0.09, 0.112]; e_drift <= 1e-3.
2. Modify: the MJCF's rod 2 grows to L = 1.5 at the same linear density (mass 1.5, COM -0.75,
   I 0.28125): theta2_max in [0.120, 0.147] (an unmodified candidate at 0.1702 fails high).
3. Extend: bent release (0.1, -0.1) on the same model. The counter-phase mode surges: the
   relative angle th2 - th1 peaks in [0.19, 0.23] (ref 0.208; a straight-release candidate at
   0.1385 fails low); theta2_max in [0.115, 0.142].

## Shape notes (worth knowing)

1. The bad control drops the nested body pos: rod 2's hinge lands on the FIXED pivot, which
   never moves, so arm 2 receives no excitation and hangs still (theta2_max ~ 0 vs the required
   0.17). Declarative nesting -> imperative placement is the skill probed.
2. The energy column doubles as an instrumentation test: the candidate must assemble
   KE_trans + KE_rot (local frame!) + PE from body states; a wrong rotational term shows up as
   apparent drift.
3. Runtime: ~4 s per turn. Timeout 120.

## Gate (de-scoped judge)

1. L1: runs headless on pinned pychrono10 (timeout 120 s); emits `out.csv` (`t,th1,th2,rel,e`)
   + one JSON line.
2. L2 (minimal): revolute joints (two of them on turn 1), explicit gravity.
3. L3 (measured): the amplitude/relative-angle bands + the energy-drift bound, all derived
   from the logged CSV.

## Verify

```
conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/mjcf_double_pendulum --turn N            # 100
conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/mjcf_double_pendulum --turn 1 demo_data_10/mjcf_double_pendulum/samples/good_candidate.py   # 100
conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/mjcf_double_pendulum --turn 1 demo_data_10/mjcf_double_pendulum/samples/bad_candidate.py    # 40 (invariant-fail, nested body pos dropped)
```
