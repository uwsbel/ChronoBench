# fsi_object_drop -- task contract (v2.0, PyChrono 10.0, HIP build)

Fluid-solid interaction on the SPH axis: a rigid sphere (R = 0.12 m) dropped into a 0.8 x 0.8 x
0.5 water tank (`ChFsiProblemCartesian`, spacing 0.025, the demo_FSI_ObjectDrop idiom), graded
from the candidate's own logged sphere-center trajectory. First suite task to run through the
judge's per-task environment registry (`run.env_id = "hip"`: the AMD-GPU source build; see
`scoring/envs.template.json` and docs/BUILD_HIP.md).

## Oracle posture (Archimedes, plus an honest SPH bias note)

`oracle.py` (stdlib math, NO Chrono) solves the spherical-cap draft equation
x^3 - 3x^2 + 4*ratio = 0 (x = h/R): density 500 floats with its center AT the surface (x = 1
exactly), density 900 floats 0.073 m below (x = 1.6084), density 2500 has no flotation solution
and rests on the tank floor (center -0.38 m vs surface).

Calibration on the pinned build exposed a systematic SPH artifact the bands must own: the BCE
marker skin makes the sphere hydrodynamically larger than its geometric radius (~half a spacing),
adding ~35% effective buoyant volume. Every floating case therefore settles ~0.04-0.05 m HIGH of
Archimedes (500 measures +0.0425, not 0.0; 900 measures -0.0096, not -0.073), and a 1.2 density
ratio still floats, which is why the sinking turn uses 2.5. The bands are Archimedes-ANCHORED
(ordering, spacing, and the float/sink split all come from the oracle) but calibrated-and-frozen
around the measured truth values; they are NOT re-derived from a Chrono run at grade time.

## Turns

1. Create: density 500, 3 s. Settled center (tail mean, t >= 2) in [0.018, 0.075] vs the surface
   (ref +0.0425, tail osc 0.012); released ABOVE the water (max zrel >= 0.10).
2. Modify: density 900. Settled center in [-0.045, 0.012] (ref -0.0096); an unmodified
   density-500 candidate (+0.0425) fails high.
3. Extend: density 2500, 5 s, real collision shapes + a fixed floor with top face at z = 0.
   Settled center in [-0.42, -0.30] (rests on the floor, center R above it). A still-floating
   candidate fails high; one that omits `CreateCollisionShapes` falls THROUGH the floor and
   fails low (the calibration probe measured z ~ -99 for exactly that bug).

## Calibration findings (probe-derived, worth knowing)

1. The BCE-skin float bias above: +0.0425 at ratio 0.5, +0.063 at ratio 0.9 (measured minus
   Archimedes). Direction and rough magnitude follow from (R + h/2)^3 / R^3 ~ 1.35 at
   R = 0.12, h = 0.025.
2. Density separation at spacing 0.025 is ~0.011 m of settled height per 100 kg/m^3, so the
   original 500/700/1200 design had turn gaps comparable to the surface oscillation; the frozen
   design (500/900/2500) keeps every pair of bands disjoint with margin.
3. `ChBodyGeometry` BCE geometry does NOT create Bullet collision shapes;
   `EnableCollision(True)` alone lets the sphere fall through the floor. The sinking turn needs
   `geometry.CreateCollisionShapes(body, family, contact_method)` (see docs/DELTAS_10.md).
4. With collision shapes in place the sink case is EXACT: measured settled center -0.3800 vs the
   surface (floor + R to four decimals), tail oscillation 0.0, so the turn-3 band is the loosest
   of the three yet the most decisive.
5. Runtime on gfx1151: ~70 s per 3 s turn, ~152 s for the 5 s turn (timeout 600).

## Gate (de-scoped judge)

1. L1: runs headless through the `hip` env registry entry (timeout 600 s); emits `out.csv`
   (`t,z,zrel`) + one JSON line.
2. L2 (minimal): `pychrono.fsi`, `ChFsiProblemCartesian`, `SPHParameters`, `AddRigidBody`,
   `DepthPressurePropertiesCallback`, plus `CreateCollisionShapes` + `ChBodyEasyBox` (turn 3).
3. L3 (measured): the settled-center bands above + the released-above-water check, all derived
   from the logged CSV.

## Verify

```
conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/fsi_object_drop --turn N            # 100
conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/fsi_object_drop --turn 1 demo_data_10/fsi_object_drop/samples/good_candidate.py   # 100
conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/fsi_object_drop --turn 1 demo_data_10/fsi_object_drop/samples/bad_candidate.py    # 40 (invariant-fail, density left at 1000)
```
