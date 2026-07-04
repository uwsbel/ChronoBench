# yaml_mbs -- task contract (v2.0, PyChrono 10.0)

Data-import / declarative-model axis: authoring a mechanism in Chrono's MBS-YAML schema (bodies,
joints, DISTANCE constraints, declared motors, plus the simulation and solver files) and driving
it through `chrono::parsers::ChParserMbsYAML`. The candidate SCRIPT writes the YAML files inline
and then loads them, so the graded skill is declarative-model authoring, not just loading; grading
stays single-file.

## Independent oracle (anti-circularity)

`oracle.py` (stdlib math, NO Chrono): the inline slider-crank closed forms (the same physics
identity as the `slider_crank` task, different numbers): stroke = 2r exactly; peak slider speed =
max |omega dx/dtheta|. Two-way validation: references measure stroke 0.500003 / 0.800004 /
0.800016 and peak speed 2.711 vs the oracle's 2.710687.

## Turns

1. Create: declare the full model (ground, motor-driven crank r = 0.25, DISTANCE rod l = 1.0,
   prismatic slider), load, run. Stroke = 0.5 m.
2. Modify: r = 0.4 INSIDE the declared model, a geometry-coupled YAML edit (slider location,
   prismatic location, and both constraint points move together). Stroke = 0.8 m.
3. Extend: double the declared drive speed (omega = 2 pi). Stroke unchanged; peak slider speed
   2.7107 m/s (catches "speed not changed" at 50% error).

## Gate (de-scoped judge)

1. L1: runs headless on pinned pychrono10; emits `out.csv` (`t,slider_x`) + one JSON line.
2. L2 (minimal): `ChParserMbsYAML`, `Populate`, a declared motor (`actuation_type|ROTATION`), a
   declared `DISTANCE` constraint.
3. L3 (measured): stroke (range of slider_x) each turn; turn 3 adds the peak slider speed
   (max|d slider_x/dt|).

## Authoring findings (recorded, also in docs/DELTAS_10.md)

1. The 10.0 `ChParserMbsYAML` wrapper takes (sim_yaml[, verbose]), NOT the (model, sim, verbose)
   signature the shipped `demo_YAML_mbs.py` uses; the simulation YAML references the model and
   solver files, and those references resolve RELATIVE TO THE SIM FILE'S DIRECTORY (absolute paths
   inside break; pass an absolute path to the constructor, bare filenames inside).
2. DISTANCE-constraint `point1`/`point2` are GLOBAL coordinates of the initial assembly, not
   body-local. Misreading them as body-local pins the rod to the pivot: the model runs cleanly and
   the slider simply never moves. That exact bug is the shipped bad control (stroke 0, capped 40);
   only the measured stroke catches it.
3. The motor's CONSTANT SPEED value is applied in rad/s (the parser's use_degrees default affects
   angles/phases, not this path), verified by measuring the crank rate.

## Verify

```
conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/yaml_mbs --turn N            # 100
conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/yaml_mbs --turn 1 demo_data_10/yaml_mbs/samples/good_candidate.py   # 100
conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/yaml_mbs --turn 1 demo_data_10/yaml_mbs/samples/bad_candidate.py    # 40 (invariant-fail, stroke 0)
```
