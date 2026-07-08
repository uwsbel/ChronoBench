# mjcf_cartpole -- task contract (v2.0, PyChrono 10.0)

First task of the CROSS-SIMULATOR CONVERSION shape (MuJoCo MJCF -> PyChrono): the prompt embeds
a compact authored MJCF (`source/cartpole_v1..3.xml`, explicit inertial blocks, no contacts,
no actuators) and the candidate must translate the SEMANTICS by hand: PyChrono 10 has no MJCF
importer (verified: the wrapper exposes only URDF and YAML parsers; the Chrono source tree has
zero MJCF references), so this genuinely tests cross-ecosystem model reading: z-up gravity,
slide -> prismatic, hinge -> revolute, and, the classic trap, the `inertial pos` COM offset.
The physics deliberately overlaps existing suite members (pendulum, slider) SO THAT the task
shape is the only new variable: score deltas against those tasks attribute to conversion skill,
not new physics.

## Oracle posture (closed form, tight)

`oracle.py` (stdlib math, NO Chrono, NO MuJoCo): the linearized coupled cart-pole oscillation
omega^2 = m g d / (I_h - m^2 d^2/(M+m)); the mass-ratio term is the free cart's recoil. Periods:
1.5101 s (M = 2), 1.2949 s (M = 0.5), 1.6379 s (cart locked, the plain compound-pendulum limit,
and exactly the import_urdf free-swing period: a cross-task consistency anchor). Two-way
validation on the pinned build: measured 1.5138 / 1.3010 / 1.6402 (0.14-0.5%, inside the 0.14%
amplitude correction plus integrator damping). Bonus invariant: with zero initial horizontal
momentum the SYSTEM COM x is conserved; Chrono holds it to machine zero (range 0.0).

## Turns

1. Convert: free cart M = 2, release 0.15 rad. Period in [1.46, 1.56]; amplitude in
   [0.135, 0.165]; COM-x range <= 1e-3.
2. Modify: the MJCF's cart mass drops to 0.5 (updated file embedded). Lighter cart, more
   recoil, FASTER: period in [1.25, 1.35] (an unmodified candidate at 1.5138 fails high);
   same amplitude and COM checks.
3. Extend: the MJCF loses its slide joint (a MuJoCo body with no joint is WELDED to its
   parent): the cart must be fixed. Period in [1.59, 1.69] (a still-sliding candidate fails
   low); cart-x range <= 1e-4 pins the joint removal.

## Shape notes (worth knowing)

1. The bad control is the conversion trap this task exists for: dropping the `inertial pos`
   COM offset pivots the rod through its own COM, gravity exerts no torque, the pole never
   oscillates, and the period derive goes NaN (invariant-fail). Declarative formats put the
   COM in an attribute; imperative APIs make you PLACE the body: that mismatch is the skill
   under test.
2. MuJoCo semantics that carry grading weight: z-up gravity; joint-less body = weld (turn 3).
3. Runtime: ~3 s per turn. Timeout 120.

## Gate (de-scoped judge)

1. L1: runs headless on pinned pychrono10 (timeout 120 s); emits `out.csv` (`t,x,theta,xcom`)
   + one JSON line.
2. L2 (minimal): prismatic + revolute + explicit gravity (turns 1-2); revolute + gravity +
   a cart-locking construct (turn 3).
3. L3 (measured): the period/amplitude/COM bands above, all derived from the logged CSV.

## Verify

```
conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/mjcf_cartpole --turn N            # 100
conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/mjcf_cartpole --turn 1 demo_data_10/mjcf_cartpole/samples/good_candidate.py   # 100
conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/mjcf_cartpole --turn 1 demo_data_10/mjcf_cartpole/samples/bad_candidate.py    # 40 (invariant-fail, inertial COM offset dropped)
```
