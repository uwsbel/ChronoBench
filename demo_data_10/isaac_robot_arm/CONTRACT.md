# isaac_robot_arm -- task contract (v2.0, PyChrono 10.0)

The PRACTICAL Isaac Sim / USD conversion task, built around the ecosystem's two great SILENT
unit hazards, both carried in STAGE METADATA rather than in any prim: `metersPerUnit` (generic
USD assets default to 0.01, i.e. centimeters, while Isaac convention is 1.0; vast amounts of
real content are cm-authored) and `upAxis` (USD default Y, Isaac convention Z). The prompt
embeds compact hand-authored USDA stages (`source/arm3dof_cm_v1..2.usda` in CENTIMETERS, Z-up,
where even gravityMagnitude reads 981; `source/arm3dof_m_yup_v3.usda`, the SAME arm re-exported
at meters, Y-up) for a 3-DOF arm with high-stiffness position DriveAPIs; the controller
trajectory is decreed in the prompt with the imposed-motion mapping (the sanctioned-mapping
pattern shared with mjcf_robot_arm).

## Oracle posture (closed-form FK, tight; USD-side honesty)

`oracle.py` (stdlib math): FK swept over the decreed reach-out-and-down arc (all joint
references zero at t = 0 and t = 10). Two-way validation on the pinned build: every extremum
to 4 decimals (deepest reach 0.5311 / 0.5253, sweep 0.3556 / 0.4334) and home return exact to
1e-12 (1.3 / 1.5 m). The turn-3 invariance is verified LITERALLY: the Y-up meter build
reproduces the turn-2 values to 12 digits. The USDA stages are usd-core linted (parse clean;
3 bodies / 3 joints / 3 drives each; metadata 0.01/Z, 0.01/Z, 1.0/Y as intended); their Isaac
Sim replay is PENDING (this AMD machine cannot run Isaac): an explicitly recorded to-do for
the NVIDIA machine.

## Turns

1. Convert the centimeter stage. Home h_final in [1.29, 1.31] (an un-scaled candidate homes at
   130); deepest reach in [0.50, 0.56]; sweep |hy| in [0.33, 0.38]; final radius <= 0.02.
2. Modify: the stage's forearm grows 60 -> 80 (in ITS units: 0.80 m), MassAPI scaled. Home in
   [1.49, 1.51]; sweep in [0.41, 0.46]; reach in [0.495, 0.555] (nearly unchanged by design:
   home and sweep carry the discrimination).
3. Extend: the SAME arm re-exported with metersPerUnit = 1.0 and upAxis = Y. The graded truth
   is INVARIANCE: every turn-2 band verbatim, with h = the up-axis coordinate and hy = the
   yaw-sweep horizontal per the canonical reporting schema. Rescaling twice lands at 0.015 or
   150; reporting z as height in the Y-up world hovers near 0.

## Shape notes (worth knowing)

1. The reporting schema (t, hx, hy, h, r with h = up-axis coordinate) is defined in turn 1
   precisely so turn 3 can grade frame-convention handling without changing a single band.
2. The trajectory was REDESIGNED during authoring: the first draft folded the arm back on
   itself and the yaw-sweep observable collapsed to ~1 cm; the committed reach-out arc (elbow
   bending with the shoulder) aligns the yaw peak with full extension (sweep 0.36-0.43 m).
3. Under imposed motion the MassAPI values do not affect the trajectory, but the stages author
   them correctly in their own units (inertia in kg cm^2 in v1/v2) to keep the artifact honest.
4. Runtime: ~4 s per turn. Timeout 120.

## Controls

Good = stylistic variant converting the stage's centimeters once at the top (~100). Bad = the
metersPerUnit slip: stage numbers read as meters, a 130 m robot that simulates happily and
"homes" at h = 130 (runs clean, fails every band).

## Gate (de-scoped judge)

1. L1: runs headless on pinned pychrono10 (timeout 120 s); emits `out.csv` (`t,hx,hy,h,r`) +
   one JSON line.
2. L2 (minimal): `ChLinkMotorRotationAngle`, explicit gravity, a `ChFunction` driving the
   motors.
3. L3 (measured): the FK home/reach/sweep/radius bands, all derived from the logged CSV.

## Verify

```
conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/isaac_robot_arm --turn N            # 100
conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/isaac_robot_arm --turn 1 demo_data_10/isaac_robot_arm/samples/good_candidate.py   # 100
conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/isaac_robot_arm --turn 1 demo_data_10/isaac_robot_arm/samples/bad_candidate.py    # 40 (invariant-fail, metersPerUnit ignored)
```
