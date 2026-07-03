# solver_nsc_smc -- task contract (v2.0, PyChrono 10.0)

Solver / contact-method axis (a first-class axis per the design red-team): the NSC
(complementarity) vs SMC (penalty/compliant) contact formulations, restitution physics, collision
system setup, and contact-parameter hygiene, graded from the candidate's own logged trajectory.

## Independent oracle (anti-circularity)

`oracle.py` (stdlib math, NO Chrono): exact bouncing-ball impact kinematics. Ball bottom released
from rest at h0 above the floor: first-bounce apex = e^2 * h0, at t = sqrt(2 h0/g) * (1+e). No
integration; closed form. Two-way validation on the pinned build (dt = 2e-4): NSC e=0.7 -> 0.4902
(+0.03%), NSC e=0.9 -> 0.8101 (+0.01%), SMC e=0.9 with E_contact=1e8 -> 0.8155 (+0.7%).

## Turns (h0 = 1 m, R = 0.1 m, density 1000, friction 0.3, dt = 2e-4 s, 1.5 s)

1. Create: NSC drop, e = 0.7. Apex = 0.49 m.
2. Modify: e = 0.9. Apex = 0.81 m (an "e not changed" candidate sits at 0.49 and fails at 40%).
3. Extend: switch the formulation to SMC (same e = 0.9, contact Young's modulus 1.0e8 Pa). The
   physics must NOT change with the method: apex stays ~0.81 m.

## Gate (de-scoped judge)

1. L1: runs headless on pinned pychrono10; emits `out.csv` (`t,y_bot`) + one JSON line.
2. L2 (minimal): `ChSystemNSC`/`ChSystemSMC`, `ChContactMaterialNSC`/`...SMC`, `SetRestitution`,
   `SetCollisionSystemType`.
3. L3 (measured): apex1 = max ball-bottom height after t = 0.5 s vs e^2*h0, rel_tol 0.12; plus a
   no-penetration floor check (min y_bot >= -0.02 m).

## Authoring findings (recorded, also in docs/DELTAS_10.md)

1. **NSC restitution has an isolated bad time-step pocket.** At dt = 1.0e-4 the e=0.7 drop rebounds
   to only 54% of the ideal apex (dt = 5e-5: 95%), while 2e-5, 2e-4, 5e-4, and 1e-3 are all within
   0.2%. The task pins dt = 2e-4 (exact, cheap, wide safe neighborhood); the 12% band forgives
   every probed step except the 1e-4 pocket.
2. **The SMC default contact stiffness is far too soft for rigid-body bouncing.** With the default
   material Young's modulus the ball sinks ~9 cm into the floor mid-impact (apex still ~right, the
   physics mushy). The turn-3 reference and prompt set E_contact = 1.0e8 Pa (compliant overlap
   ~7 mm), and the no-penetration invariant catches the too-soft default.
3. The bad control candidate calls `SetRestitution` on an UNUSED material object (so L2's text
   check sees it) while the attached material keeps dead restitution; only the measured apex
   (0.16 m vs 0.49 m) catches it, a clean demonstration of why L3 grades behavior, not text.

## Verify

```
conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/solver_nsc_smc --turn N            # 100
conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/solver_nsc_smc --turn 1 demo_data_10/solver_nsc_smc/samples/good_candidate.py   # 100
conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/solver_nsc_smc --turn 1 demo_data_10/solver_nsc_smc/samples/bad_candidate.py    # 40 (invariant-fail)
```
