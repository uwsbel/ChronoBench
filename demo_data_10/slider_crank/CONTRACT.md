# Task contract: slider_crank (3-turn, oracle-grounded)

A motor-driven inline slider-crank (closed kinematic loop: motor + two revolute joints + one prismatic
joint). Fourth task through the hardened template; the mechanism/closed-loop case.

- **Axis:** mechanism kinematics & constraints (closed loop, motor actuation, instrumented).
- **Simulator:** PyChrono 10.0 (CPU, headless).
- **System:** crank (radius r) driven at omega by a rotational-speed motor, connecting rod (length l=1.5 m),
  piston on a prismatic slide; mounted on a fixed base.

## Turns

1. **create** (`truth1.py`, r=0.4, omega=pi): report the piston stroke.
2. **modify** (`truth2.py`, r=0.6): larger crank -> larger stroke.
3. **extend** (`truth3.py`, r=0.6, omega=2*pi): double the drive speed and also report the peak piston
   speed. `pyinput2.py`=truth1, `pyinput3.py`=truth2.

## Ground truth = INDEPENDENT oracle (not a Chrono run)

Targets come from `oracle.py` (stdlib math, NO Chrono): the analytic inline slider-crank kinematics
x(theta)=r*cos(theta)+sqrt(l^2 - r^2 sin^2 theta). Stroke = 2*r (exact); peak speed = max|omega*dx/dtheta|
(scanned numerically). The Chrono reference agrees with the oracle to ~5 sig figs (two-way check).

| Turn | Observable | Oracle target | Chrono `truth` | tol |
|------|-----------|---------------|----------------|-----|
| 1 | stroke (m) | 0.8 | 0.8000 | 6% |
| 2 | stroke (m) | 1.2 | 1.2000 | 6% |
| 3 | stroke (m) | 1.2 | 1.2000 | 6% |
| 3 | peak_speed (m/s) | 4.066 | 4.0661 | 8% |

## Judge-derived observables (anti-gaming)

Graded values are computed by the judge FROM the emitted `out.csv` (`t,piston_x`), not read from the
model's JSON: `stroke_meas` = range of `piston_x`; `peak_speed_meas` = max |d(piston_x)/dt| (finite
difference). A model must produce the correct piston trajectory, not just print a number.

## Scoring (tunable, per-task, human-set)

`contract.json -> scoring`: weights 0.30/0.20/0.50, `invariant_fail_cap` = 40. Any failed invariant caps
the score; L1 failure is 0. Edit to retune; omit to inherit the global default.

## Gate self-check

`conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/slider_crank --turn {1,2,3}` (references
-> 100). Samples: `samples/good_candidate.py` (correct, crank at origin, different style -> ~100),
`samples/bad_candidate.py` (crank radius 0.25 instead of 0.4, runs but wrong stroke -> capped at 40).
