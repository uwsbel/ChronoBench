# Task contract: swig_contact_reporter (3-turn, oracle-grounded)

Drops rigid spheres onto a plane and reads back the resting contacts through a custom Python subclass of
Chrono's C++ contact-reporting callback. Fifth task through the hardened template; the SWIG-extension /
contact-instrumentation case.

- **Axis:** SWIG Python/C++ extension (subclass a C++ callback, object lifecycle, contact instrumentation).
- **Simulator:** PyChrono 10.0 (NSC contact, Bullet collision, headless).
- **System:** N spheres (radius 0.1 m, mass m) dropped a short distance onto a fixed floor, settle ~1.5 s.

## Turns

1. **create** (`truth1.py`, N=4, m=1): report the resting contacts.
2. **modify** (`truth2.py`, N=6, m=1): more spheres.
3. **extend** (`truth3.py`, N=6, m=2): heavier spheres + per-contact inspection.
   `pyinput2.py`=truth1, `pyinput3.py`=truth2.

## Ground truth = INDEPENDENT oracle (not a Chrono run)

Targets come from `oracle.py` (stdlib, NO Chrono): static-equilibrium force balance. Each sphere resting
on the flat ground makes one contact carrying its weight, so n_contacts=N, total normal force = N*m*g,
per-contact = m*g. The Chrono reference agrees to ~0.1% (two-way check).

| Turn | Observable | Oracle target | Chrono `truth` | tol |
|------|-----------|---------------|----------------|-----|
| 1 | n_contacts | >= 4 | 4 | (min) |
| 1 | total force (N) | 39.24 | 39.26 | 15% |
| 2 | n_contacts | >= 6 | 6 | (min) |
| 2 | total force (N) | 58.86 | 58.88 | 15% |
| 3 | total force (N) | 117.72 | 117.76 | 15% |
| 3 | per-contact max (N) | 19.62 | ~19.6 | 20% |

## Judge-derived observables (anti-gaming)

The callback LOGS each contact's normal force to `out.csv` (`contact_id,normal_force`), and the judge
derives `n_contacts_meas` = row count, `force_sum_meas` = sum of the column, and (turn 3)
`per_contact_max` = max of the column. The per-contact-max check (~m*g) ensures the callback reports
INDIVIDUAL contacts, not one lumped total. A model must actually enumerate the contacts and report real
forces, it cannot self-report a scalar.

## Scoring (tunable, per-task, human-set)

`contract.json -> scoring`: weights 0.30/0.20/0.50, `invariant_fail_cap` = 40. Any failed invariant caps
the score; L1 failure is 0.

## Gate self-check

`conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/swig_contact_reporter --turn {1,2,3}`
(references -> 100). Samples: `samples/good_candidate.py` (correct, different style -> ~100),
`samples/bad_candidate.py` (only 2 spheres dropped, runs but too few contacts / too little force -> 40).
