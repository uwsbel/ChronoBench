# Task contract: mass_spring_damper (3-turn, oracle-grounded)

A linear damped spring-mass oscillator, the second task taken fully through the hardened template. It
demonstrates the template's three pillars: multi-turn staging, judge-DERIVED (un-gameable) observables,
and INDEPENDENT-oracle ground truth.

- **Axis:** mechanism kinematics & constraints (force element + damped/forced dynamics, instrumented).
- **Simulator:** PyChrono 10.0 (CPU, headless).
- **System:** mass m=1.0 kg on a translational spring-damper (`ChLinkTSDA`) to a fixed ground, no gravity,
  spring stiffness k=100 N/m, rest length 1.0 m, released from x0=0.1 m off equilibrium. wn=sqrt(k/m)=10 rad/s.

## Turns

1. **create** (`truth1.py`, c=2 -> zeta=0.1): report the damped period and damping ratio.
2. **modify** (`truth2.py`, c=6 -> zeta=0.3): heavier damping; same observables.
3. **extend** (`truth3.py`): add a resonant driving force F(t)=1.0*sin(10 t) N; report the steady-state
   amplitude. `pyinput2.py`=truth1, `pyinput3.py`=truth2 (the model modifies the correct prior turn).

## Ground truth = INDEPENDENT oracle (not a Chrono run)

Targets come from `oracle.py` (numpy only, NO Chrono): the linear oscillator m*x''+c*x'+k*x=F(t) solved
BOTH closed-form and by high-fidelity RK4 (dt=1e-5). The two agree to ~6 digits, and the Chrono reference
then agrees with the oracle to ~4 significant figures (two-way validation). This tests "matches the true
physics", not "matches our Chrono run". Reproduce: `conda run -n chronobench python oracle.py`.

| Turn | Observable | Oracle target | Chrono `truth` | tol |
|------|-----------|---------------|----------------|-----|
| 1 | period_d (s) | 0.6315 | 0.6315 | 5% |
| 1 | zeta | 0.1 | 0.1 | 20% |
| 2 | period_d (s) | 0.6587 | 0.6586 | 5% |
| 2 | zeta | 0.3 | 0.3000 | 20% |
| 3 | ss_amp (m) | 0.016667 | 0.016667 | 15% |

Method note: the period is MEASURED by crossing the known equilibrium (0), NOT the empirical mean; for a
decaying oscillation the transient makes the window-mean nonzero and biases mean-crossing period (this
pilot found a 1.3%-6.8% bias that way, now fixed in `scoring/judge_v2.py:_period`).

## Judge-derived observables (anti-gaming)

The graded values are computed by the judge FROM the emitted `out.csv` (`derive` block), not read from the
model's JSON: `period_d_meas` (period of `disp`), `zeta_meas` (log_decrement of `disp`), `ss_amp_meas`
(max |disp| for t>=3 s). A model cannot pass by printing the analytic answer.

## Scoring (tunable, per-task, human-set)

`contract.json -> scoring`: weights L1/L2/L3 = 0.30/0.20/0.50 and `invariant_fail_cap` = 40. Any failed L3
invariant caps the score at 40 (wrong physics is clearly penalized); L1 failure is 0. Edit these numbers to
retune this task; omit the block to inherit the global default in `judge_v2.py`.

## Gate self-check

`conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/mass_spring_damper --turn {1,2,3}`
(references pass -> 100). Samples: `samples/good_candidate.py` (correct, different style -> ~100),
`samples/bad_candidate.py` (k=400 typo, runs but wrong -> capped at 40).
