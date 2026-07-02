# Task contract: beam (3-turn, oracle-grounded FEA)

A clamped Euler-Bernoulli cantilever, the third task taken fully through the hardened template and the
FEA/static stress-test for the methodology: an oracle that is an independent FE *solver* (not just a
formula), and un-gameable checking for a *static scalar*.

- **Axis:** FEA structural (mesh + beam elements, section/material, static solve, BC).
- **Simulator:** PyChrono 10.0 (CPU, Pardiso, headless).
- **Beam:** length L=1.0 m, square section b=0.05 m, E=2.0e10 Pa, nu=0.3, rho=7800 kg/m^3, ~10 Euler
  beam elements, one end fully clamped. I = b^4/12 = 5.208e-7 m^4, EI = 10416.7 N*m^2.

## Turns

1. **create** (`truth1.py`): transverse tip point load F=150 N -> tip deflection.
2. **modify** (`truth2.py`): remove the tip load, deflect under SELF-WEIGHT (automatic gravity, g=9.81).
3. **extend** (`truth3.py`): tip load F plus an equal load P=F at the mid-span node -> superposition.
   `pyinput2.py`=truth1, `pyinput3.py`=truth2 (the model modifies the correct prior turn).

## Ground truth = INDEPENDENT oracle (not a Chrono run)

Targets come from `oracle.py` (numpy only, NO Chrono): an independent Euler-Bernoulli cantilever FE
solver (Hermite cubic beam elements) refined to N=50, plus the exact textbook closed forms. FE and
closed form agree, and the Chrono reference then agrees with the oracle (two-way validation).
Reproduce: `conda run -n chronobench python oracle.py`.

| Turn | Loading | Oracle target (m) | Chrono `truth` (m) | tol |
|------|---------|-------------------|--------------------|-----|
| 1 | tip point load F | 0.0048 | 0.0048 | 8% |
| 2 | self-weight q=rho*A*g | 0.00229554 | 0.0023032 (+0.33%) | 8% |
| 3 | tip F + mid-span P (superpose) | 0.0063 | 0.0063 | 8% |

## Un-gameable static scalar (the FEA-specific hardening)

A static tip deflection is a single number a model could just print. So each turn logs the full DEFLECTED
SHAPE to `out.csv` (`x,y` per node) and the judge DERIVES the tip deflection as `max_abs(y)` over the
shape (for downward loading the deflection is monotone to the tip, so max|y| is the tip). A model must
produce a physically consistent deflected curve, not just a scalar.

## Deferred: first-natural-frequency turn

Turn 3 was originally intended as the first bending frequency (f1 ~= 12.93 Hz, T1 ~= 0.0773 s; oracle
computes it via a consistent-mass FE eigenproblem). It is DEFERRED on this build: PyChrono's `modal`
module is not compiled here, and a free-vibration transient does not cleanly recover the true frequency,
the only stable integrator (HHT with numerical damping) biases it ~9% low, and low/zero-damping steppers
(Newmark/Trapezoidal) go unstable and segfault on this stiff system. Static superposition keeps turn 3
oracle-exact. Revisit if a modal build becomes available.

## Scoring (tunable, per-task, human-set)

`contract.json -> scoring`: weights L1/L2/L3 = 0.30/0.20/0.50, `invariant_fail_cap` = 40 (any failed
invariant caps the score; L1 failure is 0). Edit to retune; omit to inherit the global default.

## Gate self-check

`conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/beam --turn {1,2,3}` (references -> 100).
Samples: `samples/good_candidate.py` (correct, N=8, different style -> ~100), `samples/bad_candidate.py`
(E=2e11 units slip, runs but 10x too stiff -> capped at 40).
