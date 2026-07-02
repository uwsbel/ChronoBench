# Task contract: plate_sinkage_scm (3-turn, oracle-grounded, terramechanics)

The first REALISTIC task, a bevameter / plate-sinkage test on SCM (Soil Contact Model) deformable terrain,
sourced from the SBEL terramechanics line of work (`sbel-reproducibility`). It is the template's first
outing on a domain where NO tight closed-form oracle exists, so it grades with a coarse independent band
plus a cross-turn qualitative law.

- **Axis:** vehicle dynamics & terrain (terramechanics / deformable soil).
- **Simulator:** PyChrono 10.0 (`pychrono.vehicle.SCMTerrain`, CPU, headless).
- **System:** a rigid flat plate rests on a Bekker-Wong SCM patch under a vertical load and sinks to
  equilibrium. Baseline: plate 0.2 x 0.2 x 0.05 m, load 500 N, soil Kphi=2e6 (Kc=0, n=1).

## Turns

1. **create** (`truth1.py`): plate on baseline soil under 500 N.
2. **modify** (`truth2.py`): SOFTER soil (Kphi 2e6 -> 5e5, 4x softer) -> deeper.
3. **extend** (`truth3.py`): HEAVIER load (500 -> 2000 N) -> deeper. `pyinput2.py`=truth1, `pyinput3.py`=truth2.

## Ground truth = INDEPENDENT oracle (coarse band, NOT a Chrono run)

`oracle.py` (stdlib, NO Chrono) uses the Bekker-Wong pressure-sinkage relation z = (p/(Kc/b + Kphi))^(1/n),
p = F/A. Honest limitation: Chrono's SCM adds elastic + dynamic + discretization effects, so the ABSOLUTE
sinkage is measured at ~1.0-1.5x the ideal Bekker value (super-linear, slope ~1.15), NOT tightly equal.
So this task grades on a COARSE band [0.5x, 2.5x] Bekker (independent order of magnitude), and encodes the
"softer soil / heavier load -> deeper" law ACROSS turns via the shifting band (an un-softened / un-loaded
candidate lands ~0.0077 m, below the turn-2/3 lower bound). Two-way check (Chrono vs oracle):

| Turn | Change | Bekker z (m) | band [0.5x, 2.5x] | Chrono `truth` (m) |
|------|--------|--------------|-------------------|--------------------|
| 1 | baseline | 0.00625 | [0.00313, 0.01563] | 0.00765 (1.22x) |
| 2 | Kphi/4 (softer) | 0.025 | [0.0125, 0.0625] | 0.0299 (1.19x) |
| 3 | 4x load | 0.025 | [0.0125, 0.0625] | 0.0358 (1.43x) |

## Judge-derived observable

The reference logs `t,sinkage` (plate drop below its initial position). The judge derives `sinkage_final`
(`final` of the column) and checks it against the coarse band. A model must run real SCM and produce a
plate that sinks by the right order of magnitude; it cannot self-report a number.

Note: within-run monotonicity is NOT used, the settled plate jitters at equilibrium under SMC contact
(the sinkage-vs-time monotonic fraction is ~0.5), so it is not a valid instrument here. The min-bound
("actually sank") + max-bound ("didn't blow up / fall through") + cross-turn band shifts do the work.

## Parameter-first (contamination-ready)

Each turn's `params` block in `contract.json` is the single declared source of the soil/plate/load values;
`oracle.py`, the prompts, and the references all trace to it, and `bekker_sinkage_m` is a pure function of
them. Eval-time randomization = pick new params, recompute the Bekker band, re-render the prompt.

## Scoring (tunable): weights 0.30/0.20/0.50, `invariant_fail_cap` = 40.

## Gate self-check

`conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/plate_sinkage_scm --turn {1,2,3}` (or
`python -m chronobench.score_v2 --task plate_sinkage_scm`). References pass; `samples/good_candidate.py`
(correct, different style) passes; `samples/bad_candidate.py` (Kphi 100x too stiff -> ~0.08 mm sinkage,
below band) is capped at 40.
