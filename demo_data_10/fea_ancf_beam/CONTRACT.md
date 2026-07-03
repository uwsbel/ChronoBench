# fea_ancf_beam -- task contract (v2.0, PyChrono 10.0)

FEA / ANCF axis: the ANCF gradient-deficient cable element (`ChElementCableANCF`), circular-section
setup on `ChBeamSectionCable`, node clamping (position + slope), point and self-weight loading, and
the NONLINEAR static solve, graded from the candidate's own logged deflected shape. Complements the
`beam` task (same physics discipline, different element technology: Euler elements + linear solve
there, ANCF + nonlinear solve here).

## Independent oracle (anti-circularity)

`oracle.py` (numpy, NO Chrono): the same independent Euler-Bernoulli Hermite-FE cantilever solver
used for `beam`, adapted to the circular section (A = pi d^2/4, I = pi d^4/64), cross-checked
against the textbook closed forms. In the small-deflection regime the ANCF cable statics must
converge to Euler-Bernoulli, and it does. Two-way validation (Chrono reference vs oracle): turn 1
agrees to 0.002%, turn 2 to 0.03%, turn 3 to 0.003%.

## Turns (L=1 m, d=0.02 m, E=2e10 Pa, rho=7800; deflections kept well inside the linear regime)

1. Create: tip point load F=2 N.       delta = F*L^3/(3*E*I)                    = 4.2441e-3 m.
2. Modify: self-weight only (g=9.81).  delta = q*L^4/(8*E*I), q=rho*A*g         = 1.9129e-2 m
   (catches "tip load left in": 4.2e-3 fails; probes automatic mesh gravity).
3. Extend: tip F plus mid-span P=2 N.  delta = F*L^3/(3EI) + 5*P*L^3/(48EI)     = 5.5704e-3 m
   (catches "mid load at wrong station" and "tip-only").

## Gate (de-scoped judge)

1. L1: runs headless on pinned pychrono10; emits `out.csv` (`x,y` per node) + one JSON line.
2. L2 (minimal): `ChMesh`; `ChElementCableANCF|ChBuilderCableANCF`; `ChBeamSectionCable` (turns
   1/3) or gravity caps (turn 2); `DoStaticNonlinear|DoStatic`.
3. L3 (measured): tip deflection derived as max|y| of the logged shape vs the oracle value,
   rel_tol 0.06 (roomy for discretization choices; the 10x-stiff E slip fails at 90%).

## Authoring notes

1. `DoStaticNonlinear(100)` + PardisoMKL converged cleanly for all turns; none of the planned
   fallbacks (incremental static analysis, damped dynamic settling) were needed.
2. `ChNodeFEAxyzD.SetFixed(True)` clamps position AND slope, exactly a cantilever root clamp; a
   hinge (`ChLinkNodeFrame` alone) would NOT be a clamp.
3. The good control candidate builds nodes/elements manually (no builder) with N=30 and the MINRES
   iterative solver, and lands on the same deflection; the gate is style-insensitive.

## Verify

```
conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/fea_ancf_beam --turn N            # 100
conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/fea_ancf_beam --turn 1 demo_data_10/fea_ancf_beam/samples/good_candidate.py   # 100
conda run -n pychrono10 python scoring/judge_v2.py demo_data_10/fea_ancf_beam --turn 1 demo_data_10/fea_ancf_beam/samples/bad_candidate.py    # 40 (invariant-fail)
```
