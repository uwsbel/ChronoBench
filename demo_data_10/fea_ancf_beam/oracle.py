"""Independent oracle for the fea_ancf_beam task (numpy only, NO Chrono).

Ground truth computed OUTSIDE Chrono: the same independent Euler-Bernoulli cantilever finite-element
solver used for the `beam` task (Hermite cubic elements; DOF = transverse deflection w and slope),
adapted to a CIRCULAR section (the ANCF cable element's section), plus the exact textbook closed
forms. A slender circular cantilever in the small-deflection regime is Euler-Bernoulli; the ANCF
gradient-deficient cable element must converge to the same statics, which is precisely what the task
grades (same physics, different element technology than the `beam` task).

  Turn 1 (create):  tip point load F.                delta = F*L^3/(3*E*I).
  Turn 2 (modify):  self-weight (q = rho*A*g).       delta = q*L^4/(8*E*I).
  Turn 3 (extend):  tip F PLUS equal P at mid-span.  delta = F*L^3/(3*E*I) + 5*P*L^3/(48*E*I).

Section: circular, diameter d.  A = pi d^2/4,  I = pi d^4/64.

Run offline once; kept in-repo for provenance.
Reproduce: conda run -n chronobench python demo_data_10/fea_ancf_beam/oracle.py
"""
import json
import math

import numpy as np

L = 1.0
d = 0.02                      # circular section diameter
E = 2.0e10
rho = 7800.0
F = 2.0                       # turn-1 tip load; turn-3 also uses P = F at mid-span
g = 9.81
A = math.pi * d ** 2 / 4.0
I = math.pi * d ** 4 / 64.0
EI = E * I
q_self = rho * A * g


def _assemble_K(n_el):
    le = L / n_el
    ndof = 2 * (n_el + 1)
    K = np.zeros((ndof, ndof))
    ke = (EI / le ** 3) * np.array([
        [12, 6 * le, -12, 6 * le],
        [6 * le, 4 * le ** 2, -6 * le, 2 * le ** 2],
        [-12, -6 * le, 12, -6 * le],
        [6 * le, 2 * le ** 2, -6 * le, 4 * le ** 2]])
    for e in range(n_el):
        dofs = [2 * e, 2 * e + 1, 2 * e + 2, 2 * e + 3]
        K[np.ix_(dofs, dofs)] += ke
    return K, le


def _solve(n_el, f):
    """Solve K u = f with node 0 clamped (drop DOF 0,1); return the full DOF vector."""
    K, _ = _assemble_K(n_el)
    u = np.zeros(K.shape[0])
    u[2:] = np.linalg.solve(K[2:, 2:], f[2:])
    return u


def tip_point_load(n_el):
    ndof = 2 * (n_el + 1)
    f = np.zeros(ndof)
    f[ndof - 2] = -F
    return abs(_solve(n_el, f)[ndof - 2])


def tip_self_weight(n_el):
    K, le = _assemble_K(n_el)
    ndof = K.shape[0]
    f = np.zeros(ndof)
    fe = -q_self * np.array([le / 2.0, le ** 2 / 12.0, le / 2.0, -le ** 2 / 12.0])
    for e in range(n_el):
        dofs = [2 * e, 2 * e + 1, 2 * e + 2, 2 * e + 3]
        f[dofs] += fe
    return abs(_solve(n_el, f)[ndof - 2])


def tip_two_loads(n_el):
    """Tip load F plus an equal P=F at the mid-span node (x=L/2); needs even n_el."""
    ndof = 2 * (n_el + 1)
    f = np.zeros(ndof)
    f[ndof - 2] = -F
    f[2 * (n_el // 2)] = -F
    return abs(_solve(n_el, f)[ndof - 2])


out = {
    "params": {"L": L, "d": d, "E": E, "rho": rho, "A": round(A, 10), "I": round(I, 14),
               "EI": round(EI, 6), "q_self": round(q_self, 6)},
    "turn1_tip_point_load": {
        "delta_closed_form": F * L ** 3 / (3.0 * EI),
        "delta_fe_N10": round(tip_point_load(10), 10),
        "delta_fe_N50": round(tip_point_load(50), 10),
    },
    "turn2_self_weight": {
        "delta_closed_form": q_self * L ** 4 / (8.0 * EI),
        "delta_fe_N10": round(tip_self_weight(10), 10),
        "delta_fe_N50": round(tip_self_weight(50), 10),
    },
    "turn3_tip_plus_midspan": {
        "delta_closed_form": F * L ** 3 / (3.0 * EI) + 5.0 * F * L ** 3 / (48.0 * EI),
        "delta_fe_N10": round(tip_two_loads(10), 10),
        "delta_fe_N50": round(tip_two_loads(50), 10),
    },
}
print(json.dumps(out, indent=2, default=lambda x: round(float(x), 10)))
