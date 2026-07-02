"""Independent oracle for the beam task (numpy only, NO Chrono).

Ground truth computed OUTSIDE Chrono, an independent Euler-Bernoulli cantilever finite-element solver
(Hermite cubic beam elements: 2 nodes, DOF = transverse deflection w and slope theta), plus the exact
textbook closed forms. The FE solver is refined (N elements) to show convergence and cross-check the
closed forms; this is the "high-fidelity independent reference" the targets come from.

  Turn 1 (create):  cantilever, transverse point load F at the tip.     delta = F*L^3/(3*E*I).
  Turn 2 (modify):  cantilever under its own weight (uniform load q=rho*A*g).  delta = q*L^4/(8*E*I).
  Turn 3 (extend):  turn-1 tip load PLUS an equal point load P at mid-span (x=L/2); tip deflection by
                    superposition = F*L^3/(3*E*I) + 5*P*L^3/(48*E*I).

(A first-natural-frequency turn was considered but deferred: PyChrono's modal module is not built here,
and a free-vibration transient is timestepper-biased, the only stable integrator, HHT with numerical
damping, shifts the apparent frequency ~9% and low-damping steppers go unstable. Static superposition
keeps turn 3 oracle-exact and low-risk.)

Run offline once; kept in-repo for provenance.
Reproduce: conda run -n chronobench python demo_data_10/beam/oracle.py
"""
import json

import numpy as np

L = 1.0
b = 0.05                      # square section b x b
E = 2.0e10
rho = 7800.0
F = 150.0                     # turn-1 tip load; turn-3 also uses P = F at mid-span
g = 9.81                      # turn-2 self-weight
A = b * b
I = b ** 4 / 12.0
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
        d = [2 * e, 2 * e + 1, 2 * e + 2, 2 * e + 3]
        K[np.ix_(d, d)] += ke
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
    f[ndof - 2] = -F                          # transverse load at tip node's w DOF
    return abs(_solve(n_el, f)[ndof - 2])


def tip_self_weight(n_el):
    K, le = _assemble_K(n_el)
    ndof = K.shape[0]
    f = np.zeros(ndof)
    fe = -q_self * np.array([le / 2.0, le ** 2 / 12.0, le / 2.0, -le ** 2 / 12.0])   # consistent uniform load
    for e in range(n_el):
        d = [2 * e, 2 * e + 1, 2 * e + 2, 2 * e + 3]
        f[d] += fe
    return abs(_solve(n_el, f)[ndof - 2])


def tip_two_loads(n_el):
    """Tip load F plus an equal load P=F at mid-span node (x=L/2); requires an even n_el for an exact mid node."""
    ndof = 2 * (n_el + 1)
    f = np.zeros(ndof)
    f[ndof - 2] = -F                          # tip
    f[2 * (n_el // 2)] = -F                    # mid-span node w DOF
    return abs(_solve(n_el, f)[ndof - 2])


beta1 = 1.8751040687119611
out = {
    "params": {"L": L, "b": b, "E": E, "rho": rho, "I": I, "A": A, "EI": EI, "q_self": round(q_self, 4)},
    "turn1_tip_point_load": {
        "delta_closed_form": F * L ** 3 / (3.0 * EI),
        "delta_fe_N10": round(tip_point_load(10), 8),
        "delta_fe_N50": round(tip_point_load(50), 8),
    },
    "turn2_self_weight": {
        "delta_closed_form": q_self * L ** 4 / (8.0 * EI),
        "delta_fe_N10": round(tip_self_weight(10), 8),
        "delta_fe_N50": round(tip_self_weight(50), 8),
    },
    "turn3_tip_plus_midspan": {
        "delta_closed_form": F * L ** 3 / (3.0 * EI) + 5.0 * F * L ** 3 / (48.0 * EI),
        "delta_fe_N10": round(tip_two_loads(10), 8),
        "delta_fe_N50": round(tip_two_loads(50), 8),
    },
}
print(json.dumps(out, indent=2, default=lambda x: round(float(x), 8)))
