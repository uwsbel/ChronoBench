"""Independent oracle for the swig_contact_reporter task (stdlib math only, NO Chrono).

Ground truth is static equilibrium (Newton, not Chrono): N identical spheres of mass m resting on a flat
rigid ground each make ONE ground contact that carries their weight. Therefore
    n_contacts = N,   total normal force = N*m*g,   per-contact normal force = m*g.
The sphere mass follows from the geometry + density used in the reference (ChBodyEasySphere mass =
density * (4/3)*pi*r^3); the references pick density so m = 1.0 kg (turns 1-2) or m = 2.0 kg (turn 3).

Turns: 1 (N=4, m=1) / 2 (N=6, m=1) / 3 (N=6, m=2).

Run offline once; kept in-repo for provenance.
Reproduce: conda run -n chronobench python demo_data_10/swig_contact_reporter/oracle.py
"""
import json
import math

r = 0.1
g = 9.81
vol = (4.0 / 3.0) * math.pi * r ** 3


def case(N, m):
    return {"N": N, "m": round(m, 4), "n_contacts": N,
            "total_force": round(N * m * g, 4), "per_contact_force": round(m * g, 4)}


out = {
    "sphere_volume": round(vol, 8),
    "density_for_m1": round(1.0 / vol, 4),
    "density_for_m2": round(2.0 / vol, 4),
    "turn1": case(4, 1.0),
    "turn2": case(6, 1.0),
    "turn3": case(6, 2.0),
}
print(json.dumps(out, indent=2))
