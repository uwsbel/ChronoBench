"""Independent oracle for the solver_nsc_smc task (stdlib math only, NO Chrono).

Ideal bouncing-ball impact kinematics, pure functions of the declared params. A rigid ball whose
bottom starts at height h0 above a rigid floor, released from rest under gravity g, with
coefficient of restitution e:
  - impact speed        v0 = sqrt(2 g h0),    at t_impact = sqrt(2 h0 / g)
  - rebound speed       v1 = e * v0
  - first-bounce apex   apex1 = e^2 * h0,     at t_apex = t_impact * (1 + e)
Subsequent apexes decay by e^2 each bounce, so the maximum of the ball-bottom height over any
window that starts after the first impact IS apex1. These are exact closed-form identities;
no integration is involved.

Targets (h0 = 1.0 m, g = 9.81):
  turn 1 (create):  NSC, e = 0.7  -> apex1 = 0.49 m   (t_impact 0.4515 s, t_apex 0.7676 s)
  turn 2 (modify):  NSC, e = 0.9  -> apex1 = 0.81 m   (t_apex 0.8579 s)
  turn 3 (extend):  SMC, e = 0.9  -> apex1 = 0.81 m   (same physics, compliant contact method)

Cross-turn law encoded: higher e must rebound higher (0.49 -> 0.81); switching the contact
formulation (NSC complementarity -> SMC penalty) must NOT change the physics.

Calibration of the Chrono references against this oracle (measured on the pinned pychrono10,
dt = 2.0e-4, default solver settings): NSC e=0.7 -> 0.4902 (+0.03%), NSC e=0.9 -> 0.8101 (+0.01%),
SMC e=0.9 -> 0.8031 (-0.85%). AUTHORING FINDING: NSC restitution has an isolated bad time-step
pocket, at dt = 1.0e-4 the same NSC e=0.7 drop rebounds to only 0.266 (54% of ideal; dt = 5e-5
gives 95%, while 2e-5, 2e-4, 5e-4, and 1e-3 are all within 0.2%). The task therefore SPECIFIES
dt = 2.0e-4, and the graded band (rel_tol 0.12) forgives every probed step except the 1e-4
pocket. Recorded in docs/DELTAS_10.md.

Run offline once; kept in-repo for provenance.
Reproduce: conda run -n chronobench python demo_data_10/solver_nsc_smc/oracle.py
"""
import json
import math

g = 9.81
h0 = 1.0


def case(e):
    t_imp = math.sqrt(2.0 * h0 / g)
    return {"e": e, "h0": h0,
            "impact_speed": round(math.sqrt(2.0 * g * h0), 6),
            "t_impact": round(t_imp, 6),
            "apex1": e * e * h0,
            "t_apex1": round(t_imp * (1.0 + e), 6)}


out = {"turn1_nsc": case(0.7), "turn2_nsc": case(0.9), "turn3_smc": case(0.9)}
print(json.dumps(out, indent=2))
