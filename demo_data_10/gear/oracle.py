"""Independent oracle for the gear task (stdlib math only, NO Chrono).

Ideal rigid transmission kinematics for a fixed-axis train; pure functions of the declared params.
  - External gear mesh (spur pair, parallel axes): the pitch-line velocities match, so
        omega_2 = -(r1/r2) * omega_1
    (counter-rotation, hence the minus sign; r = pitch radius).
  - Two-stage compound of external meshes: each stage reverses the sense, so two stages restore
    it, and the compound magnitude ratio telescopes:
        omega_3 = -(r2/r3) * omega_2 = +(r1/r3) * omega_1.
These are exact kinematic identities for ideal (non-slipping, rigid) transmissions; no integration
is involved, so the oracle is closed form.

Targets:
  turn 1 (create):  r1=0.2, r2=0.4, omega_in=4.0        -> w2 = -(0.2/0.4)*4.0 = -2.0 rad/s
  turn 2 (modify):  r1=0.2, r2=0.6, omega_in=4.0        -> w2 = -(0.2/0.6)*4.0 = -4/3 rad/s
  turn 3 (extend):  turn-2 train + gear C, r3=0.2       -> w2 = -4/3 rad/s (unchanged) and
                                                           w3 = -(0.6/0.2)*(-4/3) = +4.0 rad/s

Cross-turn law encoded: a larger driven gear (turn 2) slows the output; the second stage (turn 3)
reverses the sense back to positive and, since r1/r3 = 1, restores |w3| = |omega_in| exactly. The
SIGN of w3 is the discriminator for getting the mesh count right.

(Authoring note: the original turn-3 design used a belt stage via ChLinkLockPulley, but measured
behavior of that link with demo-style shaft frames enforces omega_out/omega_in = tau + 2 rather
than the textbook tau = rp1/rp2, independent of shaft distance; a candidate implementing correct
belt physics would be graded wrong, so the belt was replaced by a second gear mesh. Recorded in
docs/DELTAS_10.md.)

Run offline once; kept in-repo for provenance.
Reproduce: conda run -n chronobench python demo_data_10/gear/oracle.py
"""
import json


def external_mesh(r_in, r_out, w_in):
    return -(r_in / r_out) * w_in


cases = {}
w1 = 4.0
cases["turn1"] = {"r1": 0.2, "r2": 0.4, "omega_in": w1,
                  "w2": external_mesh(0.2, 0.4, w1)}
cases["turn2"] = {"r1": 0.2, "r2": 0.6, "omega_in": w1,
                  "w2": external_mesh(0.2, 0.6, w1)}
w2_t3 = external_mesh(0.2, 0.6, w1)
cases["turn3"] = {"r1": 0.2, "r2": 0.6, "r3": 0.2, "omega_in": w1,
                  "w2": w2_t3, "w3": external_mesh(0.6, 0.2, w2_t3)}
print(json.dumps(cases, indent=2))
