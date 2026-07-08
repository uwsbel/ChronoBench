"""Independent oracle for the cpp_ujoint task (stdlib math only, NO Chrono).

Cardan/Hooke (universal) joint kinematics, the textbook closed form. With the input shaft
driven at constant angular velocity omega1 and a bend angle beta between the shafts, the output
shaft's angular velocity is

    omega2(t) = omega1 * cos(beta) / (1 - sin(beta)^2 * cos(omega1 t)^2),

which oscillates at TWICE the shaft frequency between

    min omega2 = omega1 * cos(beta)      (input cross-arm in the bend plane)
    max omega2 = omega1 / cos(beta)      (a quarter turn later),

with one full omega2 oscillation every pi/omega1 seconds. Both shafts complete the same
rotation per revolution, so the cycle-average of omega2 equals omega1.

  turn 1 (translate): beta = 30 deg, omega1 = 1 rad/s -> min 0.8660, max 1.1547.
  turn 2 (modify):    beta = 45 deg, omega1 = 1 rad/s -> min 0.7071, max 1.4142; an
                      untouched 30-deg translation (0.8660/1.1547) fails BOTH bands.
  turn 3 (extend):    beta = 45 deg, omega1 = 2 rad/s -> min 1.4142, max 2.8284, oscillation
                      period pi/2 = 1.5708 s; a 1 rad/s candidate fails all three.

The source C++ (source/demo_MBS_ujoint.cpp, from projectchrono/chrono src/demos/mbs) drives the
motor with a ChFunctionRamp angle (slope = omega1), gravity off, so the run is deterministic
kinematics: the graded bands are the analytic values +- ~4% (solver + sampling slack).

SIGN convention (verified on the pinned build): the demo initializes the motor as
Initialize(ground, shaft_1, ...), which drives the ramp angle of GROUND relative to the SHAFT,
so both shafts spin in NEGATIVE local z: omega1 = -1, omega2 in [-1/cos b, -cos b]. A faithful
translation preserves the demo's body order and reproduces the negative signed values; the
bands are SIGNED (flipping the direction is a different physical run and fails).

Run offline once; kept in-repo for provenance.
Reproduce: conda run -n chronobench python demo_data_10/cpp_ujoint/oracle.py
"""
import json
import math


def ujoint(beta_rad, omega1):
    return {
        "omega2_min": omega1 * math.cos(beta_rad),
        "omega2_max": omega1 / math.cos(beta_rad),
        "omega2_cycle_mean": omega1,
        "osc_period_s": math.pi / omega1,
    }


# sanity: the closed form's extrema match dense sampling of omega2(t)
def sampled_extrema(beta_rad, omega1, n=200000):
    s2 = math.sin(beta_rad) ** 2
    vals = [omega1 * math.cos(beta_rad) / (1.0 - s2 * math.cos(omega1 * k * 1e-4) ** 2)
            for k in range(n)]
    return min(vals), max(vals)


out = {"sign_note": "demo motor order Initialize(ground, shaft) => signed values are NEGATIVE"}
for name, beta_deg, w1 in (("turn1", 30.0, 1.0), ("turn2", 45.0, 1.0), ("turn3", 45.0, 2.0)):
    b = math.radians(beta_deg)
    o = ujoint(b, w1)
    lo, hi = sampled_extrema(b, w1)
    assert abs(lo - o["omega2_min"]) < 1e-6 and abs(hi - o["omega2_max"]) < 1e-4
    out[name] = {"beta_deg": beta_deg, "omega1": w1,
                 **{k: round(v, 4) for k, v in o.items()},
                 "signed_omega2_min": round(-o["omega2_max"], 4),
                 "signed_omega2_max": round(-o["omega2_min"], 4)}
print(json.dumps(out, indent=2))
