"""Independent oracle for the import_urdf task (stdlib math only, NO Chrono).

The task ships `assets/pendulum.urdf` (a single-link compound pendulum; the URDF text is the
parameter-first source of truth: rod mass m = 2.0 kg, COM distance d = 0.5 m below the revolute
joint "swing" about +Z, inertia about the COM Izz = m*L^2/12 = 0.1666667 for the L = 1.0 m rod).

  Turns 1-2 (imposed kinematics): the joint is POSITION-actuated with theta(t) = A sin(2 pi f t).
  The observables are then exact closed forms of the actuation parameters, independent of the
  dynamics: amplitude = A, period = 1/f.
      turn 1: A = 0.5 rad, f = 1.0 Hz  -> amplitude 0.5, period 1.0 s
      turn 2: A = 0.8 rad, f = 0.5 Hz  -> amplitude 0.8, period 2.0 s

  Turn 3 (free swing, tests that the parser truly imports the URDF's physical parameters):
  released from rest at theta0 = 0.2 rad, the compound pendulum has
      I_pivot = Izz + m d^2 = 0.1666667 + 2.0*0.25 = 0.6666667 kg m^2
      T_small = 2 pi sqrt(I_pivot / (m g d)) = 2 pi sqrt(0.6666667 / 9.81) = 1.637947 s
  with the finite-amplitude correction T(theta0) ~= T_small * (1 + theta0^2/16) = 1.642042 s.

Run offline once; kept in-repo for provenance.
Reproduce: conda run -n chronobench python demo_data_10/import_urdf/oracle.py
"""
import json
import math

m, L, d, g = 2.0, 1.0, 0.5, 9.81
Izz_com = m * L * L / 12.0
I_pivot = Izz_com + m * d * d
T_small = 2.0 * math.pi * math.sqrt(I_pivot / (m * g * d))
theta0 = 0.2

out = {
    "urdf_params": {"m": m, "L": L, "d": d, "Izz_com": round(Izz_com, 7), "I_pivot": round(I_pivot, 7)},
    "turn1_imposed": {"A": 0.5, "f": 1.0, "amplitude": 0.5, "period": 1.0},
    "turn2_imposed": {"A": 0.8, "f": 0.5, "amplitude": 0.8, "period": 2.0},
    "turn3_free_swing": {"theta0": theta0, "T_small": round(T_small, 6),
                         "T_corrected": round(T_small * (1.0 + theta0 ** 2 / 16.0), 6)},
}
print(json.dumps(out, indent=2))
