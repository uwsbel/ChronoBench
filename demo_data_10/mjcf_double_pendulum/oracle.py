"""Independent oracle for the mjcf_double_pendulum task (stdlib math only, NO Chrono/MuJoCo).

Double COMPOUND pendulum: two uniform rods on hinges (rod 1 from a fixed pivot, rod 2 from the
tip of rod 1), the exact Lagrangian equations integrated with RK4 at dt = 1e-5. With d_i = L_i/2
and I_i = m_i L_i^2/12 about the COMs:

    M11 = I1 + m1 d1^2 + m2 L1^2          M12 = m2 L1 d2 cos(t1 - t2)
    M22 = I2 + m2 d2^2
    rhs1 = -m2 L1 d2 sin(t1-t2) w2^2 - (m1 d1 + m2 L1) g sin(t1)
    rhs2 = +m2 L1 d2 sin(t1-t2) w1^2 - m2 g d2 sin(t2)

Small-angle releases keep the motion quasi-periodic (two linear normal modes; no chaos), so the
graded observables are window maxima over the 10 s run: the first arm's amplitude envelope and
the second arm's peak swing (energy exchange pumps arm 2 well above arm 1's release angle).
Total energy is conserved exactly in the model; the candidate logs Chrono's energy and the
judge checks its drift.

  turn 1 (convert): rods m = 1.0, L = 1.0 each; release t1 = 0.1, t2 = 0 at rest.
  turn 2 (modify):  rod 2 grows to L2 = 1.5, m2 = 1.5 (same uniform linear density);
                    the mode structure shifts and both maxima change.
  turn 3 (extend):  same long rod 2 as turn 2, released BENT: t1 = 0.1, t2 = -0.1; the
                    opposite initial bend excites the fast mode strongly.

Printed values are the RK4 window maxima used to freeze the contract bands (verify Chrono
agrees at the percent level before trusting them). Run offline once; kept in-repo.
Reproduce: conda run -n chronobench python demo_data_10/mjcf_double_pendulum/oracle.py
"""
import json
import math

G = 9.81


def simulate(m1, L1, m2, L2, t1_0, t2_0, t_end=10.0, dt=1e-5):
    d1, d2 = L1 / 2, L2 / 2
    I1, I2 = m1 * L1 ** 2 / 12, m2 * L2 ** 2 / 12

    def accel(t1, t2, w1, w2):
        c, s = math.cos(t1 - t2), math.sin(t1 - t2)
        M11 = I1 + m1 * d1 ** 2 + m2 * L1 ** 2
        M12 = m2 * L1 * d2 * c
        M22 = I2 + m2 * d2 ** 2
        r1 = -m2 * L1 * d2 * s * w2 ** 2 - (m1 * d1 + m2 * L1) * G * math.sin(t1)
        r2 = m2 * L1 * d2 * s * w1 ** 2 - m2 * G * d2 * math.sin(t2)
        det = M11 * M22 - M12 * M12
        return (M22 * r1 - M12 * r2) / det, (M11 * r2 - M12 * r1) / det

    def deriv(y):
        a1, a2 = accel(y[0], y[1], y[2], y[3])
        return [y[2], y[3], a1, a2]

    y = [t1_0, t2_0, 0.0, 0.0]
    n = int(round(t_end / dt))
    amp1 = amp2 = rel = 0.0
    for _ in range(n):
        k1 = deriv(y)
        k2 = deriv([y[i] + 0.5 * dt * k1[i] for i in range(4)])
        k3 = deriv([y[i] + 0.5 * dt * k2[i] for i in range(4)])
        k4 = deriv([y[i] + dt * k3[i] for i in range(4)])
        y = [y[i] + dt / 6 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) for i in range(4)]
        amp1 = max(amp1, abs(y[0]))
        amp2 = max(amp2, abs(y[1]))
        rel = max(rel, abs(y[1] - y[0]))
    return amp1, amp2, rel


out = {}
for name, m1, L1, m2, L2, t1_0, t2_0 in (
        ("turn1", 1.0, 1.0, 1.0, 1.0, 0.1, 0.0),
        ("turn2", 1.0, 1.0, 1.5, 1.5, 0.1, 0.0),
        ("turn3", 1.0, 1.0, 1.5, 1.5, 0.1, -0.1)):
    a1, a2, rel = simulate(m1, L1, m2, L2, t1_0, t2_0)
    out[name] = {"m2": m2, "L2": L2, "release": [t1_0, t2_0],
                 "theta1_max": round(a1, 4), "theta2_max": round(a2, 4),
                 "rel_max": round(rel, 4)}
print(json.dumps(out, indent=2))
