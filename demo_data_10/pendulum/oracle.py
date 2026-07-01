"""Independent oracle for the pendulum task (stdlib math only, NO Chrono).

Ground truth computed OUTSIDE Chrono so the benchmark tests "matches the true physics", not "matches our
Chrono run".
  Turn 1 (small angle): T = 2*pi*sqrt(L/g)  (linearized closed form).
  Turn 2 (large angle theta0): exact nonlinear period T = 4*sqrt(L/g)*K(sin(theta0/2)), where K is the
    complete elliptic integral of the first kind (computed via the arithmetic-geometric mean), cross-checked
    by a high-fidelity RK4 of theta'' = -(g/L)*sin(theta). The two must agree.
  Turn 3 (double pendulum): total mechanical energy conservation is a physical LAW (relative drift -> 0 in
    the continuum limit), so the invariant is a small-drift BOUND, not a specific oracle value; the second
    arm's angular range is chaotic (a minimum-swing bound, not a precise target).

Run offline once; kept in-repo for provenance.
Reproduce: conda run -n chronobench python demo_data_10/pendulum/oracle.py
"""
import json
import math

L, g = 1.0, 9.81


def ellipk(k):
    """Complete elliptic integral of the first kind K(k) via the arithmetic-geometric mean."""
    a, b = 1.0, math.sqrt(1.0 - k * k)
    for _ in range(60):
        a, b = 0.5 * (a + b), math.sqrt(a * b)
    return math.pi / (2.0 * a)


def rk4_period(theta0, dt=1.0e-5, t_end=11.0):
    """Nonlinear pendulum period from RK4 of theta'' = -(g/L) sin(theta), released from rest at theta0."""
    w2 = g / L
    th, om, t = theta0, 0.0, 0.0
    ts, ths = [], []
    for _ in range(int(t_end / dt)):
        k1t, k1o = om, -w2 * math.sin(th)
        k2t, k2o = om + 0.5 * dt * k1o, -w2 * math.sin(th + 0.5 * dt * k1t)
        k3t, k3o = om + 0.5 * dt * k2o, -w2 * math.sin(th + 0.5 * dt * k2t)
        k4t, k4o = om + dt * k3o, -w2 * math.sin(th + dt * k3t)
        th += dt * (k1t + 2 * k2t + 2 * k3t + k4t) / 6.0
        om += dt * (k1o + 2 * k2o + 2 * k3o + k4o) / 6.0
        t += dt
        ts.append(t); ths.append(th)
    cr = [ts[i] for i in range(1, len(ths)) if ths[i - 1] < 0.0 <= ths[i]]
    return (cr[-1] - cr[0]) / (len(cr) - 1) if len(cr) >= 2 else float("nan")


theta0 = math.radians(60.0)
out = {
    "turn1_small_angle_T_closed_form": round(2.0 * math.pi * math.sqrt(L / g), 6),
    "turn2_large_angle_T_closed_form": round(4.0 * math.sqrt(L / g) * ellipk(math.sin(theta0 / 2.0)), 6),
    "turn2_large_angle_T_rk4": round(rk4_period(theta0), 6),
    "turn2_theta_max": round(theta0, 6),
    "turn3_note": "energy conservation is a law: invariant is drift<=bound; theta2_range is chaotic (min-swing bound)",
}
print(json.dumps(out, indent=2))
