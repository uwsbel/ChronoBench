"""Independent oracle for the mass_spring_damper task (numpy only, NO Chrono).

The ground truth is computed OUTSIDE Chrono, so the benchmark tests "does the candidate match the TRUE
physics", not "does it match our Chrono run" (which would bake in Chrono's conventions and numerical
error). Governing equation: a linear damped oscillator  m*x'' + c*x' + k*x = F(t).

For each turn this provides the L3 target(s) two independent ways that must agree:
  1. closed form (exact for a linear oscillator), and
  2. a high-fidelity RK4 integration at tiny dt, with the SAME observables the judge derives from a
     candidate's CSV (period from upward zero-crossings, zeta from the log-decrement of positive peaks,
     steady-state amplitude from the tail).

Run offline ONCE; kept in-repo for provenance. The printed values are transcribed into contract.json
and CONTRACT.md. Reproduce with:  conda run -n chronobench python demo_data_10/mass_spring_damper/oracle.py
"""
import json
import math

import numpy as np

M, K, X0 = 1.0, 100.0, 0.1          # mass, stiffness, initial displacement from equilibrium
WN = math.sqrt(K / M)               # undamped natural frequency = 10 rad/s
DT, T_END = 1.0e-5, 5.0             # high-fidelity timestep, horizon


def closed_form(c):
    zeta = c / (2.0 * math.sqrt(K * M))
    wd = WN * math.sqrt(1.0 - zeta ** 2)
    return zeta, 2.0 * math.pi / wd     # zeta, damped period Td


def _rk4(c, force, x0=X0, v0=0.0, dt=DT, t_end=T_END):
    def acc(x, v, t):
        return (force(t) - c * v - K * x) / M
    n = int(round(t_end / dt))
    ts = np.empty(n); xs = np.empty(n)
    x, v, t = x0, v0, 0.0
    for i in range(n):
        k1x, k1v = v, acc(x, v, t)
        k2x, k2v = v + 0.5 * dt * k1v, acc(x + 0.5 * dt * k1x, v + 0.5 * dt * k1v, t + 0.5 * dt)
        k3x, k3v = v + 0.5 * dt * k2v, acc(x + 0.5 * dt * k2x, v + 0.5 * dt * k2v, t + 0.5 * dt)
        k4x, k4v = v + dt * k3v, acc(x + dt * k3x, v + dt * k3v, t + dt)
        x += dt * (k1x + 2 * k2x + 2 * k3x + k4x) / 6.0
        v += dt * (k1v + 2 * k2v + 2 * k3v + k4v) / 6.0
        t += dt
        ts[i] = t; xs[i] = x
    return ts, xs


def measured_period(ts, xs):
    # Cross the known equilibrium (0), not the empirical mean: unbiased for a decaying oscillation
    # (matches scoring/judge_v2.py:_period and the way the references self-report).
    cr = [ts[i] for i in range(1, len(xs)) if xs[i - 1] < 0 <= xs[i]]
    return (cr[-1] - cr[0]) / (len(cr) - 1) if len(cr) >= 2 else float("nan")


def measured_zeta(xs):
    pk = [xs[i] for i in range(1, len(xs) - 1) if xs[i] > xs[i - 1] and xs[i] >= xs[i + 1] and xs[i] > 0]
    if len(pk) < 2 or pk[-1] <= 0:
        return float("nan")
    delta = (1.0 / (len(pk) - 1)) * math.log(pk[0] / pk[-1])
    return delta / math.sqrt(4.0 * math.pi ** 2 + delta ** 2)


def steady_amp(ts, xs, t_min):
    tail = np.abs(xs[ts >= t_min])
    return float(tail.max()) if tail.size else float("nan")


out = {}
for c in (2.0, 6.0):
    zeta_cf, Td_cf = closed_form(c)
    ts, xs = _rk4(c, lambda t: 0.0)
    out[f"free_c{c:g}"] = {
        "zeta_closed_form": round(zeta_cf, 6), "Td_closed_form": round(Td_cf, 6),
        "zeta_rk4": round(measured_zeta(xs), 6), "Td_rk4": round(measured_period(ts, xs), 6),
    }

# Turn 3: force the turn-2 system (c=6) at resonance omega = WN; steady-state amplitude.
c3, F0 = 6.0, 1.0
zeta3 = c3 / (2.0 * math.sqrt(K * M))
Xss_cf = F0 / (2.0 * zeta3 * K)                      # closed form at omega = wn
ts, xs = _rk4(c3, lambda t: F0 * math.sin(WN * t))
out["forced_c6_at_wn"] = {
    "Xss_closed_form": round(Xss_cf, 6), "Xss_rk4_tail": round(steady_amp(ts, xs, 3.0), 6),
    "drive_omega": WN, "F0": F0,
}

print(json.dumps(out, indent=2))
