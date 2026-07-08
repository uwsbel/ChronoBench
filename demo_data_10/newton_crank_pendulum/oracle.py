"""Independent oracle for the newton_crank_pendulum task (stdlib math only, NO Chrono/Newton).

The FOURTH leg of the matched set: the identical crank-and-pendulum physics, here sourced from
an imperative Newton program (source/newton_arm_v*.py: newton.ModelBuilder add_link/
add_joint_revolute, a VELOCITY-mode joint actuator whose target_kd is the tracking gain and
whose target is retargeted per step through Control.joint_target_qd, and the joint's native
`damping` parameter as the elbow damper). The same system now exists as: imperative prior-RICH
PyBullet, declarative Isaac/USD, imperative prior-POOR Newton (public for about a year), and
the Chrono references. PyBullet-vs-Newton isolates ECOSYSTEM PRIOR STRENGTH at fixed
representation style (claim C16); PyBullet-vs-USD isolates representation (claim C15).

The Newton sources were EXECUTED on this machine (Warp CPU device; newton 1.3.0, warp 1.15.0)
and match this oracle to better than 1%: v1 tail 0.1999 / max 0.3051 (oracle 0.2015 / 0.3057;
the sub-percent residual is the finite drive gain, kd = 1000, vs the oracle's ideally
prescribed pivot); v2 tail 0.3250 (0.3267); v3 tail 0.0431 (0.0434), braked crank rate 0.001,
ring period 1.149 (1.1785).

Reproduce: conda run -n chronobench python demo_data_10/newton_crank_pendulum/oracle.py
"""
import json
import math

G = 9.81
L1 = 0.4
M2, D, I_H = 0.3, 0.25, 0.025
B_ELBOW = 0.05          # viscous damping at the elbow joint (torque = -b * relative rate)
T_RAMP = 0.5


def drive_profile(omega, t_brake):
    """Return (phi, phidot, phiddot) at time t for the trapezoidal speed profile."""
    def f(t):
        if t < T_RAMP:                                   # soft start
            return omega * t * t / (2 * T_RAMP), omega * t / T_RAMP, omega / T_RAMP
        phi_ramp = omega * T_RAMP / 2
        if t_brake is None or t < t_brake:               # cruise
            return phi_ramp + omega * (t - T_RAMP), omega, 0.0
        if t < t_brake + T_RAMP:                         # soft brake
            u = t - t_brake
            return (phi_ramp + omega * (t_brake - T_RAMP) + omega * u - omega * u * u / (2 * T_RAMP),
                    omega * (1 - u / T_RAMP), -omega / T_RAMP)
        return (phi_ramp + omega * (t_brake - T_RAMP) + omega * T_RAMP / 2, 0.0, 0.0)
    return f


def simulate(omega, t_brake=None, t_end=10.0, dt=2e-5, b=B_ELBOW):
    prof = drive_profile(omega, t_brake)
    theta = w = 0.0
    tmax = tail = 0.0
    crossings = []

    def acc(th, wth, t):
        phi, phid, phidd = prof(t)
        xpp = L1 * (phidd * math.cos(phi) - phid ** 2 * math.sin(phi))
        zpp = L1 * (phidd * math.sin(phi) + phid ** 2 * math.cos(phi))
        return (-(M2 * D / I_H) * (xpp * math.cos(th) + (G + zpp) * math.sin(th))
                - (b / I_H) * (wth - phid))

    prev = theta
    n = int(round(t_end / dt))
    for k in range(n):
        t = k * dt
        k1v = acc(theta, w, t); k1x = w
        k2v = acc(theta + 0.5 * dt * k1x, w + 0.5 * dt * k1v, t + 0.5 * dt); k2x = w + 0.5 * dt * k1v
        k3v = acc(theta + 0.5 * dt * k2x, w + 0.5 * dt * k2v, t + 0.5 * dt); k3x = w + 0.5 * dt * k2v
        k4v = acc(theta + dt * k3x, w + dt * k3v, t + dt); k4x = w + dt * k3v
        theta += dt / 6 * (k1x + 2 * k2x + 2 * k3x + k4x)
        w += dt / 6 * (k1v + 2 * k2v + 2 * k3v + k4v)
        tmax = max(tmax, abs(theta))
        if t >= 7.0:
            tail = max(tail, abs(theta))
        if t >= 6.5 and prev < 0.0 <= theta:
            crossings.append(t)
        prev = theta
    period = (crossings[-1] - crossings[0]) / (len(crossings) - 1) if len(crossings) > 1 else None
    return tmax, tail, period


out = {"free_period_closed_form": round(2 * math.pi * math.sqrt(I_H / (M2 * G * D)), 4)}
for name, omega, brake in (("turn1", 1.5, None), ("turn2", 2.0, None), ("turn3", 2.0, 5.0)):
    tmax, tail, period = simulate(omega, brake)
    out[name] = {"omega": omega, "brake_at": brake, "theta_max": round(tmax, 4),
                 "theta_tail": round(tail, 4),
                 "tail_period": round(period, 4) if period else None}
tmax0, tail0, _ = simulate(1.5, None, b=0.0)
out["bad_control_undamped"] = {"omega": 1.5, "b": 0.0, "theta_max": round(tmax0, 4),
                               "theta_tail": round(tail0, 4)}
print(json.dumps(out, indent=2))
