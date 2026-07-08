"""Independent oracle for the pyb_arm_motor task (stdlib math only, NO Chrono/PyBullet).

A two-link planar arm: link 1 (the crank, uniform rod L1 = 0.4, m = 0.5) is velocity-controlled
about a fixed mount; link 2 (the pendulum, rod L2 = 0.5, m2 = 0.3, d = 0.25,
I_hinge = m2 L2^2/3 = 0.025) hangs FREE from the crank tip. The pendulum is a compound pendulum
with a PRESCRIBED MOVING PIVOT p(t) = (L1 sin(phi), 1.5 - L1 cos(phi)) (both links start
straight down):

    I_h theta'' + m2 d [ x_p'' cos(theta) + (g + z_p'') sin(theta) ] = 0
    x_p'' = L1 (phi'' cos(phi) - phi'^2 sin(phi));  z_p'' = L1 (phi'' sin(phi) + phi'^2 cos(phi))

integrated with RK4 at dt = 2e-5 from rest. The drive uses a 0.5 s linear SOFT-START ramp to
omega (and, in the braked turn, a 0.5 s ramp back down): a step change in a velocity motor is
an impulsive constraint that kicks the free pendulum (a 3.6x amplitude difference was measured
against a naive step-start oracle during calibration), so the ramp profile is part of the task
spec and identical in the PyBullet source, the Chrono reference, and this oracle. The
pendulum's natural rate sqrt(m2 g d / I_h) = 5.4249 rad/s sits above the drive rates (bounded
quasi-periodic response; 3+ rad/s drives tumble it over the top: rejected during design).

  turn 1 (convert): omega = 1.5 rad/s.
  turn 2 (modify):  omega = 2.0 rad/s (stronger centripetal pumping, larger swing).
  turn 3 (extend):  omega = 2.0 ramped back to ZERO at t = 5 s (the setJointMotorControl2
                    retargeting idiom). The pendulum then swings freely: the tail period is
                    the closed-form compound-pendulum 2 pi sqrt(I_h/(m2 g d)) = 1.1582 s and
                    the crank rate in the tail is 0.

Printed values freeze the contract bands; the PyBullet SOURCE script is validated against
these same numbers in a disposable pybullet conda env before any truth is written.
Reproduce: conda run -n chronobench python demo_data_10/pyb_arm_motor/oracle.py
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


def simulate(omega, t_brake=None, t_end=10.0, dt=2e-5):
    prof = drive_profile(omega, t_brake)
    theta = w = 0.0
    tmax = tail = 0.0
    crossings = []

    def acc(th, wth, t):
        phi, phid, phidd = prof(t)
        xpp = L1 * (phidd * math.cos(phi) - phid ** 2 * math.sin(phi))
        zpp = L1 * (phidd * math.sin(phi) + phid ** 2 * math.cos(phi))
        return (-(M2 * D / I_H) * (xpp * math.cos(th) + (G + zpp) * math.sin(th))
                - (B_ELBOW / I_H) * (wth - phid))

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
print(json.dumps(out, indent=2))
