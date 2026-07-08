"""Independent oracle for the mjcf_robot_arm task (stdlib math only, NO Chrono/MuJoCo).

A 3-DOF arm (base yaw about z, shoulder pitch about y, elbow pitch about y) driving a stated
joint-space trajectory; under imposed joint motion the end effector follows pure FORWARD
KINEMATICS, closed form with pitch angles measured from vertical (+z):

    z = H + L2 cos(q2) + L3 cos(q2 + q3)
    r = L2 sin(q2) + L3 sin(q2 + q3)
    x = r cos(q1),   y = r sin(q1)

with column height H = 0.4 and the trajectories (all starting at zero, so the arm starts
straight up and the motors impose the angles directly):

    q1(t) = 0.5 sin(0.2 pi t)                (yaw sweep, one cycle in 10 s)
    q2(t) = 0.35 (1 - cos(w t))              (shoulder reaches down and back)
    q3(t) = -0.5 (1 - cos(w t))              (elbow folds and unfolds)

  turn 1 (convert): L2 = L3 = 0.5, w = 0.4 pi. Deepest reach z_min at (q2, q3) = (0.7, -1.0);
                    the trajectory returns HOME at t = 10 (all q = 0): z_final = 1.4, r ~ 0.
  turn 2 (modify):  the MJCF's upper arm grows: L2 = 0.7. Everything rescales per FK;
                    home is now z = 1.6.
  turn 3 (extend):  L2 = 0.7 and the trajectory is RETIMED 2x (w and the yaw rate double;
                    two full cycles in 10 s, still ending home). The geometry-driven extremes
                    are unchanged; the peak vertical EE rate |dz/dt| doubles.

Printed values (numeric sweep at dt = 1e-4) freeze the contract bands.
Reproduce: conda run -n chronobench python demo_data_10/mjcf_robot_arm/oracle.py
"""
import json
import math

H = 0.4


def sweep(L2, L3, rate_scale, t_end=10.0, dt=1e-4):
    zmin, ymax, dzmax = 10.0, 0.0, 0.0
    zprev = None
    n = int(round(t_end / dt))
    for k in range(n + 1):
        t = k * dt
        q1 = 0.5 * math.sin(0.2 * math.pi * rate_scale * t)
        q2 = 0.35 * (1 - math.cos(0.4 * math.pi * rate_scale * t))
        q3 = -0.5 * (1 - math.cos(0.4 * math.pi * rate_scale * t))
        z = H + L2 * math.cos(q2) + L3 * math.cos(q2 + q3)
        r = L2 * math.sin(q2) + L3 * math.sin(q2 + q3)
        y = r * math.sin(q1)
        zmin = min(zmin, z)
        ymax = max(ymax, abs(y))
        if zprev is not None:
            dzmax = max(dzmax, abs(z - zprev) / dt)
        zprev = z
    # final sample is t_end exactly
    return zmin, ymax, dzmax, z, abs(r)


out = {}
for name, L2, scale in (("turn1", 0.5, 1.0), ("turn2", 0.7, 1.0), ("turn3", 0.7, 2.0)):
    zmin, ymax, dzmax, zf, rf = sweep(L2, 0.5, scale)
    out[name] = {"L2": L2, "rate_scale": scale,
                 "z_min": round(zmin, 4), "y_maxabs": round(ymax, 4),
                 "dz_rate_max": round(dzmax, 4),
                 "z_final": round(zf, 4), "r_final": round(rf, 6)}
print(json.dumps(out, indent=2))
