"""Independent oracle for the isaac_robot_arm task (stdlib math only, NO Chrono/Isaac).

A 3-DOF arm (base yaw + shoulder/elbow pitch) sourced from an Isaac Sim USD stage authored in
CENTIMETERS (metersPerUnit = 0.01, Z-up; column 30 cm, upper arm 40 cm, forearm 60 cm), with
high-stiffness position DriveAPIs whose reference trajectory is decreed in the prompt (the
imposed-motion mapping). Under imposed joint motion the end effector follows closed-form
forward kinematics, with pitch angles from vertical:

    h = H + L2 cos(q2) + L3 cos(q2 + q3)          (h = the up-axis coordinate)
    r = L2 sin(q2) + L3 sin(q2 + q3)
    hx = r cos(q1),   hy = r sin(q1)              (hy = the yaw-sweep horizontal direction)

Trajectories (all zero at t = 0 and t = 10, so the arm starts and ends straight up):

    q1(t) = 0.4 sin(0.2 pi t)
    q2(t) = 0.45 (1 - cos(0.4 pi t))
    q3(t) = 0.35 (1 - cos(0.4 pi t))     (elbow bends the SAME way: a reach-out-and-down arc,
                                          so the yaw peak coincides with full extension)

  turn 1 (convert): H, L2, L3 = 0.30, 0.40, 0.60 m, authored as 30/40/60 in the cm stage: the
                    star trap is metersPerUnit, a candidate that ignores it builds a 100x arm.
  turn 2 (modify):  the stage's forearm grows 60 -> 80 cm: everything rescales per FK.
  turn 3 (extend):  the SAME arm re-exported with metersPerUnit = 1.0 and upAxis = Y (numbers
                    rescaled, axes permuted). NOTHING physical changes: every turn-2 value
                    holds verbatim (this oracle proves the invariance by construction), with
                    height reported along the stage's up axis. Rescaling twice (100x) or
                    keeping z as height in the Y-up world both fail.

Printed values (numeric sweep at dt = 1e-4) freeze the contract bands.
Reproduce: conda run -n chronobench python demo_data_10/isaac_robot_arm/oracle.py
"""
import json
import math

H = 0.30


def sweep(L2, L3, t_end=10.0, dt=1e-4):
    hmin, hymax, dhmax = 10.0, 0.0, 0.0
    hprev = None
    n = int(round(t_end / dt))
    for k in range(n + 1):
        t = k * dt
        q1 = 0.4 * math.sin(0.2 * math.pi * t)
        q2 = 0.45 * (1 - math.cos(0.4 * math.pi * t))
        q3 = 0.35 * (1 - math.cos(0.4 * math.pi * t))
        h = H + L2 * math.cos(q2) + L3 * math.cos(q2 + q3)
        r = L2 * math.sin(q2) + L3 * math.sin(q2 + q3)
        hy = r * math.sin(q1)
        hmin = min(hmin, h)
        hymax = max(hymax, abs(hy))
        if hprev is not None:
            dhmax = max(dhmax, abs(h - hprev) / dt)
        hprev = h
    return hmin, hymax, dhmax, h, abs(r)


out = {}
for name, L3 in (("turn1", 0.60), ("turn2", 0.80), ("turn3_same_arm_reexported", 0.80)):
    hmin, hymax, dhmax, hf, rf = sweep(0.40, L3)
    out[name] = {"L3_m": L3, "h_min": round(hmin, 4), "hy_maxabs": round(hymax, 4),
                 "dh_rate_max": round(dhmax, 4), "h_final": round(hf, 4),
                 "r_final": round(rf, 6)}
print(json.dumps(out, indent=2))
