"""Bouncing ball, NSC contact, turn 1 (CREATE) -- PyChrono 10.0, headless -- contracted reference.

A rigid sphere (radius 0.1 m, density 1000) is released from rest with its BOTTOM 1.0 m above the
top of a fixed rigid floor, under gravity 9.81 m/s^2 along -Y. Contact uses the NSC
(non-smooth / complementarity) formulation with coefficient of restitution e = 0.7 and friction
0.3 on both bodies; the Bullet collision system must be set explicitly. Ideal impact kinematics:
the first-bounce apex of the ball bottom is e^2 * h0 = 0.49 m. Fixed step 2.0e-4 s for 1.5 s.
Logs the ball-bottom height every step; the judge measures the apex from the t >= 0.5 s window
(after the first impact at t = 0.4515 s; later apexes are lower, so the window max IS apex1).
"""
import csv
import json

import pychrono.core as chrono

e_rest = 0.7
h0 = 1.0
R = 0.1
dt = 2.0e-4
t_end = 1.5

sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

mat = chrono.ChContactMaterialNSC()
mat.SetRestitution(e_rest)
mat.SetFriction(0.3)

floor = chrono.ChBodyEasyBox(2, 0.2, 2, 1000, True, True, mat)
floor.SetPos(chrono.ChVector3d(0, -0.1, 0))      # top face at y = 0
floor.SetFixed(True)
sys.Add(floor)

ball = chrono.ChBodyEasySphere(R, 1000, True, True, mat)
ball.SetPos(chrono.ChVector3d(0, h0 + R, 0))     # ball bottom at y = h0
sys.Add(ball)

ts, ys = [], []
while sys.GetChTime() < t_end:
    sys.DoStepDynamics(dt)
    ts.append(sys.GetChTime())
    ys.append(ball.GetPos().y - R)               # ball-bottom height

apex1 = max(y for t, y in zip(ts, ys) if t >= 0.5)

with open("out.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["t", "y_bot"])
    for t, y in zip(ts, ys):
        w.writerow([f"{t:.6f}", f"{y:.6e}"])

print(json.dumps({"apex1": apex1, "restitution": e_rest, "method": "NSC"}))
