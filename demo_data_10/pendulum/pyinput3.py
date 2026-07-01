"""Pendulum turn 2 (MODIFY): large-amplitude release (PyChrono 10.0, headless) -- reference.

Same single pendulum as turn 1, but released from a LARGE angle (theta0 = 60 deg). A correct rigid-body
simulation captures the large-amplitude nonlinearity, so the measured period is noticeably LONGER than
the small-angle value 2*pi*sqrt(L/g) ~= 2.006 s (exact factor (2/pi)*K(sin(theta0/2)) ~= 1.073 at 60
deg -> ~2.15 s). A model that hardcodes the small-angle formula, or does not really integrate the
dynamics, gets this wrong. Logs (t, theta); runs longer so several periods are captured.
"""
import math
import csv
import json

import pychrono as chrono

m = 1.0
L = 1.0
g = 9.81
theta0 = math.radians(60.0)   # turn-2 change: large amplitude
t_end = 10.0
dt = 1.0e-3

sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, -g, 0.0))

ground = chrono.ChBody()
ground.SetFixed(True)
sys.AddBody(ground)

bob = chrono.ChBody()
bob.SetMass(m)
bob.SetInertiaXX(chrono.ChVector3d(1e-4, 1e-4, 1e-4))
bob.SetPos(chrono.ChVector3d(L * math.sin(theta0), -L * math.cos(theta0), 0.0))
sys.AddBody(bob)

rev = chrono.ChLinkLockRevolute()
rev.Initialize(ground, bob, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
sys.AddLink(rev)

ts, thetas = [], []
while sys.GetChTime() < t_end:
    sys.DoStepDynamics(dt)
    p = bob.GetPos()
    ts.append(sys.GetChTime())
    thetas.append(math.atan2(p.x, -p.y))

crossings = [ts[i] for i in range(1, len(thetas)) if thetas[i - 1] < 0.0 <= thetas[i]]
period_est = (crossings[-1] - crossings[0]) / (len(crossings) - 1) if len(crossings) >= 2 else float("nan")
theta_max = max(abs(t) for t in thetas)

with open("out.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["t", "theta"])
    for t, th in zip(ts, thetas):
        w.writerow([f"{t:.6f}", f"{th:.6e}"])

print(json.dumps({"period_est": period_est, "theta_max": theta_max}))
