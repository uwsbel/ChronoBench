"""Pendulum virtual experiment (PyChrono 10.0, headless) -- pilot reference.

A compact bob (mass m) at distance L from a revolute pivot at the origin, swinging in the X-Y plane
about global Z under gravity -Y, released from a small angle. Logs (t, theta) and reports the
estimated oscillation period and peak angle. No visualization (headless).
"""
import math
import csv
import json

import pychrono as chrono

# --- prompt parameters (from the contract) ---
m = 1.0          # bob mass [kg]
L = 1.0          # pivot-to-bob distance [m]
g = 9.81         # gravity [m/s^2]
theta0 = math.radians(5.0)
t_end = 5.0
dt = 1.0e-3

sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, -g, 0.0))

# Ground (fixed) at the pivot.
ground = chrono.ChBody()
ground.SetFixed(True)
sys.AddBody(ground)

# Compact bob: small own-inertia so the body acts as a point mass at distance L
# (then the small-angle period is the simple-pendulum value 2*pi*sqrt(L/g)).
bob = chrono.ChBody()
bob.SetMass(m)
bob.SetInertiaXX(chrono.ChVector3d(1e-4, 1e-4, 1e-4))
# Released from theta0 measured from the downward vertical (equilibrium at (0,-L,0)).
bob.SetPos(chrono.ChVector3d(L * math.sin(theta0), -L * math.cos(theta0), 0.0))
sys.AddBody(bob)

# Revolute joint at the origin, axis = global Z (default frame Z).
rev = chrono.ChLinkLockRevolute()
rev.Initialize(ground, bob, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
sys.AddLink(rev)

ts, thetas = [], []
while sys.GetChTime() < t_end:
    sys.DoStepDynamics(dt)
    p = bob.GetPos()
    theta = math.atan2(p.x, -p.y)   # angle from the downward vertical; 0 at equilibrium
    ts.append(sys.GetChTime())
    thetas.append(theta)

# Period from upward zero-crossings of theta(t).
crossings = [ts[i] for i in range(1, len(thetas)) if thetas[i - 1] < 0.0 <= thetas[i]]
if len(crossings) >= 2:
    period_est = (crossings[-1] - crossings[0]) / (len(crossings) - 1)
else:
    period_est = float("nan")
theta_max = max(abs(t) for t in thetas)

with open("out.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["t", "theta"])
    for t, th in zip(ts, thetas):
        w.writerow([f"{t:.6f}", f"{th:.6e}"])

print(json.dumps({"period_est": period_est, "theta_max": theta_max}))
