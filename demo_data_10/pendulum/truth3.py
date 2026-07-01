"""Pendulum turn 3 (EXTEND): double pendulum with energy conservation (PyChrono 10.0, headless) -- reference.

Extends the single pendulum into a planar DOUBLE pendulum: a second compact bob hung from the first by
a second revolute joint, both swinging about global Z. The motion is (mildly) chaotic, so there is no
clean period; the robust, un-gameable invariant is CONSERVATION OF TOTAL MECHANICAL ENERGY -- a
frictionless, undriven system should keep E ~ constant, and a correct model with a decent integrator
holds the relative drift small. Logs (t, theta1, theta2, energy). Energy uses PE = 0 at each bob's
lowest reachable point, so E(0) is comfortably positive and the relative-drift metric is well posed.
"""
import math
import csv
import json

import pychrono as chrono

m1 = m2 = 1.0
L1 = L2 = 1.0
g = 9.81
t_end = 10.0
dt = 1.0e-3

sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, -g, 0.0))

ground = chrono.ChBody()
ground.SetFixed(True)
sys.AddBody(ground)

# Both arms released horizontal (theta = 90 deg from the downward vertical), from rest.
bob1 = chrono.ChBody()
bob1.SetMass(m1)
bob1.SetInertiaXX(chrono.ChVector3d(1e-4, 1e-4, 1e-4))
bob1.SetPos(chrono.ChVector3d(L1, 0.0, 0.0))
sys.AddBody(bob1)

bob2 = chrono.ChBody()
bob2.SetMass(m2)
bob2.SetInertiaXX(chrono.ChVector3d(1e-4, 1e-4, 1e-4))
bob2.SetPos(chrono.ChVector3d(L1 + L2, 0.0, 0.0))
sys.AddBody(bob2)

rev1 = chrono.ChLinkLockRevolute()
rev1.Initialize(ground, bob1, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
sys.AddLink(rev1)

rev2 = chrono.ChLinkLockRevolute()
rev2.Initialize(bob1, bob2, chrono.ChFramed(chrono.ChVector3d(L1, 0, 0), chrono.QUNIT))
sys.AddLink(rev2)


def total_energy():
    p1, p2 = bob1.GetPos(), bob2.GetPos()
    v1, v2 = bob1.GetPosDt(), bob2.GetPosDt()
    ke = 0.5 * m1 * (v1.x ** 2 + v1.y ** 2 + v1.z ** 2) + 0.5 * m2 * (v2.x ** 2 + v2.y ** 2 + v2.z ** 2)
    pe = m1 * g * (p1.y + L1) + m2 * g * (p2.y + L1 + L2)   # PE = 0 at each bob's lowest point
    return ke + pe


ts, th1s, th2s, energies = [], [], [], []
while sys.GetChTime() < t_end:
    sys.DoStepDynamics(dt)
    p1, p2 = bob1.GetPos(), bob2.GetPos()
    ts.append(sys.GetChTime())
    th1s.append(math.atan2(p1.x, -p1.y))
    th2s.append(math.atan2(p2.x - p1.x, -(p2.y - p1.y)))   # arm-2 angle from vertical
    energies.append(total_energy())

e0 = energies[0]
energy_drift = max(abs(e - e0) for e in energies) / abs(e0)
theta2_range = max(th2s) - min(th2s)

with open("out.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["t", "theta1", "theta2", "energy"])
    for t, a1, a2, e in zip(ts, th1s, th2s, energies):
        w.writerow([f"{t:.6f}", f"{a1:.6e}", f"{a2:.6e}", f"{e:.6e}"])

print(json.dumps({"energy_drift": energy_drift, "theta2_range": theta2_range}))
