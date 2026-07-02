"""Plate-sinkage on SCM, turn 2 (MODIFY: softer soil) -- PyChrono 10.0, headless, reference.

Same plate and load as turn 1, but the soil is made SOFTER: Bekker Kphi is lowered from 2e6 to 5e5 (4x
softer). Softer soil -> the plate sinks deeper (the Bekker band shifts up ~4x). Logs (t, sinkage).
"""
import csv
import json

import pychrono as chrono
import pychrono.vehicle as veh

Kphi, Kc, n = 5.0e5, 0.0, 1.0     # turn-2 change: 2e6 -> 5e5 (4x softer)
cohesion, friction_deg, Janosi = 0.0, 30.0, 0.01
elastic_K, damping = 2.0e7, 1.0e4
bx, by, bz = 0.2, 0.2, 0.05
F = 500.0
g = 9.81
dt, t_end = 2.0e-3, 1.5
sizeX, sizeY, delta = 1.0, 1.0, 0.02

sys = chrono.ChSystemSMC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -g))

terrain = veh.SCMTerrain(sys, False)
terrain.SetSoilParameters(Kphi, Kc, n, cohesion, friction_deg, Janosi, elastic_K, damping)
terrain.Initialize(sizeX, sizeY, delta)

mat = chrono.ChContactMaterialSMC()
mat.SetFriction(0.8)
mat.SetYoungModulus(1.0e7)

plate = chrono.ChBodyEasyBox(bx, by, bz, (F / g) / (bx * by * bz), True, True, mat)
plate.SetPos(chrono.ChVector3d(0, 0, bz / 2.0))
sys.Add(plate)
z0 = plate.GetPos().z

ts, sink = [], []
t = 0.0
while t < t_end:
    terrain.Synchronize(t)
    sys.DoStepDynamics(dt)
    terrain.Advance(dt)
    t = sys.GetChTime()
    ts.append(t)
    sink.append(z0 - plate.GetPos().z)

with open("out.csv", "w", newline="") as fo:
    w = csv.writer(fo)
    w.writerow(["t", "sinkage"])
    for a, s in zip(ts, sink):
        w.writerow([f"{a:.5f}", f"{s:.6e}"])

print(json.dumps({"sinkage": sink[-1], "load_N": F}))
