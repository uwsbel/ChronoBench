"""Plate-sinkage on SCM deformable terrain, turn 1 (CREATE) -- PyChrono 10.0, headless, reference.

A rigid flat plate rests on Bekker-Wong SCM (Soil Contact Model) deformable terrain under its own weight
(vertical load F) and sinks to equilibrium (a bevameter-style pressure-sinkage test). Logs (t, sinkage),
sinkage = drop of the plate below the initial soil surface. The independent Bekker oracle sets a COARSE
band (SCM sinkage runs ~1.0-1.5x the ideal Bekker value). Baseline params: Kphi=2e6 (Kc=0,n=1), plate
0.2x0.2x0.05 m, load F=500 N.
"""
import csv
import json

import pychrono as chrono
import pychrono.vehicle as veh

# --- declared params (match contract.json `params` + oracle.py) ---
Kphi, Kc, n = 2.0e6, 0.0, 1.0
cohesion, friction_deg, Janosi = 0.0, 30.0, 0.01
elastic_K, damping = 2.0e7, 1.0e4
bx, by, bz = 0.2, 0.2, 0.05
F = 500.0
g = 9.81
dt, t_end = 2.0e-3, 1.5
sizeX, sizeY, delta = 1.0, 1.0, 0.02

sys = chrono.ChSystemSMC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)   # 10.0: required before SCMTerrain
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -g))

terrain = veh.SCMTerrain(sys, False)
terrain.SetSoilParameters(Kphi, Kc, n, cohesion, friction_deg, Janosi, elastic_K, damping)
terrain.Initialize(sizeX, sizeY, delta)

mat = chrono.ChContactMaterialSMC()
mat.SetFriction(0.8)
mat.SetYoungModulus(1.0e7)

plate = chrono.ChBodyEasyBox(bx, by, bz, (F / g) / (bx * by * bz), True, True, mat)
plate.SetPos(chrono.ChVector3d(0, 0, bz / 2.0))     # plate bottom resting at the soil surface (z=0)
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
