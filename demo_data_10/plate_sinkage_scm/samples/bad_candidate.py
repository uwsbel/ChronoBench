"""A RUNS-BUT-WRONG turn-1 plate-sinkage: structurally fine and it executes cleanly (passes L1 and the
minimal L2 capability checks, real SCM terrain + soil params + step loop), but it uses the wrong soil
stiffness, Bekker Kphi = 2.0e8 instead of 2.0e6 (100x too stiff, a magnitude slip). The plate barely
sinks (~0.08 mm), so the sinkage falls far below the coarse Bekker band and the invariant fails."""
import csv
import json

import pychrono as chrono
import pychrono.vehicle as veh

Kphi, Kc, n = 2.0e8, 0.0, 1.0     # WRONG: should be 2.0e6 (100x too stiff)
cohesion, friction_deg, Janosi = 0.0, 30.0, 0.01
elastic_K, damping = 2.0e7, 1.0e4
bx, by, bz = 0.2, 0.2, 0.05
F = 500.0
g = 9.81
dt, t_end = 2.0e-3, 1.5

sys = chrono.ChSystemSMC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -g))
terrain = veh.SCMTerrain(sys, False)
terrain.SetSoilParameters(Kphi, Kc, n, cohesion, friction_deg, Janosi, elastic_K, damping)
terrain.Initialize(1.0, 1.0, 0.02)
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
