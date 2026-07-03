"""A RUNS-BUT-WRONG turn-1 bouncing ball: structurally fine and it executes cleanly (passes L1 and
the minimal L2 capability checks; SetRestitution appears in the source, on the WRONG object), but
the restitution never reaches the contact material: it is set on a second, unused material object,
so the material actually attached to the bodies keeps the default (dead) restitution and the ball
barely rebounds. The CSV-derived L3 apex invariant catches it and the wrong-physics cap applies."""
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
mat.SetFriction(0.3)                       # restitution never set on THIS material (default 0)

mat_unused = chrono.ChContactMaterialNSC() # WRONG: restitution set on a material nobody uses
mat_unused.SetRestitution(e_rest)

floor = chrono.ChBodyEasyBox(2, 0.2, 2, 1000, True, True, mat)
floor.SetPos(chrono.ChVector3d(0, -0.1, 0))
floor.SetFixed(True)
sys.Add(floor)

ball = chrono.ChBodyEasySphere(R, 1000, True, True, mat)
ball.SetPos(chrono.ChVector3d(0, h0 + R, 0))
sys.Add(ball)

ts, ys = [], []
while sys.GetChTime() < t_end:
    sys.DoStepDynamics(dt)
    ts.append(sys.GetChTime())
    ys.append(ball.GetPos().y - R)

apex1 = max(y for t, y in zip(ts, ys) if t >= 0.5)

with open("out.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["t", "y_bot"])
    for t, y in zip(ts, ys):
        w.writerow([f"{t:.6f}", f"{y:.6e}"])

print(json.dumps({"apex1": apex1, "restitution": e_rest, "method": "NSC"}))
