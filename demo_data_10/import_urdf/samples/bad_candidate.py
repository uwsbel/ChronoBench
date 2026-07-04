"""A RUNS-BUT-WRONG turn-1 URDF import: structurally fine and it executes cleanly (passes L1 and
the minimal L2 capability checks), but it SWAPS the sine arguments, ChFunctionSine(f, A) instead of
ChFunctionSine(A, f), so the joint is driven at amplitude 1.0 rad and 0.5 Hz instead of 0.5 rad at
1.0 Hz. Both CSV-derived L3 invariants (amplitude and period) catch it and the wrong-physics cap
applies."""
import csv
import json
import math

import pychrono as chrono
import pychrono.parsers as parsers

A, f = 0.5, 1.0
t_end, dt = 3.0, 1.0e-3

sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

parser = parsers.ChParserURDF("pendulum.urdf")
parser.SetRootInitPose(chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
parser.SetAllJointsActuationType(parsers.ChParserURDF.ActuationType_POSITION)
parser.PopulateSystem(sys)
parser.GetRootChBody().SetFixed(True)
parser.SetMotorFunction("swing", chrono.ChFunctionSine(f, A))   # WRONG: arguments swapped

arm = parser.GetChBody("arm")

ts, thetas = [], []
while sys.GetChTime() < t_end:
    sys.DoStepDynamics(dt)
    q = arm.GetRot()
    ts.append(sys.GetChTime())
    thetas.append(2.0 * math.atan2(q.e3, q.e0))

with open("out.csv", "w", newline="") as fh:
    w = csv.writer(fh)
    w.writerow(["t", "theta"])
    for t, th in zip(ts, thetas):
        w.writerow([f"{t:.6f}", f"{th:.6e}"])

print(json.dumps({"amplitude": max(abs(x) for x in thetas), "A": A, "f": f}))
