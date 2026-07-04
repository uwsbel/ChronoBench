"""URDF import + actuation, turn 1 (CREATE) -- PyChrono 10.0, headless -- contracted reference.

Loads the provided `pendulum.urdf` (in the current working directory) with ChParserURDF, fixes the
root, sets the named revolute joint "swing" to POSITION actuation, and drives it with
theta(t) = A sin(2 pi f t), A = 0.5 rad, f = 1.0 Hz. The logged joint angle must reproduce the
imposed kinematics exactly: amplitude 0.5 rad, period 1.0 s (independent oracle: closed form of
the actuation, no dynamics involved).
"""
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
parser.SetMotorFunction("swing", chrono.ChFunctionSine(A, f))

arm = parser.GetChBody("arm")

ts, thetas = [], []
while sys.GetChTime() < t_end:
    sys.DoStepDynamics(dt)
    q = arm.GetRot()
    ts.append(sys.GetChTime())
    thetas.append(2.0 * math.atan2(q.e3, q.e0))   # planar rotation about +Z

amp = max(abs(x) for x in thetas)

with open("out.csv", "w", newline="") as fh:
    w = csv.writer(fh)
    w.writerow(["t", "theta"])
    for t, th in zip(ts, thetas):
        w.writerow([f"{t:.6f}", f"{th:.6e}"])

print(json.dumps({"amplitude": amp, "A": A, "f": f}))
