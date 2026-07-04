"""URDF import, free swing, turn 3 (EXTEND) -- PyChrono 10.0, headless -- contracted reference.

Loads the provided `pendulum.urdf` with ChParserURDF, fixes the root, and this time leaves the
"swing" joint PASSIVE (no actuation): the arm is released from rest at theta0 = 0.2 rad and swings
freely under gravity. The measured period must match the compound-pendulum physics DECLARED IN THE
URDF FILE (m = 2 kg, COM at d = 0.5 m, Izz_com = 0.1666667): T = 2 pi sqrt(I_pivot/(m g d)) with
the finite-amplitude correction, 1.6420 s (independent oracle). This is the turn that catches a
parser/import that drops or mangles the URDF's inertial parameters.
"""
import csv
import json
import math

import pychrono as chrono
import pychrono.parsers as parsers

theta0 = 0.2
t_end, dt = 6.0, 1.0e-3

sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

parser = parsers.ChParserURDF("pendulum.urdf")
parser.SetRootInitPose(chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
parser.PopulateSystem(sys)               # NO actuation: the revolute stays passive
parser.GetRootChBody().SetFixed(True)

arm = parser.GetChBody("arm")
arm.SetRot(chrono.QuatFromAngleZ(theta0))   # release from rest at theta0 (joint at the origin)

ts, thetas = [], []
while sys.GetChTime() < t_end:
    sys.DoStepDynamics(dt)
    q = arm.GetRot()
    ts.append(sys.GetChTime())
    thetas.append(2.0 * math.atan2(q.e3, q.e0))

amp = max(abs(x) for x in thetas)

with open("out.csv", "w", newline="") as fh:
    w = csv.writer(fh)
    w.writerow(["t", "theta"])
    for t, th in zip(ts, thetas):
        w.writerow([f"{t:.6f}", f"{th:.6e}"])

print(json.dumps({"amplitude": amp, "theta0": theta0}))
