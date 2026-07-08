"""Isaac Sim USD arm converted to PyChrono, turn 1 (CONVERT) -- PyChrono 10.0, headless --
contracted reference.

Hand conversion of source/arm3dof_cm_v1.usda, a 3-DOF arm (base yaw about Z, shoulder/elbow
pitch about Y) authored in CENTIMETERS: metersPerUnit = 0.01 (even gravityMagnitude reads 981),
so every length converts by 0.01: column 0.30 m, upper arm 0.40 m, forearm 0.60 m, home EE at
(0, 0, 1.3). The stage's high-stiffness position DriveAPIs track the controller reference the
prompt decrees (imposed with ChLinkMotorRotationAngle + custom ChFunctions): a reach-out-and-
down pick arc, zero at both ends. FK oracle: deepest reach h = 0.5311, yaw sweep |hy| = 0.3556,
exact home return. The star trap is the unit conversion: an un-scaled candidate builds a 100x
arm and misses every band.
"""
import csv
import json
import math

import pychrono as chrono

H, L2, L3 = 0.30, 0.40, 0.60
RATE = 1.0
STEP = 1e-3
T_END = 10.0

sysNSC = chrono.ChSystemNSC()
sysNSC.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))


class Traj(chrono.ChFunction):
    """Imposed joint angle: the stage's position servos track these controller references."""

    def __init__(self, which):
        super().__init__()
        self.which = which

    def GetVal(self, t):
        if self.which == 1:
            return 0.4 * math.sin(0.2 * math.pi * RATE * t)
        if self.which == 2:
            return 0.45 * (1 - math.cos(0.4 * math.pi * RATE * t))
        return 0.35 * (1 - math.cos(0.4 * math.pi * RATE * t))

    def Clone(self):
        return Traj(self.which)


ground = chrono.ChBody()
ground.SetFixed(True)
sysNSC.AddBody(ground)

column = chrono.ChBody()
column.SetMass(2.0)
column.SetInertiaXX(chrono.ChVector3d(0.015, 0.015, 0.0002))
column.SetPos(chrono.ChVector3d(0, 0, 0.15))
sysNSC.AddBody(column)

upper = chrono.ChBody()
upper.SetMass(1.2)
upper.SetInertiaXX(chrono.ChVector3d(0.016, 0.016, 0.00015))
upper.SetPos(chrono.ChVector3d(0, 0, H + 0.20))
sysNSC.AddBody(upper)

fore = chrono.ChBody()
fore.SetMass(0.9)
fore.SetInertiaXX(chrono.ChVector3d(0.027, 0.027, 0.00012))
fore.SetPos(chrono.ChVector3d(0, 0, H + L2 + 0.30))
sysNSC.AddBody(fore)

fun1, fun2, fun3 = Traj(1), Traj(2), Traj(3)

yaw = chrono.ChLinkMotorRotationAngle()
yaw.Initialize(column, ground, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
yaw.SetAngleFunction(fun1)
sysNSC.AddLink(yaw)

pitch_frame = chrono.QuatFromAngleX(-chrono.CH_PI_2)
shoulder = chrono.ChLinkMotorRotationAngle()
shoulder.Initialize(upper, column, chrono.ChFramed(chrono.ChVector3d(0, 0, H), pitch_frame))
shoulder.SetAngleFunction(fun2)
sysNSC.AddLink(shoulder)

elbow = chrono.ChLinkMotorRotationAngle()
elbow.Initialize(fore, upper, chrono.ChFramed(chrono.ChVector3d(0, 0, H + L2), pitch_frame))
elbow.SetAngleFunction(fun3)
sysNSC.AddLink(elbow)

rows = []
t = 0.0
n = 0
while t < T_END:
    t = sysNSC.GetChTime()
    sysNSC.DoStepDynamics(STEP)
    n += 1
    if n % 5 == 0:                    # sample every 5e-3 s
        ee = fore.TransformPointLocalToParent(chrono.ChVector3d(0, 0, 0.30))
        rows.append((t, ee.x, ee.y, ee.z, math.hypot(ee.x, ee.y)))

with open("out.csv", "w", newline="") as fh:
    w = csv.writer(fh)
    w.writerow(["t", "hx", "hy", "h", "r"])
    for r in rows:
        w.writerow([f"{r[0]:.6f}", f"{r[1]:.6e}", f"{r[2]:.6e}", f"{r[3]:.6e}", f"{r[4]:.6e}"])

print(json.dumps({"h_final": rows[-1][3], "r_final": rows[-1][4], "L3": L3, "frame": "z-up"}))
