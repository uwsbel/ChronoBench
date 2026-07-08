"""Isaac USD arm conversion, turn 3 (EXTEND) -- PyChrono 10.0, headless -- contracted
reference.

The SAME physical arm as turn 2, re-exported (source/arm3dof_m_yup_v3.usda) with
metersPerUnit = 1.0 and upAxis = Y: lengths already in meters, up is now +Y, the yaw joint
spins about Y, the pitch joints about X, and the yaw-sweep horizontal direction is X. The
conversion must follow the conventions, not the numbers: gravity along -Y, links along +Y,
and the log reports h = the up-axis coordinate (y) with hy = the sweep direction (x). NOTHING
physical changes: every turn-2 band holds verbatim (deepest reach 0.5253, sweep 0.4334, home
1.5, radius 0), which is itself the graded truth. Rescaling a second time (100x) or continuing
to report z as height both fail.
"""
import csv
import json
import math

import pychrono as chrono

H, L2, L3 = 0.30, 0.40, 0.80
RATE = 1.0
STEP = 1e-3
T_END = 10.0

sysNSC = chrono.ChSystemNSC()
sysNSC.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))


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
column.SetInertiaXX(chrono.ChVector3d(0.015, 0.0002, 0.015))
column.SetPos(chrono.ChVector3d(0, 0.15, 0))
sysNSC.AddBody(column)

upper = chrono.ChBody()
upper.SetMass(1.2)
upper.SetInertiaXX(chrono.ChVector3d(0.016, 0.00015, 0.016))
upper.SetPos(chrono.ChVector3d(0, H + 0.20, 0))
sysNSC.AddBody(upper)

fore = chrono.ChBody()
fore.SetMass(1.2)
fore.SetInertiaXX(chrono.ChVector3d(0.064, 0.00016, 0.064))
fore.SetPos(chrono.ChVector3d(0, H + L2 + 0.40, 0))
sysNSC.AddBody(fore)

fun1, fun2, fun3 = Traj(1), Traj(2), Traj(3)

# yaw about global Y: rotate the motor frame's z onto y
yaw = chrono.ChLinkMotorRotationAngle()
yaw.Initialize(column, ground, chrono.ChFramed(chrono.ChVector3d(0, 0, 0),
                                               chrono.QuatFromAngleX(-chrono.CH_PI_2)))
yaw.SetAngleFunction(fun1)
sysNSC.AddLink(yaw)

# pitches about global X: rotate the motor frames' z onto x
pitch_frame = chrono.QuatFromAngleY(chrono.CH_PI_2)
shoulder = chrono.ChLinkMotorRotationAngle()
shoulder.Initialize(upper, column, chrono.ChFramed(chrono.ChVector3d(0, H, 0), pitch_frame))
shoulder.SetAngleFunction(fun2)
sysNSC.AddLink(shoulder)

elbow = chrono.ChLinkMotorRotationAngle()
elbow.Initialize(fore, upper, chrono.ChFramed(chrono.ChVector3d(0, H + L2, 0), pitch_frame))
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
        ee = fore.TransformPointLocalToParent(chrono.ChVector3d(0, 0.40, 0))
        # h = the up-axis coordinate (y); hy = the yaw-sweep horizontal (x); hx = the other (z)
        rows.append((t, ee.z, ee.x, ee.y, math.hypot(ee.x, ee.z)))

with open("out.csv", "w", newline="") as fh:
    w = csv.writer(fh)
    w.writerow(["t", "hx", "hy", "h", "r"])
    for r in rows:
        w.writerow([f"{r[0]:.6f}", f"{r[1]:.6e}", f"{r[2]:.6e}", f"{r[3]:.6e}", f"{r[4]:.6e}"])

print(json.dumps({"h_final": rows[-1][3], "r_final": rows[-1][4], "L3": L3, "frame": "y-up"}))
