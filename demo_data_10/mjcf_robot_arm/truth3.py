"""3-DOF arm conversion, turn 3 (EXTEND) -- PyChrono 10.0, headless -- contracted reference.

Same long-armed robot as turn 2, its trajectory RETIMED 2x: every joint reference runs at
double rate (two full pick cycles in the 10 s run, still ending home). Pure kinematics keeps
the geometric extremes identical (z_min 1.4131, home 1.6); what doubles is the end effector
SPEED: peak |dz/dt| goes 0.1519 -> 0.3039 (FK oracle), the turn discriminator.
"""
import csv
import json
import math

import pychrono as chrono

H, L2, L3 = 0.4, 0.7, 0.5
RATE = 2.0
STEP = 1e-3
T_END = 10.0

sysNSC = chrono.ChSystemNSC()
sysNSC.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))


class Traj(chrono.ChFunction):
    """Imposed joint angle: the MJCF position servos track these controller references."""

    def __init__(self, which):
        super().__init__()
        self.which = which

    def GetVal(self, t):
        if self.which == 1:
            return 0.5 * math.sin(0.2 * math.pi * RATE * t)
        if self.which == 2:
            return 0.35 * (1 - math.cos(0.4 * math.pi * RATE * t))
        return -0.5 * (1 - math.cos(0.4 * math.pi * RATE * t))

    def Clone(self):
        return Traj(self.which)


ground = chrono.ChBody()
ground.SetFixed(True)
sysNSC.AddBody(ground)

column = chrono.ChBody()
column.SetMass(2.0)
column.SetInertiaXX(chrono.ChVector3d(0.0266667, 0.0266667, 0.002))
column.SetPos(chrono.ChVector3d(0, 0, 0.2))
sysNSC.AddBody(column)

upper = chrono.ChBody()
upper.SetMass(1.4)
upper.SetInertiaXX(chrono.ChVector3d(0.0571667, 0.0571667, 0.0007))
upper.SetPos(chrono.ChVector3d(0, 0, H + 0.35))
sysNSC.AddBody(upper)

fore = chrono.ChBody()
fore.SetMass(1.0)
fore.SetInertiaXX(chrono.ChVector3d(0.0208333, 0.0208333, 0.0005))
fore.SetPos(chrono.ChVector3d(0, 0, H + L2 + 0.25))
sysNSC.AddBody(fore)

fun1, fun2, fun3 = Traj(1), Traj(2), Traj(3)

# yaw: motor rotates about its frame's z = global z; body order (link, parent) so the link
# takes the +q rotation (a motor angle applies to body1 relative to body2)
yaw = chrono.ChLinkMotorRotationAngle()
yaw.Initialize(column, ground, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
yaw.SetAngleFunction(fun1)
sysNSC.AddLink(yaw)

# shoulder and elbow: pitch about y -> rotate the motor frames' z onto global y
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
        ee = fore.TransformPointLocalToParent(chrono.ChVector3d(0, 0, 0.25))
        rows.append((t, ee.x, ee.y, ee.z, math.hypot(ee.x, ee.y)))

with open("out.csv", "w", newline="") as fh:
    w = csv.writer(fh)
    w.writerow(["t", "x", "y", "z", "r"])
    for r in rows:
        w.writerow([f"{r[0]:.6f}", f"{r[1]:.6e}", f"{r[2]:.6e}", f"{r[3]:.6e}", f"{r[4]:.6e}"])

print(json.dumps({"z_final": rows[-1][3], "r_final": rows[-1][4], "L2": L2,
                  "rate_scale": RATE}))
