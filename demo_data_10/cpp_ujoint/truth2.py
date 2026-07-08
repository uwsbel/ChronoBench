"""Universal-joint translation, turn 2 (MODIFY) -- PyChrono 10.0, headless -- contracted
reference.

Same translated mechanism as turn 1, with the bend angle increased 30 -> 45 degrees (the shaft-2
position and every joint frame follow the new angle, as in the C++ original where everything is
derived from `angle`). Cardan kinematics sharpens: omega2 now oscillates between cos 45 and
1/cos 45 times the drive rate, i.e. signed values in [-1.4142, -0.7071] with the demo's motor
convention; an unmodified 30-degree candidate (-1.1547/-0.8660) fails BOTH extremum bands.
"""
import csv
import json
import math

import pychrono as chrono

HL = 2.0
BETA = math.pi / 4
OMEGA1 = 1.0
STEP = 5e-3
T_END = 10.0

sysNSC = chrono.ChSystemNSC()
sysNSC.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

cosa, sina = math.cos(BETA), math.sin(BETA)
rot = chrono.QuatFromAngleX(BETA)

ground = chrono.ChBody()
ground.SetFixed(True)
ground.EnableCollision(False)
sysNSC.AddBody(ground)

shaft_1 = chrono.ChBody()
shaft_1.SetFixed(False)
shaft_1.EnableCollision(False)
shaft_1.SetMass(1)
shaft_1.SetInertiaXX(chrono.ChVector3d(1, 1, 0.2))
shaft_1.SetPos(chrono.ChVector3d(0, 0, -HL))
shaft_1.SetRot(chrono.ChQuaterniond(1, 0, 0, 0))
sysNSC.AddBody(shaft_1)

shaft_2 = chrono.ChBody()
shaft_2.SetFixed(False)
shaft_2.EnableCollision(False)
shaft_2.SetMass(1)
shaft_2.SetInertiaXX(chrono.ChVector3d(1, 1, 0.2))
shaft_2.SetPos(chrono.ChVector3d(0, -HL * sina, HL * cosa))
shaft_2.SetRot(rot)
sysNSC.AddBody(shaft_2)

motor = chrono.ChLinkMotorRotationAngle()
motor.Initialize(ground, shaft_1, chrono.ChFramed(chrono.ChVector3d(0, 0, -HL), chrono.ChQuaterniond(1, 0, 0, 0)))
motor.SetAngleFunction(chrono.ChFunctionRamp(0, OMEGA1))
sysNSC.AddLink(motor)

cyljoint = chrono.ChLinkLockCylindrical()
cyljoint.Initialize(ground, shaft_2, chrono.ChFramed(chrono.ChVector3d(0, -HL * sina, HL * cosa), rot))
sysNSC.AddLink(cyljoint)

ujoint = chrono.ChLinkUniversal()
ujoint.Initialize(shaft_1, shaft_2, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), rot))
sysNSC.AddLink(ujoint)

rows = []
t = 0.0
while t < T_END:
    t = sysNSC.GetChTime()
    sysNSC.DoStepDynamics(STEP)
    rows.append((t, shaft_1.GetAngVelLocal().z, shaft_2.GetAngVelLocal().z))

with open("out.csv", "w", newline="") as fh:
    w = csv.writer(fh)
    w.writerow(["t", "w1", "w2"])
    for r in rows:
        w.writerow([f"{r[0]:.6f}", f"{r[1]:.6e}", f"{r[2]:.6e}"])

tail = [r for r in rows if r[0] >= 2.0]
print(json.dumps({"omega2_min": min(r[2] for r in tail),
                  "omega2_max": max(r[2] for r in tail),
                  "omega1_mean": sum(r[1] for r in tail) / len(tail)}))
