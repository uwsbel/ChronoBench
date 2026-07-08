"""Universal-joint translation, turn 3 (EXTEND) -- PyChrono 10.0, headless -- contracted
reference.

Same 45-degree mechanism as turn 2, driven twice as fast: the motor ramp slope doubles to
2 rad/s. Everything scales with the drive rate: signed omega2 now oscillates in
[-2.8284, -1.4142] and one full speed oscillation takes pi/omega1 = 1.5708 s (the judge also
measures the oscillation period from crossings of the drive level). A candidate still at
1 rad/s fails the extremum bands, the drive-mean band, and the period band.
"""
import csv
import json
import math

import pychrono as chrono

HL = 2.0
BETA = math.pi / 4
OMEGA1 = 2.0
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
