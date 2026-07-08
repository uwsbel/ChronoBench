"""Double-pendulum conversion, turn 3 (EXTEND) -- PyChrono 10.0, headless -- contracted
reference.

Same long-rod-2 pendulum as turn 2, released BENT: theta1 = 0.1, theta2 = -0.1 (the second rod
tilted opposite; note the second body's placement follows ITS OWN release angle from the tip of
rod 1). The opposite bend pumps the counter-phase (fast) mode: the RELATIVE angle theta2 -
theta1 peaks at 0.208 rad (RK4 oracle; a straight-release candidate reaches only 0.1385), which
is the turn's discriminating band. Energy is still conserved.
"""
import csv
import json
import math

import pychrono as chrono

M1, L1 = 1.0, 1.0
M2, L2 = 1.5, 1.5
TH1_0, TH2_0 = 0.1, -0.1
PIVOT = chrono.ChVector3d(0, 0, 2)
STEP = 1e-3
T_END = 10.0
G = 9.81

I1 = M1 * L1 ** 2 / 12
I2 = M2 * L2 ** 2 / 12

sysNSC = chrono.ChSystemNSC()
sysNSC.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -G))

ground = chrono.ChBody()
ground.SetFixed(True)
sysNSC.AddBody(ground)

rot1 = chrono.QuatFromAngleY(TH1_0)
com1 = PIVOT + rot1.Rotate(chrono.ChVector3d(0, 0, -L1 / 2))
rod1 = chrono.ChBody()
rod1.SetMass(M1)
rod1.SetInertiaXX(chrono.ChVector3d(I1, I1, 0.0001))
rod1.SetPos(com1)
rod1.SetRot(rot1)
sysNSC.AddBody(rod1)

tip1 = PIVOT + rot1.Rotate(chrono.ChVector3d(0, 0, -L1))
rot2 = chrono.QuatFromAngleY(TH2_0)
com2 = tip1 + rot2.Rotate(chrono.ChVector3d(0, 0, -L2 / 2))
rod2 = chrono.ChBody()
rod2.SetMass(M2)
rod2.SetInertiaXX(chrono.ChVector3d(I2, I2, 0.0001))
rod2.SetPos(com2)
rod2.SetRot(rot2)
sysNSC.AddBody(rod2)

hinge1 = chrono.ChLinkLockRevolute()
hinge1.Initialize(ground, rod1, chrono.ChFramed(PIVOT, chrono.QuatFromAngleX(-chrono.CH_PI_2)))
sysNSC.AddLink(hinge1)

hinge2 = chrono.ChLinkLockRevolute()
hinge2.Initialize(rod1, rod2, chrono.ChFramed(tip1, chrono.QuatFromAngleX(-chrono.CH_PI_2)))
sysNSC.AddLink(hinge2)


def y_angle(body):
    q = body.GetRot()
    return 2.0 * math.atan2(q.e2, q.e0)


def total_energy():
    e = 0.0
    for body, izz in ((rod1, I1), (rod2, I2)):
        v = body.GetPosDt()
        w = body.GetAngVelLocal()
        ine = body.GetInertiaXX()
        e += 0.5 * body.GetMass() * v.Length2()
        e += 0.5 * (ine.x * w.x ** 2 + ine.y * w.y ** 2 + ine.z * w.z ** 2)
        e += body.GetMass() * G * body.GetPos().z
    return e


rows = []
t = 0.0
while t < T_END:
    t = sysNSC.GetChTime()
    sysNSC.DoStepDynamics(STEP)
    a1, a2 = y_angle(rod1), y_angle(rod2)
    rows.append((t, a1, a2, a2 - a1, total_energy()))

with open("out.csv", "w", newline="") as fh:
    w = csv.writer(fh)
    w.writerow(["t", "th1", "th2", "rel", "e"])
    for r in rows:
        w.writerow([f"{r[0]:.6f}", f"{r[1]:.6e}", f"{r[2]:.6e}", f"{r[3]:.6e}", f"{r[4]:.6e}"])

print(json.dumps({"theta1_max": max(abs(r[1]) for r in rows),
                  "theta2_max": max(abs(r[2]) for r in rows),
                  "config": "long-rod2-bent-release"}))
