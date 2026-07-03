"""A CORRECT-BUT-DIFFERENT turn-1 gear train: same physics, different style. Uses plain ChBody
objects with explicit inertia (no ChBodyEasy helpers), mounts gear B on the NEGATIVE x side (an
equally valid external mesh), keeps gravity at its default (it exerts no torque about the gear
axes), and computes the tail means with a running accumulator. Should pass L1/L2/L3 near ceiling."""
import csv
import json
import math

import pychrono.core as chrono

RA, RB = 0.2, 0.4
OMEGA = 4.0
T_END, DT = 3.0, 1.0e-3

system = chrono.ChSystemNSC()

base = chrono.ChBody()
base.SetFixed(True)
system.Add(base)


def make_wheel(radius, x):
    b = chrono.ChBody()
    b.SetPos(chrono.ChVector3d(x, 0.0, 0.0))
    b.SetRot(chrono.QuatFromAngleX(0.5 * math.pi))
    b.SetMass(3.0)
    b.SetInertiaXX(chrono.ChVector3d(0.02, 0.01, 0.02))
    system.Add(b)
    return b


wheelA = make_wheel(RA, 0.0)
wheelB = make_wheel(RB, -(RA + RB))   # mounted on the other side: still an external mesh

drive = chrono.ChLinkMotorRotationSpeed()
drive.Initialize(wheelA, base, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
drive.SetSpeedFunction(chrono.ChFunctionConst(OMEGA))
system.Add(drive)

pivotB = chrono.ChLinkLockRevolute()
pivotB.Initialize(wheelB, base, chrono.ChFramed(chrono.ChVector3d(-(RA + RB), 0, 0)))
system.Add(pivotB)

mesh = chrono.ChLinkLockGear()
mesh.Initialize(wheelA, wheelB, chrono.ChFramed())
mesh.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-0.5 * math.pi)))
mesh.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-0.5 * math.pi)))
mesh.SetTransmissionRatio(RA / RB)
mesh.SetEnforcePhase(True)
system.Add(mesh)

rows = []
s1 = s2 = n_tail = 0
while system.GetChTime() < T_END:
    system.DoStepDynamics(DT)
    t = system.GetChTime()
    w1 = wheelA.GetAngVelParent().z
    w2 = wheelB.GetAngVelParent().z
    rows.append((t, w1, w2))
    if t >= 2.0:
        s1 += w1
        s2 += w2
        n_tail += 1

with open("out.csv", "w", newline="") as fh:
    writer = csv.writer(fh)
    writer.writerow(["t", "w1", "w2"])
    for t, w1, w2 in rows:
        writer.writerow([f"{t:.6f}", f"{w1:.6e}", f"{w2:.6e}"])

print(json.dumps({"w1_mean": s1 / n_tail, "w2_mean": s2 / n_tail, "ratio": RA / RB}))
