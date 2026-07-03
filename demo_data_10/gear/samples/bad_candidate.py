"""A RUNS-BUT-WRONG turn-1 gear train: structurally fine and it executes cleanly (passes L1 and the
minimal L2 capability checks), but it INVERTS the transmission ratio (r2/r1 = 2.0 instead of
r1/r2 = 0.5), the classic ratio-direction bug. The driven gear then spins at -8 rad/s instead of
-2 rad/s; the CSV-derived L3 invariant catches it and the wrong-physics cap applies."""
import csv
import json
import math

import pychrono.core as chrono

r1, r2 = 0.2, 0.4
omega_in = 4.0
t_end, dt = 3.0, 1.0e-3

sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

truss = chrono.ChBodyEasyBox(1.5, 1.5, 0.1, 1000)
truss.SetPos(chrono.ChVector3d(0, 0, -0.2))
truss.SetFixed(True)
sys.Add(truss)

gearA = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, r1, 0.05, 1000)
gearA.SetPos(chrono.ChVector3d(0, 0, 0))
gearA.SetRot(chrono.QuatFromAngleX(math.pi / 2))
sys.Add(gearA)

gearB = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, r2, 0.05, 1000)
gearB.SetPos(chrono.ChVector3d(r1 + r2, 0, 0))
gearB.SetRot(chrono.QuatFromAngleX(math.pi / 2))
sys.Add(gearB)

motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(gearA, truss, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
motor.SetSpeedFunction(chrono.ChFunctionConst(omega_in))
sys.Add(motor)

revB = chrono.ChLinkLockRevolute()
revB.Initialize(gearB, truss, chrono.ChFramed(chrono.ChVector3d(r1 + r2, 0, 0)))
sys.Add(revB)

gearAB = chrono.ChLinkLockGear()
gearAB.Initialize(gearA, gearB, chrono.ChFramed())
gearAB.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-math.pi / 2)))
gearAB.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-math.pi / 2)))
gearAB.SetTransmissionRatio(r2 / r1)   # WRONG: inverted ratio (2.0 instead of 0.5)
gearAB.SetEnforcePhase(True)
sys.Add(gearAB)

ts, w1s, w2s = [], [], []
while sys.GetChTime() < t_end:
    sys.DoStepDynamics(dt)
    ts.append(sys.GetChTime())
    w1s.append(gearA.GetAngVelParent().z)
    w2s.append(gearB.GetAngVelParent().z)

tail = [i for i, t in enumerate(ts) if t >= 2.0]
w1_mean = sum(w1s[i] for i in tail) / len(tail)
w2_mean = sum(w2s[i] for i in tail) / len(tail)

with open("out.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["t", "w1", "w2"])
    for t, a, b in zip(ts, w1s, w2s):
        w.writerow([f"{t:.6f}", f"{a:.6e}", f"{b:.6e}"])

print(json.dumps({"w1_mean": w1_mean, "w2_mean": w2_mean, "ratio": r2 / r1}))
