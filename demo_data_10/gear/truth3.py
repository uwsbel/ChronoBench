"""Two-stage gear train (PyChrono 10.0, headless) -- contracted reference, turn 3.

Extends the turn-2 train (r1=0.2 driving r2=0.6, motor at 4 rad/s about global Z) with a second
stage: a third gear C of pitch radius 0.2 m externally meshing with gear B on its own fixed,
parallel axis. Ideal kinematics: w2 = -(r1/r2)*w1 = -4/3 rad/s (first external mesh reverses),
w3 = -(r2/r3)*w2 = -(0.6/0.2)*(-4/3) = +4.0 rad/s (second mesh reverses again). Two external
meshes restore the sense of rotation, and the compound ratio (r1/r2)*(r2/r3) = r1/r3 = 1 makes
|w3| = |omega_in| exactly.
"""
import csv
import json
import math

import pychrono.core as chrono

r1, r2, r3 = 0.2, 0.6, 0.2
omega_in = 4.0
t_end, dt = 3.0, 1.0e-3

sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

truss = chrono.ChBodyEasyBox(3.0, 1.5, 0.1, 1000)
truss.SetPos(chrono.ChVector3d(0.8, 0, -0.2))
truss.SetFixed(True)
sys.Add(truss)

# Gears as thin cylinders; ChBodyEasyCylinder's axis is local Y, rotated here onto global Z.
gearA = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, r1, 0.05, 1000)
gearA.SetPos(chrono.ChVector3d(0, 0, 0))
gearA.SetRot(chrono.QuatFromAngleX(math.pi / 2))
sys.Add(gearA)

gearB = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, r2, 0.05, 1000)
gearB.SetPos(chrono.ChVector3d(r1 + r2, 0, 0))
gearB.SetRot(chrono.QuatFromAngleX(math.pi / 2))
sys.Add(gearB)

gearC = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, r3, 0.05, 1000)
gearC.SetPos(chrono.ChVector3d(r1 + r2 + r2 + r3, 0, 0))
gearC.SetRot(chrono.QuatFromAngleX(math.pi / 2))
sys.Add(gearC)

# Drive gear A about global Z; the motor also serves as gear A's revolute mount.
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(gearA, truss, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
motor.SetSpeedFunction(chrono.ChFunctionConst(omega_in))
sys.Add(motor)

# Gears B and C each spin about their own fixed, parallel axes.
revB = chrono.ChLinkLockRevolute()
revB.Initialize(gearB, truss, chrono.ChFramed(chrono.ChVector3d(r1 + r2, 0, 0)))
sys.Add(revB)

revC = chrono.ChLinkLockRevolute()
revC.Initialize(gearC, truss, chrono.ChFramed(chrono.ChVector3d(r1 + r2 + r2 + r3, 0, 0)))
sys.Add(revC)

# Stage 1: gear constraint A-B (shaft frames local, frame Z along each wheel axis).
gearAB = chrono.ChLinkLockGear()
gearAB.Initialize(gearA, gearB, chrono.ChFramed())
gearAB.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-math.pi / 2)))
gearAB.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-math.pi / 2)))
gearAB.SetTransmissionRatio(r1 / r2)
gearAB.SetEnforcePhase(True)
sys.Add(gearAB)

# Stage 2: gear constraint B-C.
gearBC = chrono.ChLinkLockGear()
gearBC.Initialize(gearB, gearC, chrono.ChFramed())
gearBC.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-math.pi / 2)))
gearBC.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-math.pi / 2)))
gearBC.SetTransmissionRatio(r2 / r3)
gearBC.SetEnforcePhase(True)
sys.Add(gearBC)

ts, w1s, w2s, w3s = [], [], [], []
while sys.GetChTime() < t_end:
    sys.DoStepDynamics(dt)
    ts.append(sys.GetChTime())
    w1s.append(gearA.GetAngVelParent().z)
    w2s.append(gearB.GetAngVelParent().z)
    w3s.append(gearC.GetAngVelParent().z)

tail = [i for i, t in enumerate(ts) if t >= 2.0]
w1_mean = sum(w1s[i] for i in tail) / len(tail)
w2_mean = sum(w2s[i] for i in tail) / len(tail)
w3_mean = sum(w3s[i] for i in tail) / len(tail)

with open("out.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["t", "w1", "w2", "w3"])
    for t, a, b, c in zip(ts, w1s, w2s, w3s):
        w.writerow([f"{t:.6f}", f"{a:.6e}", f"{b:.6e}", f"{c:.6e}"])

print(json.dumps({"w1_mean": w1_mean, "w2_mean": w2_mean, "w3_mean": w3_mean,
                  "stage1_ratio": r1 / r2, "stage2_ratio": r2 / r3}))
