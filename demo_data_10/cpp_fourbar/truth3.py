"""Four-bar translation, turn 3 (EXTEND) -- PyChrono 10.0, headless -- contracted reference.

Same short-crank linkage as turn 2, driven twice as fast (2*pi rad/s: one crank revolution and
hence one rocker oscillation per second) and INSTRUMENTED one step further: the script now also
logs the coupler midpoint's height, whose vertical excursion the loop-closure oracle puts at
1.0077 (the midpoint of the crank pin, on a unit circle, and the rocker pin, on the swing arc).
The swing amplitude is unchanged (kinematics does not care about speed); the oscillation period
and the drive-rate band halve/double accordingly.
"""
import csv
import json
import math

import pychrono as chrono

CRANK_SPEED = 2 * math.pi
STEP = 1e-3
T_END = 10.0

sysNSC = chrono.ChSystemNSC()
sysNSC.SetGravityY()

body_A = chrono.ChBody()              # truss
body_A.SetFixed(True)
sysNSC.AddBody(body_A)

body_B = chrono.ChBody()              # flywheel / crank
body_B.SetPos(chrono.ChVector3d(0, 0, 0))
sysNSC.AddBody(body_B)

body_C = chrono.ChBody()              # rod / coupler
body_C.SetPos(chrono.ChVector3d(4, 0, 0))
sysNSC.AddBody(body_C)

body_D = chrono.ChBody()              # rocker
body_D.SetPos(chrono.ChVector3d(8, -4, 0))
sysNSC.AddBody(body_D)

link_AB = chrono.ChLinkMotorRotationSpeed()
link_AB.Initialize(body_A, body_B, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
link_AB.SetSpeedFunction(chrono.ChFunctionConst(CRANK_SPEED))
sysNSC.AddLink(link_AB)

link_BC = chrono.ChLinkLockRevolute()
link_BC.Initialize(body_B, body_C, chrono.ChFramed(chrono.ChVector3d(1, 0, 0)))
sysNSC.AddLink(link_BC)

link_CD = chrono.ChLinkLockRevolute()
link_CD.Initialize(body_C, body_D, chrono.ChFramed(chrono.ChVector3d(8, 0, 0)))
sysNSC.AddLink(link_CD)

link_DA = chrono.ChLinkLockRevolute()
link_DA.Initialize(body_D, body_A, chrono.ChFramed(chrono.ChVector3d(8, -8, 0)))
sysNSC.AddLink(link_DA)

sysNSC.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)


def z_rotation(body):
    q = body.GetRot()
    return 2.0 * math.atan2(q.e3, q.e0)


rows = []
t = 0.0
n = 0
while t < T_END:
    t = sysNSC.GetChTime()
    sysNSC.DoStepDynamics(STEP)
    n += 1
    if n % 5 == 0:                    # sample every 5e-3 s
        p1 = body_B.TransformPointLocalToParent(chrono.ChVector3d(1, 0, 0))   # crank pin
        p2 = body_D.TransformPointLocalToParent(chrono.ChVector3d(0, 4, 0))   # rocker pin
        rows.append((t, z_rotation(body_D), body_B.GetAngVelLocal().z, 0.5 * (p1.y + p2.y)))

with open("out.csv", "w", newline="") as fh:
    w = csv.writer(fh)
    w.writerow(["t", "alpha", "wz", "cmy"])
    for r in rows:
        w.writerow([f"{r[0]:.6f}", f"{r[1]:.6e}", f"{r[2]:.6e}", f"{r[3]:.6e}"])

tail = [r for r in rows if r[0] >= 2.0]
print(json.dumps({"alpha_min": min(r[1] for r in tail),
                  "alpha_max": max(r[1] for r in tail),
                  "wz_mean": sum(r[2] for r in tail) / len(tail)}))
