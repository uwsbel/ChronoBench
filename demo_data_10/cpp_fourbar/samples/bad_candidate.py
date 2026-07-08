"""A RUNS-BUT-WRONG turn-1 candidate: structurally complete and it executes cleanly (passes L1
and the minimal L2 capability checks), but the motor constant was mis-transcribed in
translation: the C++ demo's CH_PI became CH_PI_2 (pi/2). The linkage geometry is right, so the
swing amplitude is still correct (kinematics does not depend on speed), but the crank turns at
half rate: the drive-rate band [-3.30, -2.99] fails at -1.5708 and the wrong-physics cap
applies. Exactly the class of slip a static judge cannot see."""
import csv
import json
import math

import pychrono as chrono

CRANK_SPEED = math.pi / 2    # WRONG: the C++ demo's CH_PI mis-copied as CH_PI_2
STEP = 1e-3
T_END = 10.0

sysNSC = chrono.ChSystemNSC()
sysNSC.SetGravityY()

body_A = chrono.ChBody()
body_A.SetFixed(True)
sysNSC.AddBody(body_A)

body_B = chrono.ChBody()
body_B.SetPos(chrono.ChVector3d(0, 0, 0))
sysNSC.AddBody(body_B)

body_C = chrono.ChBody()
body_C.SetPos(chrono.ChVector3d(4, 0, 0))
sysNSC.AddBody(body_C)

body_D = chrono.ChBody()
body_D.SetPos(chrono.ChVector3d(8, -4, 0))
sysNSC.AddBody(body_D)

link_AB = chrono.ChLinkMotorRotationSpeed()
link_AB.Initialize(body_A, body_B, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
link_AB.SetSpeedFunction(chrono.ChFunctionConst(CRANK_SPEED))
sysNSC.AddLink(link_AB)

link_BC = chrono.ChLinkLockRevolute()
link_BC.Initialize(body_B, body_C, chrono.ChFramed(chrono.ChVector3d(2, 0, 0)))
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
    if n % 5 == 0:
        rows.append((t, z_rotation(body_D), body_B.GetAngVelLocal().z))

with open("out.csv", "w", newline="") as fh:
    w = csv.writer(fh)
    w.writerow(["t", "alpha", "wz"])
    for r in rows:
        w.writerow([f"{r[0]:.6f}", f"{r[1]:.6e}", f"{r[2]:.6e}"])

tail = [r for r in rows if r[0] >= 2.0]
print(json.dumps({"alpha_min": min(r[1] for r in tail),
                  "alpha_max": max(r[1] for r in tail),
                  "wz_mean": sum(r[2] for r in tail) / len(tail)}))
