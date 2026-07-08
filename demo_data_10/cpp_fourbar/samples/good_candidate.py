"""A CORRECT turn-1 candidate in a different style: bodies built by a helper, joints from a
spec list, and the drive implemented with the ChLinkMotorRotationAngle alternative (ramp angle,
slope CH_PI) instead of the demo's constant-speed motor: the same physical drive. Same linkage
geometry, so the same loop-closure bands; must score ~100."""
import json
import math

import pychrono as ch

DRIVE_RATE = math.pi
DT = 1e-3
T_STOP = 10.0


def body_at(system, x, y, fixed=False):
    b = ch.ChBody()
    b.SetPos(ch.ChVector3d(x, y, 0))
    b.SetFixed(fixed)
    system.AddBody(b)
    return b


world = ch.ChSystemNSC()
world.SetGravityY()

truss = body_at(world, 0, 0, fixed=True)
crank = body_at(world, 0, 0)
coupler = body_at(world, 4, 0)
rocker = body_at(world, 8, -4)

drive = ch.ChLinkMotorRotationAngle()
drive.Initialize(truss, crank, ch.ChFramed(ch.ChVector3d(0, 0, 0)))
drive.SetAngleFunction(ch.ChFunctionRamp(0, DRIVE_RATE))
world.AddLink(drive)

for b1, b2, px, py in ((crank, coupler, 2, 0), (coupler, rocker, 8, 0), (rocker, truss, 8, -8)):
    j = ch.ChLinkLockRevolute()
    j.Initialize(b1, b2, ch.ChFramed(ch.ChVector3d(px, py, 0)))
    world.AddLink(j)

world.SetTimestepperType(ch.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)


def spin_z(b):
    q = b.GetRot()
    return 2.0 * math.atan2(q.e3, q.e0)


lines = ["t,alpha,wz"]
history = []
k = 0
t = 0.0
while t < T_STOP:
    t = world.GetChTime()
    world.DoStepDynamics(DT)
    k += 1
    if k % 5 == 0:
        rec = (t, spin_z(rocker), crank.GetAngVelLocal().z)
        history.append(rec)
        lines.append(f"{rec[0]:.6f},{rec[1]:.6e},{rec[2]:.6e}")

with open("out.csv", "w") as fh:
    fh.write("\n".join(lines) + "\n")

late = [rec for rec in history if rec[0] >= 2.0]
print(json.dumps({"alpha_min": min(rec[1] for rec in late),
                  "alpha_max": max(rec[1] for rec in late),
                  "wz_mean": sum(rec[2] for rec in late) / len(late)}))
