"""A CORRECT turn-1 candidate in a different style: builds the shafts through a helper, uses
the ChLinkMotorRotationSpeed alternative that the C++ demo's own comment suggests (constant
speed 1 rad/s instead of a ramp angle), different names, manual CSV assembly. Same mechanism
and the same body ordering, so the same signed Cardan bands; must score ~100."""
import json
import math

import pychrono as ch

HALF_LEN = 2.0
BEND = math.pi / 6
DRIVE = 1.0
DT = 5e-3
T_STOP = 10.0


def make_shaft(system, pos, quat):
    b = ch.ChBody()
    b.SetFixed(False)
    b.EnableCollision(False)
    b.SetMass(1)
    b.SetInertiaXX(ch.ChVector3d(1, 1, 0.2))
    b.SetPos(pos)
    b.SetRot(quat)
    system.AddBody(b)
    return b


world = ch.ChSystemNSC()
world.SetGravitationalAcceleration(ch.ChVector3d(0, 0, 0))

tilt = ch.QuatFromAngleX(BEND)
ca, sa = math.cos(BEND), math.sin(BEND)

base = ch.ChBody()
base.SetFixed(True)
base.EnableCollision(False)
world.AddBody(base)

drive_shaft = make_shaft(world, ch.ChVector3d(0, 0, -HALF_LEN), ch.ChQuaterniond(1, 0, 0, 0))
driven_shaft = make_shaft(world, ch.ChVector3d(0, -HALF_LEN * sa, HALF_LEN * ca), tilt)

spinner = ch.ChLinkMotorRotationSpeed()
spinner.Initialize(base, drive_shaft, ch.ChFramed(ch.ChVector3d(0, 0, -HALF_LEN), ch.ChQuaterniond(1, 0, 0, 0)))
spinner.SetSpeedFunction(ch.ChFunctionConst(DRIVE))
world.AddLink(spinner)

sleeve = ch.ChLinkLockCylindrical()
sleeve.Initialize(base, driven_shaft, ch.ChFramed(ch.ChVector3d(0, -HALF_LEN * sa, HALF_LEN * ca), tilt))
world.AddLink(sleeve)

cardan = ch.ChLinkUniversal()
cardan.Initialize(drive_shaft, driven_shaft, ch.ChFramed(ch.ChVector3d(0, 0, 0), tilt))
world.AddLink(cardan)

lines = ["t,w1,w2"]
history = []
t = 0.0
while t < T_STOP:
    t = world.GetChTime()
    world.DoStepDynamics(DT)
    rec = (t, drive_shaft.GetAngVelLocal().z, driven_shaft.GetAngVelLocal().z)
    history.append(rec)
    lines.append(f"{rec[0]:.6f},{rec[1]:.6e},{rec[2]:.6e}")

with open("out.csv", "w") as fh:
    fh.write("\n".join(lines) + "\n")

late = [rec for rec in history if rec[0] >= 2.0]
print(json.dumps({"omega2_min": min(rec[2] for rec in late),
                  "omega2_max": max(rec[2] for rec in late),
                  "omega1_mean": sum(rec[1] for rec in late) / len(late)}))
