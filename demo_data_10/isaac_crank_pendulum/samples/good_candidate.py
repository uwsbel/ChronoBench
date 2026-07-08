"""A CORRECT turn-1 candidate in a different style: bodies from a rod factory, the drive ramp
as a closure-wrapped generic ChFunction, the damper configured alongside the joint in one
helper. Same articulation as the USD stage (mount at (0,0,1.5), crank on the velocity drive at
1.5 rad/s = the stage's 85.9437 deg/s, damped free pendulum at the crank tip), so the same
matched-triple bands; must score ~100."""
import json
import math

import pychrono as ch

CRANK_L, PEND_L = 0.4, 0.5
CRANK_M, PEND_M = 0.5, 0.3
TARGET = 1.5
RAMP = 0.5
DAMP = 0.05
TOP = ch.ChVector3d(0, 0, 1.5)
DT = 1e-3
T_STOP = 10.0


class FnWrap(ch.ChFunction):
    def __init__(self, fn):
        super().__init__()
        self.fn = fn

    def GetVal(self, x):
        return self.fn(x)

    def Clone(self):
        return FnWrap(self.fn)


def rod(world, mass, length, top_point):
    b = ch.ChBody()
    b.SetMass(mass)
    iyy = mass * length ** 2 / 12
    b.SetInertiaXX(ch.ChVector3d(iyy, iyy, 1e-4))
    b.SetPos(top_point + ch.ChVector3d(0, 0, -length / 2))
    world.AddBody(b)
    return b


world = ch.ChSystemNSC()
world.SetGravitationalAcceleration(ch.ChVector3d(0, 0, -9.81))

base = ch.ChBody()
base.SetFixed(True)
world.AddBody(base)

crank = rod(world, CRANK_M, CRANK_L, TOP)
pend = rod(world, PEND_M, PEND_L, TOP + ch.ChVector3d(0, 0, -CRANK_L))

Y_FRAME = ch.QuatFromAngleX(-ch.CH_PI_2)
elbow_at = TOP + ch.ChVector3d(0, 0, -CRANK_L)

spinner = ch.ChLinkMotorRotationSpeed()
spinner.Initialize(crank, base, ch.ChFramed(TOP, Y_FRAME))
ramp_fun = FnWrap(lambda t: TARGET * min(t / RAMP, 1.0))
spinner.SetSpeedFunction(ramp_fun)
world.AddLink(spinner)

swing = ch.ChLinkLockRevolute()
swing.Initialize(crank, pend, ch.ChFramed(elbow_at, Y_FRAME))
world.AddLink(swing)

drag = ch.ChLinkRSDA()
drag.Initialize(crank, pend, ch.ChFramed(elbow_at, Y_FRAME))
drag.SetDampingCoefficient(DAMP)
world.AddLink(drag)

lines = ["t,w,theta"]
peak = 0.0
t = 0.0
while t < T_STOP:
    t = world.GetChTime()
    world.DoStepDynamics(DT)
    q = pend.GetRot()
    ang = 2.0 * math.atan2(q.e2, q.e0)
    peak = max(peak, abs(ang))
    lines.append(f"{t:.6f},{crank.GetAngVelLocal().y:.6e},{ang:.6e}")

with open("out.csv", "w") as fh:
    fh.write("\n".join(lines) + "\n")

print(json.dumps({"theta_max": peak, "omega_drive": TARGET}))
