"""A CORRECT turn-1 candidate in a different style: one generic ChFunction wrapper closing over
lambdas, links built from a spec table, EE tracked through a stored tip offset. Same arm, same
imposed trajectory, so the same FK bands; must score ~100."""
import json
import math

import pychrono as ch

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


world = ch.ChSystemNSC()
world.SetGravitationalAcceleration(ch.ChVector3d(0, 0, -9.81))

base = ch.ChBody()
base.SetFixed(True)
world.AddBody(base)


def link(mass, inertia, com_z):
    b = ch.ChBody()
    b.SetMass(mass)
    b.SetInertiaXX(ch.ChVector3d(*inertia))
    b.SetPos(ch.ChVector3d(0, 0, com_z))
    world.AddBody(b)
    return b


column = link(2.0, (0.0266667, 0.0266667, 0.002), 0.2)
upper = link(1.0, (0.0208333, 0.0208333, 0.0005), 0.65)
fore = link(1.0, (0.0208333, 0.0208333, 0.0005), 1.15)

Y_FRAME = ch.QuatFromAngleX(-ch.CH_PI_2)
traj = (lambda t: 0.5 * math.sin(0.2 * math.pi * t),
        lambda t: 0.35 * (1 - math.cos(0.4 * math.pi * t)),
        lambda t: -0.5 * (1 - math.cos(0.4 * math.pi * t)))
spec = ((column, base, ch.ChVector3d(0, 0, 0), ch.QUNIT),
        (upper, column, ch.ChVector3d(0, 0, 0.4), Y_FRAME),
        (fore, upper, ch.ChVector3d(0, 0, 0.9), Y_FRAME))

funcs = []
for (child, parent, at, q), fn in zip(spec, traj):
    m = ch.ChLinkMotorRotationAngle()
    m.Initialize(child, parent, ch.ChFramed(at, q))
    f = FnWrap(fn)
    funcs.append(f)
    m.SetAngleFunction(f)
    world.AddLink(m)

TIP = ch.ChVector3d(0, 0, 0.25)
lines = ["t,x,y,z,r"]
last = None
t = 0.0
k = 0
while t < T_STOP:
    t = world.GetChTime()
    world.DoStepDynamics(DT)
    k += 1
    if k % 5 == 0:
        p = fore.TransformPointLocalToParent(TIP)
        last = (p.x, p.y, p.z, math.hypot(p.x, p.y))
        lines.append(f"{t:.6f},{p.x:.6e},{p.y:.6e},{p.z:.6e},{last[3]:.6e}")

with open("out.csv", "w") as fh:
    fh.write("\n".join(lines) + "\n")

print(json.dumps({"z_final": last[2], "r_final": last[3], "L2": 0.5, "rate_scale": 1.0}))
