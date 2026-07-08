"""A CORRECT turn-1 candidate in a different style: a generic ChFunction wrapper over lambdas,
links from a spec table, the stage's centimeter values converted once at the top. Same arm,
same imposed trajectory, so the same FK bands; must score ~100."""
import json
import math

import pychrono as ch

CM = 0.01                              # the stage declares metersPerUnit = 0.01
H, L2, L3 = 30 * CM, 40 * CM, 60 * CM
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


column = link(2.0, (0.015, 0.015, 0.0002), 15 * CM)
upper = link(1.2, (0.016, 0.016, 0.00015), H + 20 * CM)
fore = link(0.9, (0.027, 0.027, 0.00012), H + L2 + 30 * CM)

Y_FRAME = ch.QuatFromAngleX(-ch.CH_PI_2)
traj = (lambda t: 0.4 * math.sin(0.2 * math.pi * t),
        lambda t: 0.45 * (1 - math.cos(0.4 * math.pi * t)),
        lambda t: 0.35 * (1 - math.cos(0.4 * math.pi * t)))
spec = ((column, base, ch.ChVector3d(0, 0, 0), ch.QUNIT),
        (upper, column, ch.ChVector3d(0, 0, H), Y_FRAME),
        (fore, upper, ch.ChVector3d(0, 0, H + L2), Y_FRAME))

funcs = []
for (child, parent, at, q), fn in zip(spec, traj):
    m = ch.ChLinkMotorRotationAngle()
    m.Initialize(child, parent, ch.ChFramed(at, q))
    f = FnWrap(fn)
    funcs.append(f)
    m.SetAngleFunction(f)
    world.AddLink(m)

TIP = ch.ChVector3d(0, 0, 30 * CM)
lines = ["t,hx,hy,h,r"]
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

print(json.dumps({"h_final": last[2], "r_final": last[3], "L3": L3, "frame": "z-up"}))
