"""A CORRECT turn-1 candidate in a different style: a body factory, joint frames prebuilt as
named variables, the pole angle read through GetRotAngle-style math inline, manual CSV lines.
Same masses, joints, and release, so the same coupled-oscillation bands; must score ~100."""
import json
import math

import pychrono as ch

CART_M = 2.0
POLE_M = 0.5
OFFSET = 0.5
TILT0 = 0.15
DT = 1e-3
T_STOP = 10.0


def rigid_body(mass, inertia, pos, rot=None, fixed=False):
    b = ch.ChBody()
    b.SetMass(mass)
    b.SetInertiaXX(ch.ChVector3d(*inertia))
    b.SetPos(pos)
    if rot is not None:
        b.SetRot(rot)
    b.SetFixed(fixed)
    return b


world = ch.ChSystemNSC()
world.SetGravitationalAcceleration(ch.ChVector3d(0, 0, -9.81))

anchor = rigid_body(1.0, (1, 1, 1), ch.ChVector3d(0, 0, 0), fixed=True)
world.AddBody(anchor)

trolley = rigid_body(CART_M, (0.05, 0.05, 0.05), ch.ChVector3d(0, 0, 1))
world.AddBody(trolley)

lean = ch.QuatFromAngleY(TILT0)
rod_com = ch.ChVector3d(0, 0, 1) + lean.Rotate(ch.ChVector3d(0, 0, -OFFSET))
rod = rigid_body(POLE_M, (0.0416667, 0.0416667, 0.0001), rod_com, rot=lean)
world.AddBody(rod)

slide_frame = ch.ChFramed(ch.ChVector3d(0, 0, 1), ch.QuatFromAngleY(ch.CH_PI_2))
slider = ch.ChLinkLockPrismatic()
slider.Initialize(anchor, trolley, slide_frame)
world.AddLink(slider)

hinge_frame = ch.ChFramed(ch.ChVector3d(0, 0, 1), ch.QuatFromAngleX(-ch.CH_PI_2))
swivel = ch.ChLinkLockRevolute()
swivel.Initialize(trolley, rod, hinge_frame)
world.AddLink(swivel)

lines = ["t,x,theta,xcom"]
history = []
t = 0.0
while t < T_STOP:
    t = world.GetChTime()
    world.DoStepDynamics(DT)
    q = rod.GetRot()
    ang = 2.0 * math.atan2(q.e2, q.e0)
    xc = trolley.GetPos().x
    xmix = (CART_M * xc + POLE_M * rod.GetPos().x) / (CART_M + POLE_M)
    history.append((t, ang))
    lines.append(f"{t:.6f},{xc:.6e},{ang:.6e},{xmix:.6e}")

with open("out.csv", "w") as fh:
    fh.write("\n".join(lines) + "\n")

late = [rec for rec in history if rec[0] >= 1.0]
print(json.dumps({"theta_amp": max(abs(rec[1]) for rec in late),
                  "cart_mass": CART_M, "mode": "free-cart"}))
