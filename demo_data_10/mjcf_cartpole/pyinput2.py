"""Cart-pole MJCF converted to PyChrono, turn 1 (CONVERT) -- PyChrono 10.0, headless --
contracted reference.

Hand conversion of source/cartpole_v1.xml (PyChrono has no MJCF importer, so the semantics are
translated: MuJoCo's z-up world and gravity, the slide joint becomes a prismatic joint along x,
the hinge becomes a revolute about y at the cart origin, and the explicit inertial blocks map to
SetMass/SetInertiaXX in body frames). Pole (uniform rod, m = 0.5, hinge-to-COM d = 0.5,
I_hinge = 0.1667) hangs under a FREE cart (M = 2) and is released from 0.15 rad. The coupled
small-oscillation period with cart recoil is T = 1.5101 s (oracle closed form); the system COM
x is conserved (zero initial horizontal momentum) while cart and pole counter-oscillate.
"""
import csv
import json
import math

import pychrono as chrono

M_CART = 2.0
M_POLE = 0.5
D = 0.5
THETA0 = 0.15
STEP = 1e-3
T_END = 10.0

sysNSC = chrono.ChSystemNSC()
sysNSC.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

ground = chrono.ChBody()
ground.SetFixed(True)
sysNSC.AddBody(ground)

cart = chrono.ChBody()
cart.SetMass(M_CART)
cart.SetInertiaXX(chrono.ChVector3d(0.05, 0.05, 0.05))
cart.SetPos(chrono.ChVector3d(0, 0, 1))
sysNSC.AddBody(cart)

# slide joint along x: ChLinkLockPrismatic slides along its frame's Z axis, so rotate z -> x
prismatic = chrono.ChLinkLockPrismatic()
prismatic.Initialize(ground, cart, chrono.ChFramed(chrono.ChVector3d(0, 0, 1),
                                                   chrono.QuatFromAngleY(chrono.CH_PI_2)))
sysNSC.AddLink(prismatic)

# pole: uniform rod hanging from the hinge at the cart origin, tilted THETA0 about +y
tilt = chrono.QuatFromAngleY(THETA0)
com_local = chrono.ChVector3d(0, 0, -D)
com_world = chrono.ChVector3d(0, 0, 1) + tilt.Rotate(com_local)

pole = chrono.ChBody()
pole.SetMass(M_POLE)
pole.SetInertiaXX(chrono.ChVector3d(0.0416667, 0.0416667, 0.0001))
pole.SetPos(com_world)
pole.SetRot(tilt)
sysNSC.AddBody(pole)

# hinge about y at the cart origin: ChLinkLockRevolute rotates about its frame's Z, so x-rotate
hinge = chrono.ChLinkLockRevolute()
hinge.Initialize(cart, pole, chrono.ChFramed(chrono.ChVector3d(0, 0, 1),
                                             chrono.QuatFromAngleX(-chrono.CH_PI_2)))
sysNSC.AddLink(hinge)


def pole_angle():
    q = pole.GetRot()
    return 2.0 * math.atan2(q.e2, q.e0)


rows = []
t = 0.0
while t < T_END:
    t = sysNSC.GetChTime()
    sysNSC.DoStepDynamics(STEP)
    xc = cart.GetPos().x
    xp = pole.GetPos().x
    xcom = (M_CART * xc + M_POLE * xp) / (M_CART + M_POLE)
    rows.append((t, xc, pole_angle(), xcom))

with open("out.csv", "w", newline="") as fh:
    w = csv.writer(fh)
    w.writerow(["t", "x", "theta", "xcom"])
    for r in rows:
        w.writerow([f"{r[0]:.6f}", f"{r[1]:.6e}", f"{r[2]:.6e}", f"{r[3]:.6e}"])

tail = [r for r in rows if r[0] >= 1.0]
print(json.dumps({"theta_amp": max(abs(r[2]) for r in tail),
                  "cart_mass": M_CART, "mode": "free-cart"}))
