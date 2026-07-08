"""Cart-pole conversion, turn 3 (EXTEND) -- PyChrono 10.0, headless -- contracted reference.

The MJCF loses its slide joint (source/cartpole_v3.xml): a MuJoCo body with NO joint is welded
to its parent, so the cart becomes part of the world and the model degenerates to a pure
compound pendulum. In the conversion that means fixing the cart (no prismatic joint). The
recoil term vanishes (M -> infinity): oracle period 1.6379 s, the plain m g d / I_hinge value
(and exactly the import_urdf free-swing period, a cross-task consistency anchor). A candidate
whose cart still slides oscillates at 2.0 kg-cart speed (1.51 s) and fails; the new locked-cart
invariant (cart x never moves) pins the joint removal.
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
cart.SetFixed(True)                     # no slide joint in the MJCF: the cart is world-welded
sysNSC.AddBody(cart)

tilt = chrono.QuatFromAngleY(THETA0)
com_local = chrono.ChVector3d(0, 0, -D)
com_world = chrono.ChVector3d(0, 0, 1) + tilt.Rotate(com_local)

pole = chrono.ChBody()
pole.SetMass(M_POLE)
pole.SetInertiaXX(chrono.ChVector3d(0.0416667, 0.0416667, 0.0001))
pole.SetPos(com_world)
pole.SetRot(tilt)
sysNSC.AddBody(pole)

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
                  "cart_mass": M_CART, "mode": "locked-cart"}))
