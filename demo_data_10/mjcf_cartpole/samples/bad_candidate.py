"""A RUNS-BUT-WRONG turn-1 candidate: structurally complete and it executes cleanly (passes L1
and the minimal L2 capability checks), but the MJCF inertial COM OFFSET was dropped in
conversion: the pole body sits AT the hinge (ignoring inertial pos "0 0 -0.5"), so the rod is
pivoted through its center of mass. Gravity then exerts no torque about the hinge: the pole
just holds its 0.15 rad tilt and never oscillates, the period derive returns NaN, and the
wrong-physics cap applies. The classic declarative-to-imperative conversion slip."""
import csv
import json
import math

import pychrono as chrono

M_CART = 2.0
M_POLE = 0.5
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

prismatic = chrono.ChLinkLockPrismatic()
prismatic.Initialize(ground, cart, chrono.ChFramed(chrono.ChVector3d(0, 0, 1),
                                                   chrono.QuatFromAngleY(chrono.CH_PI_2)))
sysNSC.AddLink(prismatic)

tilt = chrono.QuatFromAngleY(THETA0)

pole = chrono.ChBody()
pole.SetMass(M_POLE)
pole.SetInertiaXX(chrono.ChVector3d(0.0416667, 0.0416667, 0.0001))
pole.SetPos(chrono.ChVector3d(0, 0, 1))    # WRONG: inertial pos "0 0 -0.5" ignored, COM at hinge
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
                  "cart_mass": M_CART, "mode": "free-cart"}))
