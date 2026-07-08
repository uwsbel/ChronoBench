"""A CORRECT turn-1 candidate in a different style: a rod factory returning (body, tip), the
energy accumulated with a generator expression, manual CSV lines. Same rods, nesting, hinges,
and release, so the same RK4-anchored bands; must score ~100."""
import json
import math

import pychrono as ch

GRAV = 9.81
DT = 1e-3
T_STOP = 10.0
TOP = ch.ChVector3d(0, 0, 2)


def make_rod(world, mass, length, hinge_point, tilt_rad):
    q = ch.QuatFromAngleY(tilt_rad)
    body = ch.ChBody()
    body.SetMass(mass)
    ivy = mass * length ** 2 / 12
    body.SetInertiaXX(ch.ChVector3d(ivy, ivy, 0.0001))
    body.SetPos(hinge_point + q.Rotate(ch.ChVector3d(0, 0, -length / 2)))
    body.SetRot(q)
    world.AddBody(body)
    tip = hinge_point + q.Rotate(ch.ChVector3d(0, 0, -length))
    return body, tip


world = ch.ChSystemNSC()
world.SetGravitationalAcceleration(ch.ChVector3d(0, 0, -GRAV))

base = ch.ChBody()
base.SetFixed(True)
world.AddBody(base)

upper, upper_tip = make_rod(world, 1.0, 1.0, TOP, 0.1)
lower, _ = make_rod(world, 1.0, 1.0, upper_tip, 0.0)

j1 = ch.ChLinkLockRevolute()
j1.Initialize(base, upper, ch.ChFramed(TOP, ch.QuatFromAngleX(-ch.CH_PI_2)))
world.AddLink(j1)

j2 = ch.ChLinkLockRevolute()
j2.Initialize(upper, lower, ch.ChFramed(upper_tip, ch.QuatFromAngleX(-ch.CH_PI_2)))
world.AddLink(j2)


def angle_y(b):
    q = b.GetRot()
    return 2.0 * math.atan2(q.e2, q.e0)


def energy():
    total = 0.0
    for b in (upper, lower):
        v, w, ine = b.GetPosDt(), b.GetAngVelLocal(), b.GetInertiaXX()
        total += 0.5 * b.GetMass() * v.Length2()
        total += 0.5 * (ine.x * w.x ** 2 + ine.y * w.y ** 2 + ine.z * w.z ** 2)
        total += b.GetMass() * GRAV * b.GetPos().z
    return total


lines = ["t,th1,th2,rel,e"]
peaks = [0.0, 0.0]
t = 0.0
while t < T_STOP:
    t = world.GetChTime()
    world.DoStepDynamics(DT)
    a1, a2 = angle_y(upper), angle_y(lower)
    peaks[0] = max(peaks[0], abs(a1))
    peaks[1] = max(peaks[1], abs(a2))
    lines.append(f"{t:.6f},{a1:.6e},{a2:.6e},{a2 - a1:.6e},{energy():.6e}")

with open("out.csv", "w") as fh:
    fh.write("\n".join(lines) + "\n")

print(json.dumps({"theta1_max": peaks[0], "theta2_max": peaks[1],
                  "config": "equal-rods-straight-release"}))
