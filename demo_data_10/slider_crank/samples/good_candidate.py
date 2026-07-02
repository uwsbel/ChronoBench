"""A CORRECT turn-1 inline slider-crank in a deliberately different style (crank centered at the origin,
different naming/build order). Same kinematics -> should PASS. Shows the judge does not penalize stylistic
divergence (and that the stroke is independent of where the mechanism sits)."""
import csv
import json

import pychrono.core as chrono

R, LROD, OMEGA = 0.4, 1.5, chrono.CH_PI
center = chrono.ChVector3d(0, 0, 0)
tend, step = 3.0, 1.0e-3

world = chrono.ChSystemNSC()
base = chrono.ChBodyEasyBox(3, 1, 3, 1000)
base.SetPos(chrono.ChVector3d(0, -0.5, 0))
base.SetFixed(True)
world.Add(base)

disk = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, R, 0.1, 1000)
disk.SetPos(center + chrono.ChVector3d(0, 0, -0.1))
disk.SetRot(chrono.Q_ROTATE_Y_TO_Z)
world.Add(disk)

conrod = chrono.ChBodyEasyBox(LROD, 0.1, 0.1, 1000)
conrod.SetPos(center + chrono.ChVector3d(R + LROD / 2, 0, 0))
world.Add(conrod)

slug = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.2, 0.3, 1000)
slug.SetPos(center + chrono.ChVector3d(R + LROD, 0, 0))
slug.SetRot(chrono.Q_ROTATE_Y_TO_X)
world.Add(slug)

drive = chrono.ChLinkMotorRotationSpeed()
drive.Initialize(disk, base, chrono.ChFramed(center))
drive.SetMotorFunction(chrono.ChFunctionConst(OMEGA))
world.Add(drive)

p1 = chrono.ChLinkLockRevolute()
p1.Initialize(conrod, disk, chrono.ChFramed(center + chrono.ChVector3d(R, 0, 0)))
world.Add(p1)
p2 = chrono.ChLinkLockRevolute()
p2.Initialize(slug, conrod, chrono.ChFramed(center + chrono.ChVector3d(R + LROD, 0, 0)))
world.Add(p2)
slide = chrono.ChLinkLockPrismatic()
slide.Initialize(slug, base, chrono.ChFramed(center + chrono.ChVector3d(R + LROD, 0, 0), chrono.Q_ROTATE_Z_TO_X))
world.Add(slide)

ts, xs = [], []
while world.GetChTime() < tend:
    world.DoStepDynamics(step)
    ts.append(world.GetChTime())
    xs.append(slug.GetPos().x)

with open("out.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["t", "piston_x"])
    for t, x in zip(ts, xs):
        w.writerow([f"{t:.6f}", f"{x:.6e}"])
print(json.dumps({"stroke": max(xs) - min(xs), "crank_rad": R}))
