"""Four-bar C++ demo translated to PyChrono, turn 1 (TRANSLATE) -- PyChrono 10.0, headless --
contracted reference.

Faithful translation of source/demo_MBS_fourbar.cpp (projectchrono/chrono, src/demos/mbs): a
crank-rocker four-bar (crank pivot at the origin, pin at (2,0); coupler to (8,0); rocker to the
ground pivot (8,-8)), the crank driven at a constant CH_PI rad/s by a ChLinkMotorRotationSpeed,
gravity in -Y, EULER_IMPLICIT_LINEARIZED at dt = 1e-3; all Irrlicht/GUI code dropped. The demo
starts at the extended crank-coupler collinear configuration, so the rocker begins AT one swing
extreme: its rotation alpha oscillates in [0, 0.5494] rad (independent loop-closure oracle),
once per crank revolution (2 s). Logs t, the rocker rotation about z, and the crank's angular
velocity; the judge derives swing extremes, oscillation period, and drive rate from the CSV.
"""
import csv
import json
import math

import pychrono as chrono

CRANK_SPEED = math.pi
STEP = 1e-3
T_END = 10.0

sysNSC = chrono.ChSystemNSC()
sysNSC.SetGravityY()

body_A = chrono.ChBody()              # truss
body_A.SetFixed(True)
sysNSC.AddBody(body_A)

body_B = chrono.ChBody()              # flywheel / crank
body_B.SetPos(chrono.ChVector3d(0, 0, 0))
sysNSC.AddBody(body_B)

body_C = chrono.ChBody()              # rod / coupler
body_C.SetPos(chrono.ChVector3d(4, 0, 0))
sysNSC.AddBody(body_C)

body_D = chrono.ChBody()              # rocker
body_D.SetPos(chrono.ChVector3d(8, -4, 0))
sysNSC.AddBody(body_D)

link_AB = chrono.ChLinkMotorRotationSpeed()
link_AB.Initialize(body_A, body_B, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
link_AB.SetSpeedFunction(chrono.ChFunctionConst(CRANK_SPEED))
sysNSC.AddLink(link_AB)

link_BC = chrono.ChLinkLockRevolute()
link_BC.Initialize(body_B, body_C, chrono.ChFramed(chrono.ChVector3d(2, 0, 0)))
sysNSC.AddLink(link_BC)

link_CD = chrono.ChLinkLockRevolute()
link_CD.Initialize(body_C, body_D, chrono.ChFramed(chrono.ChVector3d(8, 0, 0)))
sysNSC.AddLink(link_CD)

link_DA = chrono.ChLinkLockRevolute()
link_DA.Initialize(body_D, body_A, chrono.ChFramed(chrono.ChVector3d(8, -8, 0)))
sysNSC.AddLink(link_DA)

sysNSC.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)


def z_rotation(body):
    q = body.GetRot()
    return 2.0 * math.atan2(q.e3, q.e0)


rows = []
t = 0.0
n = 0
while t < T_END:
    t = sysNSC.GetChTime()
    sysNSC.DoStepDynamics(STEP)
    n += 1
    if n % 5 == 0:                    # sample every 5e-3 s
        rows.append((t, z_rotation(body_D), body_B.GetAngVelLocal().z))

with open("out.csv", "w", newline="") as fh:
    w = csv.writer(fh)
    w.writerow(["t", "alpha", "wz"])
    for r in rows:
        w.writerow([f"{r[0]:.6f}", f"{r[1]:.6e}", f"{r[2]:.6e}"])

tail = [r for r in rows if r[0] >= 2.0]
print(json.dumps({"alpha_min": min(r[1] for r in tail),
                  "alpha_max": max(r[1] for r in tail),
                  "wz_mean": sum(r[2] for r in tail) / len(tail)}))
