"""Isaac Sim USD articulation converted to PyChrono, turn 1 (CONVERT) -- PyChrono 10.0,
headless -- contracted reference.

Hand conversion of source/crank_pendulum_v1.usda (a declarative Isaac-idiom stage: UsdPhysics
revolute joints with per-body localPos frames, MassAPI mass/COM/diagonalInertia, PhysX DriveAPI):
a crank rod (L1 = 0.4, m = 0.5) whose velocity-mode drive (stiffness 0) targets 85.9437 deg/s =
1.5 rad/s (the USD Physics schema authors angular drives in DEGREES), soft-started over 0.5 s
per the controller timeline the prompt decrees (declarative stages carry no time-varying
retargets); and a pendulum rod (L2 = 0.5, m = 0.3) on a free revolute at the crank tip whose
damping-only DriveAPI is a native viscous damper (0.05 N m s/rad in SI) -> a ChLinkRSDA.
Physics is the pyb_arm_motor matched triple: RK4 oracle tail 0.2015, run max 0.3057.
"""
import csv
import json
import math

import pychrono as chrono

L1, L2 = 0.4, 0.5
M1, M2 = 0.5, 0.3
OMEGA = 1.5
B_ELBOW = 0.05
T_RAMP = 0.5
MOUNT = chrono.ChVector3d(0, 0, 1.5)
STEP = 1e-3
T_END = 10.0


class DriveSpeed(chrono.ChFunction):
    """Soft-started velocity target, as the PyBullet loop retargets its motor."""

    def GetVal(self, t):
        return OMEGA * min(t / T_RAMP, 1.0)

    def Clone(self):
        return DriveSpeed()

sysNSC = chrono.ChSystemNSC()
sysNSC.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

mount = chrono.ChBody()
mount.SetFixed(True)
sysNSC.AddBody(mount)

crank = chrono.ChBody()
crank.SetMass(M1)
crank.SetInertiaXX(chrono.ChVector3d(M1 * L1 ** 2 / 12, M1 * L1 ** 2 / 12, 1e-4))
crank.SetPos(MOUNT + chrono.ChVector3d(0, 0, -L1 / 2))
sysNSC.AddBody(crank)

pend = chrono.ChBody()
pend.SetMass(M2)
pend.SetInertiaXX(chrono.ChVector3d(M2 * L2 ** 2 / 12, M2 * L2 ** 2 / 12, 1e-4))
pend.SetPos(MOUNT + chrono.ChVector3d(0, 0, -L1 - L2 / 2))
sysNSC.AddBody(pend)

# both joints rotate about global y: rotate each frame's z onto y
y_axis = chrono.QuatFromAngleX(-chrono.CH_PI_2)

drive = chrono.ChLinkMotorRotationSpeed()
drive.Initialize(crank, mount, chrono.ChFramed(MOUNT, y_axis))
speed_fun = DriveSpeed()
drive.SetSpeedFunction(speed_fun)
sysNSC.AddLink(drive)

elbow = chrono.ChLinkLockRevolute()                    # free: no motor command needed in Chrono
elbow.Initialize(crank, pend, chrono.ChFramed(MOUNT + chrono.ChVector3d(0, 0, -L1), y_axis))
sysNSC.AddLink(elbow)

damper = chrono.ChLinkRSDA()                           # the PyBullet jointDamping equivalent
damper.Initialize(crank, pend, chrono.ChFramed(MOUNT + chrono.ChVector3d(0, 0, -L1), y_axis))
damper.SetDampingCoefficient(B_ELBOW)
sysNSC.AddLink(damper)


def abs_angle(body):
    q = body.GetRot()
    return 2.0 * math.atan2(q.e2, q.e0)


rows = []
t = 0.0
while t < T_END:
    t = sysNSC.GetChTime()
    sysNSC.DoStepDynamics(STEP)
    rows.append((t, crank.GetAngVelLocal().y, abs_angle(pend)))

with open("out.csv", "w", newline="") as fh:
    w = csv.writer(fh)
    w.writerow(["t", "w", "theta"])
    for r in rows:
        w.writerow([f"{r[0]:.6f}", f"{r[1]:.6e}", f"{r[2]:.6e}"])

print(json.dumps({"theta_max": max(abs(r[2]) for r in rows),
                  "omega_drive": OMEGA}))
