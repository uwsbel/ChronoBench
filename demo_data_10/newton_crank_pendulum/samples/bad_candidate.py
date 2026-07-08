"""A RUNS-BUT-WRONG turn-1 candidate: structurally complete and it executes cleanly (passes L1
and the minimal L2 capability checks: motor, free revolute, ChLinkRSDA all present), but the
Newton builder's per-body COM offset was dropped in conversion: add_link placed the pendulum
body at (0,0,1.1) with com=(0,0,-0.25) INSIDE the body frame, and this candidate put the Chrono
body (whose origin IS its COM) at the joint instead. A pendulum whose mass sits at its hinge
has no gravity lever arm: only the viscous damper acts, dragging the pendulum around WITH the
whirling crank until its absolute angle winds to the quaternion wrap boundary: theta_max
saturates at 2 pi instead of 0.31, failing the swing bands decisively. The imperative twin of the MJCF inertial-pos
trap."""
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
pend.SetPos(MOUNT + chrono.ChVector3d(0, 0, -L1))   # WRONG: com offset dropped, body at the joint
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
