"""PyBullet arm conversion, turn 3 (EXTEND) -- PyChrono 10.0, headless -- contracted reference.

Same 2.0 rad/s arm as turn 2, but the source (source/pyb_arm_v3.py) RETARGETS the velocity
motor back to zero at t = 5 s (a 0.5 s down-ramp; the setJointMotorControl2 idiom). The braked
crank holds still and the pendulum rings down FREELY through its elbow damper: the tail
amplitude collapses to 0.0434 rad (an un-braked candidate at 0.3267 fails), the crank rate in
the tail is zero, and the ringing period is the closed-form damped compound-pendulum value
1.1785 s (= 1.1582 / sqrt(1 - zeta^2)), all from the RK4 oracle and matched by the PyBullet
source to 4 decimals.
"""
import csv
import json
import math

import pychrono as chrono

L1, L2 = 0.4, 0.5
M1, M2 = 0.5, 0.3
OMEGA = 2.0
B_ELBOW = 0.05
T_RAMP = 0.5
MOUNT = chrono.ChVector3d(0, 0, 1.5)
STEP = 1e-3
T_END = 10.0


T_BRAKE = 5.0


class DriveSpeed(chrono.ChFunction):
    """Soft-started velocity target, ramped back to zero at T_BRAKE (motor retargeting)."""

    def GetVal(self, t):
        if t < T_BRAKE:
            return OMEGA * min(t / T_RAMP, 1.0)
        return OMEGA * max(1.0 - (t - T_BRAKE) / T_RAMP, 0.0)

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
