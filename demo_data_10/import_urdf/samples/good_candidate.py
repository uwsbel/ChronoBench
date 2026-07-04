"""A CORRECT-BUT-DIFFERENT turn-1 URDF import: same physics, different style. Steps by an integer
count instead of a time comparison, logs with plain file writes, tracks the amplitude with a
running maximum, and reads the arm rotation from the quaternion it fetches once per loop through a
helper. Should pass L1/L2/L3 near ceiling."""
import json
import math

import pychrono as chrono
import pychrono.parsers as parsers

AMPLITUDE = 0.5
FREQ_HZ = 1.0
DT = 1.0e-3
N_STEPS = 3000


def planar_angle_z(body):
    q = body.GetRot()
    return 2.0 * math.atan2(q.e3, q.e0)


world = chrono.ChSystemSMC()
world.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

urdf = parsers.ChParserURDF("pendulum.urdf")
urdf.SetRootInitPose(chrono.ChFramed(chrono.VNULL, chrono.QUNIT))
urdf.SetAllJointsActuationType(parsers.ChParserURDF.ActuationType_POSITION)
urdf.PopulateSystem(world)
urdf.GetRootChBody().SetFixed(True)
urdf.SetMotorFunction("swing", chrono.ChFunctionSine(AMPLITUDE, FREQ_HZ))

arm_body = urdf.GetChBody("arm")

peak = 0.0
with open("out.csv", "w") as fh:
    fh.write("t,theta\n")
    for _ in range(N_STEPS):
        world.DoStepDynamics(DT)
        angle = planar_angle_z(arm_body)
        peak = max(peak, abs(angle))
        fh.write(f"{world.GetChTime():.6f},{angle:.6e}\n")

print(json.dumps({"amplitude": peak, "A": AMPLITUDE, "f": FREQ_HZ}))
