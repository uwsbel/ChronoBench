"""A CORRECT pendulum, written in a deliberately different style from the reference (different
structure, naming, and period-estimation method). Same physics -> should PASS L1 + L3. Demonstrates
the judge does not penalize stylistic divergence from the reference."""
import json
import math

import pychrono as chrono


def build_world(length, grav):
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0.0, -grav, 0.0))
    anchor = chrono.ChBody()
    anchor.SetFixed(True)
    system.AddBody(anchor)
    weight = chrono.ChBody()
    weight.SetMass(1.0)
    weight.SetInertiaXX(chrono.ChVector3d(1e-4, 1e-4, 1e-4))
    return system, anchor, weight


GRAV, LEN, A0, DT, TEND = 9.81, 1.0, math.radians(5.0), 1.0e-3, 5.0
world, anchor, weight = build_world(LEN, GRAV)
weight.SetPos(chrono.ChVector3d(LEN * math.sin(A0), -LEN * math.cos(A0), 0.0))
world.AddBody(weight)
hinge = chrono.ChLinkLockRevolute()
hinge.Initialize(anchor, weight, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
world.AddLink(hinge)

samples = []
while world.GetChTime() < TEND:
    world.DoStepDynamics(DT)
    pos = weight.GetPos()
    samples.append((world.GetChTime(), math.atan2(pos.x, -pos.y)))

with open("out.csv", "w") as fh:
    fh.write("t,theta\n")
    for tt, ang in samples:
        fh.write("%.6f,%.6e\n" % (tt, ang))

# Period via successive maxima (peak-to-peak) instead of zero crossings.
peaks = [samples[i][0] for i in range(1, len(samples) - 1)
         if samples[i][1] > samples[i - 1][1] and samples[i][1] >= samples[i + 1][1] and samples[i][1] > 0]
period = (peaks[-1] - peaks[0]) / (len(peaks) - 1) if len(peaks) >= 2 else float("nan")
print(json.dumps({"period_est": period, "theta_max": max(abs(a) for _, a in samples)}))
