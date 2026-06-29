"""A RUNS-BUT-WRONG pendulum: structurally fine and it executes cleanly (passes L1 and the minimal
L2 capability checks), but it uses the wrong pendulum length (0.5 m instead of the contracted 1.0 m),
so the period is physically wrong (~1.42 s vs ~2.01 s). The current text-only judge could easily miss this; the L3
behavioral invariant catches it. Demonstrates the execution+behavioral gate's discriminating power."""
import json
import math

import pychrono as chrono

m = 1.0
L = 0.5          # WRONG: contract specifies L = 1.0 m
g = 9.81
theta0 = math.radians(5.0)
t_end = 5.0
dt = 1.0e-3

sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, -g, 0.0))
ground = chrono.ChBody()
ground.SetFixed(True)
sys.AddBody(ground)
bob = chrono.ChBody()
bob.SetMass(m)
bob.SetInertiaXX(chrono.ChVector3d(1e-4, 1e-4, 1e-4))
bob.SetPos(chrono.ChVector3d(L * math.sin(theta0), -L * math.cos(theta0), 0.0))
sys.AddBody(bob)
rev = chrono.ChLinkLockRevolute()
rev.Initialize(ground, bob, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
sys.AddLink(rev)

ts, thetas = [], []
while sys.GetChTime() < t_end:
    sys.DoStepDynamics(dt)
    p = bob.GetPos()
    ts.append(sys.GetChTime())
    thetas.append(math.atan2(p.x, -p.y))

crossings = [ts[i] for i in range(1, len(thetas)) if thetas[i - 1] < 0.0 <= thetas[i]]
period_est = (crossings[-1] - crossings[0]) / (len(crossings) - 1) if len(crossings) >= 2 else float("nan")
with open("out.csv", "w") as f:
    f.write("t,theta\n")
    for t, th in zip(ts, thetas):
        f.write("%.6f,%.6e\n" % (t, th))
print(json.dumps({"period_est": period_est, "theta_max": max(abs(t) for t in thetas)}))
