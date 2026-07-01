"""A CORRECT turn-1 damped oscillator written in a deliberately different style (different structure,
naming, and period/zeta bookkeeping). Same physics as the reference -> should PASS L1 + L3. Demonstrates
the judge does not penalize stylistic divergence."""
import json
import math

import pychrono as chrono

MASS, STIFF, DAMP, REST, OFFSET = 1.0, 100.0, 2.0, 1.0, 0.1
DT, TEND = 1.0e-4, 5.0

world = chrono.ChSystemNSC()
world.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, 0.0))
wall = chrono.ChBody()
wall.SetFixed(True)
world.AddBody(wall)
block = chrono.ChBody()
block.SetMass(MASS)
block.SetInertiaXX(chrono.ChVector3d(1e-3, 1e-3, 1e-3))
block.SetPos(chrono.ChVector3d(0.0, -(REST + OFFSET), 0.0))
world.AddBody(block)
sd = chrono.ChLinkTSDA()
sd.Initialize(wall, block, False, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, -(REST + OFFSET), 0))
sd.SetRestLength(REST)
sd.SetSpringCoefficient(STIFF)
sd.SetDampingCoefficient(DAMP)
world.AddLink(sd)

times, ys = [], []
while world.GetChTime() < TEND:
    world.DoStepDynamics(DT)
    times.append(world.GetChTime())
    ys.append(block.GetPos().y + REST)

with open("out.csv", "w") as fh:
    fh.write("t,disp\n")
    for t, y in zip(times, ys):
        fh.write("%.6f,%.6e\n" % (t, y))

ups = [times[i] for i in range(1, len(ys)) if ys[i - 1] < 0.0 <= ys[i]]
period = (ups[-1] - ups[0]) / (len(ups) - 1) if len(ups) >= 2 else float("nan")
pks = [ys[i] for i in range(1, len(ys) - 1) if ys[i] > ys[i - 1] and ys[i] >= ys[i + 1] and ys[i] > 0]
delta = (1.0 / (len(pks) - 1)) * math.log(pks[0] / pks[-1]) if len(pks) >= 2 else float("nan")
zeta = delta / math.sqrt(4.0 * math.pi ** 2 + delta ** 2)
print(json.dumps({"period_d": period, "zeta_est": zeta}))
