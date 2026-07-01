"""A RUNS-BUT-WRONG turn-1 oscillator: structurally fine and it executes cleanly (passes L1 and the
minimal L2 capability checks), but it uses the wrong stiffness (k=400 instead of the specified 100, a
2x-too-stiff 'typo'), so the oscillation period and damping ratio are both physically wrong. The
CSV-derived L3 invariants catch it, and the wrong-physics cap applies. Demonstrates the gate's power."""
import json
import math

import pychrono as chrono

m, k, c, d, x0 = 1.0, 400.0, 2.0, 1.0, 0.1     # WRONG: k should be 100
t_end, dt = 5.0, 1.0e-4

sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, 0.0))
ground = chrono.ChBody()
ground.SetFixed(True)
sys.AddBody(ground)
mass = chrono.ChBody()
mass.SetMass(m)
mass.SetInertiaXX(chrono.ChVector3d(1e-3, 1e-3, 1e-3))
mass.SetPos(chrono.ChVector3d(0.0, -(d + x0), 0.0))
sys.AddBody(mass)
tsda = chrono.ChLinkTSDA()
tsda.Initialize(ground, mass, False, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, -(d + x0), 0))
tsda.SetRestLength(d)
tsda.SetSpringCoefficient(k)
tsda.SetDampingCoefficient(c)
sys.AddLink(tsda)

ts, disp = [], []
while sys.GetChTime() < t_end:
    sys.DoStepDynamics(dt)
    ts.append(sys.GetChTime())
    disp.append(mass.GetPos().y + d)

zc = [ts[i] for i in range(1, len(disp)) if disp[i - 1] < 0.0 <= disp[i]]
period_d = (zc[-1] - zc[0]) / (len(zc) - 1) if len(zc) >= 2 else float("nan")
peaks = [disp[i] for i in range(1, len(disp) - 1)
         if disp[i] > disp[i - 1] and disp[i] >= disp[i + 1] and disp[i] > 0.0]
delta = (1.0 / (len(peaks) - 1)) * math.log(peaks[0] / peaks[-1]) if len(peaks) >= 2 else float("nan")
zeta_est = delta / math.sqrt(4.0 * math.pi ** 2 + delta ** 2)
with open("out.csv", "w") as f:
    f.write("t,disp\n")
    for t, y in zip(ts, disp):
        f.write("%.6f,%.6e\n" % (t, y))
print(json.dumps({"period_d": period_d, "zeta_est": zeta_est}))
