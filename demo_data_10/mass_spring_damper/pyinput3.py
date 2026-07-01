"""Damped spring-mass oscillator, turn 2 (MODIFY: heavier damping) -- PyChrono 10.0, headless, reference.

Same oscillator as turn 1 (m=1, k=100, released from x0=0.1 m, no gravity), but the damping coefficient
is increased from c=2 to c=6, so the damping ratio zeta = c/(2*sqrt(k*m)) rises from 0.1 to 0.3 and the
damped period lengthens slightly (Td = 2*pi/(wn*sqrt(1-zeta^2)) ~= 0.659 s, wn=10). Logs (t, disp) and
reports the damped period and the damping ratio from the log-decrement. Targets come from the independent
oracle (oracle.py), not from this run.
"""
import math
import csv
import json

import pychrono as chrono

m, k, c, d, x0 = 1.0, 100.0, 6.0, 1.0, 0.1     # turn-2 change: c 2 -> 6 (zeta 0.1 -> 0.3)
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
if len(peaks) >= 2 and peaks[-1] > 0.0:
    delta = (1.0 / (len(peaks) - 1)) * math.log(peaks[0] / peaks[-1])
    zeta_est = delta / math.sqrt(4.0 * math.pi ** 2 + delta ** 2)
else:
    zeta_est = float("nan")

with open("out.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["t", "disp"])
    for t, y in zip(ts, disp):
        w.writerow([f"{t:.6f}", f"{y:.6e}"])

print(json.dumps({"period_d": period_d, "zeta_est": zeta_est}))
