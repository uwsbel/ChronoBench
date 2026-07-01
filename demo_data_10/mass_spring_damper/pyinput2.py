"""Damped spring-mass oscillator (PyChrono 10.0, headless) -- contracted reference.

A mass on a translational spring-damper (ChLinkTSDA) to a fixed ground, no gravity, released from a
small displacement. Logs displacement about equilibrium and reports the damped period and the damping
ratio estimated from the log-decrement. Analytic (m=1, k=100, c=2): zeta = c/(2*sqrt(k*m)) = 0.1,
wn = sqrt(k/m) = 10, wd = wn*sqrt(1-zeta^2), Td = 2*pi/wd ~= 0.6315 s.
"""
import math
import csv
import json

import pychrono as chrono

m, k, c, d, x0 = 1.0, 100.0, 2.0, 1.0, 0.1     # mass, stiffness, damping, rest length, initial offset
t_end, dt = 5.0, 1.0e-4

sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, 0.0))

ground = chrono.ChBody()
ground.SetFixed(True)
sys.AddBody(ground)

mass = chrono.ChBody()
mass.SetMass(m)
mass.SetInertiaXX(chrono.ChVector3d(1e-3, 1e-3, 1e-3))
mass.SetPos(chrono.ChVector3d(0.0, -(d + x0), 0.0))   # stretched by x0 below equilibrium (-d)
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
    disp.append(mass.GetPos().y + d)               # displacement about equilibrium at y = -d

# damped period from successive upward zero-crossings of the displacement
zc = [ts[i] for i in range(1, len(disp)) if disp[i - 1] < 0.0 <= disp[i]]
period_d = (zc[-1] - zc[0]) / (len(zc) - 1) if len(zc) >= 2 else float("nan")

# damping ratio from the log-decrement over successive positive peaks
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
