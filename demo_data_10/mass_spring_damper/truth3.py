"""Damped spring-mass oscillator, turn 3 (EXTEND: resonant forcing) -- PyChrono 10.0, headless, reference.

Extends the turn-2 damped oscillator (m=1, k=100, c=6 -> zeta=0.3, wn=sqrt(k/m)=10 rad/s) by adding a
sinusoidal driving force F(t) = F0*sin(wn*t), F0=1 N, applied along the motion axis at the undamped
natural frequency (resonance). After the initial transient decays, the mass settles into a steady-state
oscillation whose amplitude is Xss = F0/(2*zeta*k) ~= 0.0167 m (independent-oracle value). Logs (t, disp)
and reports the steady-state amplitude (peak |disp| over the tail of the run).
"""
import math
import csv
import json

import pychrono as chrono

m, k, c, d, x0 = 1.0, 100.0, 6.0, 1.0, 0.1
F0 = 1.0
wn = math.sqrt(k / m)          # 10 rad/s
t_end, dt, t_meas = 5.0, 1.0e-4, 3.0

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

# Resonant sinusoidal driving force along the motion (Y) axis: F(t) = F0*sin(wn*t).
frc = chrono.ChForce()
mass.AddForce(frc)
frc.SetMode(chrono.ChForce.FORCE)
frc.SetDir(chrono.ChVector3d(0, 1, 0))
frc.SetMforce(F0)
frc.SetModulation(chrono.ChFunctionSine(1.0, wn / (2.0 * math.pi)))   # sin(wn * t)

ts, disp = [], []
while sys.GetChTime() < t_end:
    sys.DoStepDynamics(dt)
    ts.append(sys.GetChTime())
    disp.append(mass.GetPos().y + d)

tail = [abs(x) for t, x in zip(ts, disp) if t >= t_meas]
ss_amp = max(tail) if tail else float("nan")

with open("out.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["t", "disp"])
    for t, y in zip(ts, disp):
        w.writerow([f"{t:.6f}", f"{y:.6e}"])

print(json.dumps({"ss_amp": ss_amp, "drive_omega": wn}))
