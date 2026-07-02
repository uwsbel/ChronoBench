"""Slider-crank, turn 3 (EXTEND: faster drive + piston-speed instrumentation) -- 10.0, headless, reference.

Same slider-crank as turn 2 (crank radius 0.6 m, rod 1.5 m), but the motor speed is doubled from pi to
2*pi rad/s. The stroke is a geometric quantity and stays 1.2 m, while the piston's peak speed scales with
the drive: peak |piston velocity| ~= 4.07 m/s (independent-oracle value, from max|omega*dx/dtheta|). Logs
the piston slide position; the judge measures both the stroke (range) and the peak speed (from d(x)/dt).
"""
import csv
import json

import pychrono.core as chrono

crank_center = chrono.ChVector3d(-1, 0.5, 0)
crank_rad = 0.6
crank_thick = 0.1
rod_length = 1.5
t_end, dt = 3.0, 1.0e-3

sys = chrono.ChSystemNSC()

floor = chrono.ChBodyEasyBox(3, 1, 3, 1000)
floor.SetPos(chrono.ChVector3d(0, -0.5, 0))
floor.SetFixed(True)
sys.Add(floor)

crank = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, crank_rad, crank_thick, 1000)
crank.SetPos(crank_center + chrono.ChVector3d(0, 0, -0.1))
crank.SetRot(chrono.Q_ROTATE_Y_TO_Z)
sys.Add(crank)

rod = chrono.ChBodyEasyBox(rod_length, 0.1, 0.1, 1000)
rod.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length / 2, 0, 0))
sys.Add(rod)

piston = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.2, 0.3, 1000)
piston.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0))
piston.SetRot(chrono.Q_ROTATE_Y_TO_X)
sys.Add(piston)

motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(crank, floor, chrono.ChFramed(crank_center))
motor.SetMotorFunction(chrono.ChFunctionConst(2.0 * chrono.CH_PI))    # turn-3 change: pi -> 2*pi
sys.Add(motor)

jA = chrono.ChLinkLockRevolute()
jA.Initialize(rod, crank, chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad, 0, 0)))
sys.Add(jA)

jB = chrono.ChLinkLockRevolute()
jB.Initialize(piston, rod, chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0)))
sys.Add(jB)

jC = chrono.ChLinkLockPrismatic()
jC.Initialize(piston, floor, chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0),
                                             chrono.Q_ROTATE_Z_TO_X))
sys.Add(jC)

ts, xs = [], []
while sys.GetChTime() < t_end:
    sys.DoStepDynamics(dt)
    ts.append(sys.GetChTime())
    xs.append(piston.GetPos().x)

stroke = max(xs) - min(xs)
peak_speed = max(abs((xs[i] - xs[i - 1]) / (ts[i] - ts[i - 1])) for i in range(1, len(xs)))
with open("out.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["t", "piston_x"])
    for t, x in zip(ts, xs):
        w.writerow([f"{t:.6f}", f"{x:.6e}"])
print(json.dumps({"stroke": stroke, "peak_speed": peak_speed}))
