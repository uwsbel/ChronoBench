import pychrono as chrono
import numpy as np


chrono.SetChronoDataPath("path/to/chrono_data")
app = chrono.ChApp("Simple Pendulum", chrono.ChIrrApp.IrrApp(chrono.ChVectorD(1024, 768), "Simple Pendulum", chrono.ChVectorD(800, 600)))
app.SetTimestep(0.005)


ground = chrono.ChBodyEasyBox(10, 10, 1, 1e-03, True)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
app.Add(ground)


pendulum = chrono.ChBodyEasyCylinder(0.1, 1, 1000, 1e-03, True)
pendulum.SetPos(chrono.ChVectorD(0, 1, 0))
pendulum.SetMass(1)
pendulum.SetInertiaXX(chrono.ChVectorD(1, 0.1, 0.1))
app.Add(pendulum)


joint = chrono.ChLinkRevolute()
joint.Initialize(ground, pendulum, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 1))
app.Add(joint)


pendulum.SetPos_dt(chrono.ChVectorD(0, 0, 0))
pendulum.SetWvel_abs(chrono.ChVectorD(0, 0, 0.5))


logger = chrono.ChLogger()
logger.SetLogLevel(chrono.ChLogLevel.LEVEL_INFO)
logger.Enable()
logger.AddItem("Pendulum Position", pendulum.GetPos())
logger.AddItem("Pendulum Velocity", pendulum.GetWvel_abs())


while app.GetTimestep() >= 0:
    app.BeginScene()
    app.DrawAll()
    app.EndScene()
    app.Advance()