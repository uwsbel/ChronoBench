import os
import math
import numpy as np
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.robot as robosimian
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath('/path/to/data')


sys = chrono.ChSystemNSC()


truss = chrono.ChBody()
truss.SetPos(chrono.ChVector3d(0, 0, 0))
truss_shape = chrono.ChVisualShapeBox(chrono.ChVector3d(1, 1, 1))
truss.AddVisualShape(truss_shape)
sys.Add(truss)


bar = chrono.ChBody()
bar.SetPos(chrono.ChVector3d(0, 0, 1))
bar_shape = chrono.ChVisualShapeBox(chrono.ChVector3d(1, 1, 1))
bar.AddVisualShape(bar_shape)
sys.Add(bar)


gear1 = chrono.ChBody()
gear1.SetPos(chrono.ChVector3d(0, 0, 2))
gear1_shape = chrono.ChVisualShapeBox(chrono.ChVector3d(1, 1, 1))
gear1.AddVisualShape(gear1_shape)
sys.Add(gear1)

gear2 = chrono.ChBody()
gear2.SetPos(chrono.ChVector3d(0, 0, 3))
gear2_shape = chrono.ChVisualShapeBox(chrono.ChVector3d(1, 1, 1))
gear2.AddVisualShape(gear2_shape)
sys.Add(gear2)

motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(gear1, gear2, chrono.ChFramed())
sys.Add(motor)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(gear2, chrono.ChFramed(), chrono.ChFunction_Const(1.0))
sys.Add(motor)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Epicyclic Gears Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 6))
vis.AddTypicalLights()


while vis.Run():
    sys.DoStepDynamics(0.01)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()