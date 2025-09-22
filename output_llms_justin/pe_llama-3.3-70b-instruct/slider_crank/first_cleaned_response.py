import os
import math
import numpy as np
from pychrono import chrono as chrono
from pychrono import irrlicht as chronoirr


chrono.SetChronoDataPath('/path/to/data')


sys = chrono.ChSystemNSC()


floor = chrono.ChBodyEasyBox(sys, 10, 1, 10, 1000, True, True)
floor.SetPos(chrono.ChVectorD(0, -5, 0))
sys.Add(floor)


crankshaft = chrono.ChBodyEasyCylinder(sys, 1, 1, 1000, True, True)
crankshaft.SetPos(chrono.ChVectorD(0, 0, 0))
sys.Add(crankshaft)


connecting_rod = chrono.ChBodyEasyBox(sys, 5, 1, 1, 1000, True, True)
connecting_rod.SetPos(chrono.ChVectorD(0, 2, 0))
sys.Add(connecting_rod)


piston = chrono.ChBodyEasyBox(sys, 1, 1, 1, 1000, True, True)
piston.SetPos(chrono.ChVectorD(0, 4, 0))
sys.Add(piston)


joint1 = chrono.ChLinkLockRevolute()
joint1.Initialize(crankshaft, connecting_rod, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
sys.Add(joint1)

joint2 = chrono.ChLinkLockRevolute()
joint2.Initialize(connecting_rod, piston, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
sys.Add(joint2)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(crankshaft, floor, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
motor.SetSpeedFunction(chrono.ChFunction_Const(1.0))
sys.Add(motor)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Crank-Slider Mechanism')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 3, 6))
vis.AddTypicalLights()


while vis.Run():
    sys.DoStepDynamics(0.01)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()