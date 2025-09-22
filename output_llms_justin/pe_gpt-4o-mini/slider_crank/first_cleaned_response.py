import os
import math
import numpy as np
import pychrono as chrono
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath('/path/to/data')  
sys = chrono.ChSystemNSC()


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Crank-Slider Mechanism Simulation')
vis.Initialize()
vis.AddTypicalLights()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(5, 5, 10))
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))


floor = chrono.ChBodyEasyBox(10, 0.1, 10, 1000, True, True)
floor.SetPos(chrono.ChVector3d(0, -0.05, 0))
floor.SetFixed(True)
sys.Add(floor)


crankshaft = chrono.ChBodyEasyCylinder(0.05, 1, 1000, True, True)
crankshaft.SetPos(chrono.ChVector3d(0, 0, 0))
sys.Add(crankshaft)


connecting_rod = chrono.ChBodyEasyBox(0.05, 0.5, 0.1, 1000, True, True)
connecting_rod.SetPos(chrono.ChVector3d(0, 0.25, 0))
sys.Add(connecting_rod)


piston = chrono.ChBodyEasyBox(0.1, 0.1, 0.1, 1000, True, True)
piston.SetPos(chrono.ChVector3d(0, 0.5, 0))
sys.Add(piston)



crank_joint = chrono.ChLinkLockRevolute()
crank_joint.Initialize(crankshaft, floor, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
sys.Add(crank_joint)


piston_joint = chrono.ChLinkLockPrismatic()
piston_joint.Initialize(connecting_rod, piston, chrono.ChCoordsysD(chrono.ChVector3d(0, 0.5, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
sys.Add(piston_joint)


connecting_joint = chrono.ChLinkUniversal()
connecting_joint.Initialize(connecting_rod, crankshaft, chrono.ChCoordsysD(chrono.ChVector3d(0, 0.25, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
sys.Add(connecting_joint)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(crankshaft, floor, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
motor.SetSpeedFunction(chrono.ChFunction_Const(1.0))  
sys.Add(motor)


time_step = 0.01
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(time_step)


vis.Close()