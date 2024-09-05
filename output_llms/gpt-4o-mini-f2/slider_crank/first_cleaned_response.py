import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math


system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))  


floor = chrono.ChBodyEasyBox(10, 10, 1, 1000, True, True)
floor.SetPos(chrono.ChVector3d(0, 0, -0.5))  
floor.SetFixed(True)  
floor.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))  
system.Add(floor)  


crankshaft = chrono.ChBodyEasyCylinder(0.25, 5, 1000, True, True)
crankshaft.SetPos(chrono.ChVector3d(-2.5, 0, 0))  
crankshaft.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))  
system.Add(crankshaft)  


connecting_rod = chrono.ChBodyEasyBox(1, 3, 1, 1000, True, True)
connecting_rod.SetPos(chrono.ChVector3d(-1.25, 0, 0))  
connecting_rod.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/green.png"))  
system.Add(connecting_rod)  


piston = chrono.ChBodyEasyCylinder(0.5, 1, 1000, True, True)
piston.SetPos(chrono.ChVector3d(1, 0, 0))  
piston.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/red.png"))  
system.Add(piston)  


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(crankshaft, floor, chrono.ChFramed(chrono.ChVector3d(-2.5, 0, 0)))  
motor.SetSpeedFunction(chrono.ChFunctionConst(chrono.CH_PI))  
system.Add(motor)  


joint1 = chrono.ChLinkLockRevolute()
joint1.Initialize(connecting_rod, crankshaft, chrono.ChFramed(chrono.ChVector3d(-1.25, 0, 0)))  
system.Add(joint1)  


joint2 = chrono.ChLinkLockRevolute()
joint2.Initialize(piston, connecting_rod, chrono.ChFramed(chrono.ChVector3d(1, 0, 0)))  
system.Add(joint2)  


joint3 = chrono.ChLinkLockPrismatic()
joint3.Initialize(piston, floor, chrono.ChFramed(chrono.ChVector3d(1, 0, 0), chrono.QuatFromAngleY(chrono.CH_PI_2)))  
system.Add(joint3)  


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)  
vis.SetWindowSize(1024, 768)  
vis.SetWindowTitle('Crank slider')  
vis.Initialize()  
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  
vis.AddSkyBox()  
vis.AddCamera(chrono.ChVector3d(0, 3, 5), chrono.ChVector3d(0, 0, 0))  
vis.AddTypicalLights()  


time_step = 1e-2


while vis.Run():
    vis.BeginScene()  
    vis.Render()  
    vis.EndScene()  
    system.DoStepDynamics(time_step)