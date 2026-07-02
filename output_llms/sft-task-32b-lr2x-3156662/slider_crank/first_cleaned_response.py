import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math


system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))  


floor = chrono.ChBodyEasyBox(2.0, 0.1, 0.1, 1000, True, True)
floor.SetName("Floor")
floor.SetFixed(True)
floor.SetPos(chrono.ChVector3d(0, -0.2, 0))
floor.GetVisualShape(0).SetColor(chrono.ChColor(0.3, 0.3, 0.3))

texture_floor = chrono.ChVisualShapeTexture('floor_texture.jpg')
floor.AddVisualShape(texture_floor)
system.AddBody(floor)


crankshaft = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, 0.05, 0.2, 7800, True, True)
crankshaft.SetName("Crankshaft")
crankshaft.SetPos(chrono.ChVector3d(0, 0.3, 0))
crankshaft.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.6, 0.8))
system.AddBody(crankshaft)


connecting_rod = chrono.ChBodyEasyBox(0.4, 0.05, 0.05, 7800, True, True)
connecting_rod.SetName("Connecting Rod")
connecting_rod.SetPos(chrono.ChVector3d(0.23, 0.15, 0))
connecting_rod.GetVisualShape(0).SetColor(chrono.ChColor(0.9, 0.5, 0.1))
system.AddBody(connecting_rod)


piston = chrono.ChBodyEasyBox(0.2, 0.1, 0.1, 7800, True, True)
piston.SetName("Piston")
piston.SetPos(chrono.ChVector3d(0.3, 0, 0))
piston.GetVisualShape(0).SetColor(chrono.ChColor(0.7, 0.7, 0.7))
system.AddBody(piston)


rev_crank_floor = chrono.ChLinkLockRevolute()
rev_crank_floor.Initialize(crankshaft, floor, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
system.AddLink(rev_crank_floor)


rev_crank_rod = chrono.ChLinkLockRevolute()
rev_crank_rod.Initialize(crankshaft, connecting_rod, chrono.ChFramed(chrono.ChVector3d(0.1, 0.3, 0)))
system.AddLink(rev_crank_rod)


rev_rod_piston = chrono.ChLinkLockRevolute()
rev_rod_piston.Initialize(connecting_rod, piston, chrono.ChFramed(chrono.ChVector3d(0.3, 0, 0)))
system.AddLink(rev_rod_piston)


prismatic_piston = chrono.ChLinkLockPrismatic()
prismatic_piston.Initialize(piston, floor, chrono.ChFramed(chrono.ChVector3d(0.3, 0, 0), chrono.Q_ROTATE_Z_TO_X))
system.AddLink(prismatic_piston)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(crankshaft, floor, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
motor.SetMotorFunction(chrono.ChFunctionConst(chrono.CH_PI))  
system.AddLink(motor)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('PyChrono Crank-Slider Mechanism')
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1.5, 1.5, 1.5), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()


logo_shape = chrono.ChVisualShapeBillboard()
logo_shape.SetTexture('logo.png')
floor.AddVisualShape(logo_shape)


time = 0
time_step = 0.001
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(time_step)
    time += time_step