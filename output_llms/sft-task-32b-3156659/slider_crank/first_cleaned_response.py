import pychrono as chrono
import pychrono.irrlicht as chronoirr


system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))  


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 720)
vis.SetWindowTitle("Crank-Slider Mechanism")
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1.5, 1.5, 1.5), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()


floor = chrono.ChBodyEasyBox(10, 0.1, 10, 1000, True, True)
floor.SetPos(chrono.ChVector3d(0, -0.5, 0))
floor.SetFixed(True)
floor.GetVisualShape(0).SetColor(chrono.ChColor(0.5, 0.5, 0.5))
system.AddBody(floor)


crank = chrono.ChBodyEasyCylinder(chrono.ChAxis_X, 0.05, 0.2, 1000, True, True)
crank.SetPos(chrono.ChVector3d(0, 0, 0))
crank.SetMass(1.0)
crank.SetInertiaXX(chrono.ChVector3d(0.01, 0.01, 0.01))
crank.GetVisualShape(0).SetColor(chrono.ChColor(0.8, 0.2, 0.2))
system.AddBody(crank)


rod_length = 0.5
rod = chrono.ChBodyEasyBox(rod_length, 0.05, 0.05, 1000, True, True)
rod.SetPos(chrono.ChVector3d(0.35, 0, 0))  
rod.SetMass(0.5)
rod.SetInertiaXX(chrono.ChVector3d(0.001, 0.001, 0.001))
rod.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.8, 0.2))
system.AddBody(rod)


piston = chrono.ChBodyEasyBox(0.1, 0.05, 0.05, 1000, True, True)
piston.SetPos(chrono.ChVector3d(0.6, 0, 0))  
piston.SetMass(0.2)
piston.SetInertiaXX(chrono.ChVector3d(0.0001, 0.0001, 0.0001))
piston.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.2, 0.8))
system.AddBody(piston)


rev_crank = chrono.ChLinkLockRevolute()
rev_crank.Initialize(crank, floor, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
system.AddLink(rev_crank)


rev_rod_crank = chrono.ChLinkLockRevolute()
rev_rod_crank.Initialize(rod, crank, chrono.ChFramed(chrono.ChVector3d(0.1, 0, 0)))
system.AddLink(rev_rod_crank)


rev_rod_piston = chrono.ChLinkLockRevolute()
rev_rod_piston.Initialize(rod, piston, chrono.ChFramed(chrono.ChVector3d(0.6, 0, 0)))
system.AddLink(rev_rod_piston)


prismatic = chrono.ChLinkLockPrismatic()
prismatic.Initialize(piston, floor, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
system.AddLink(prismatic)


motor = chrono.ChLinkMotorRotationSpeed()
motor.SetName("Crank Motor")
motor.Initialize(crank, floor, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
motor.SetSpeedFunction(chrono.ChFunctionConst(chrono.CH_C_PI))  
system.AddLink(motor)


time_step = 1e-3
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(time_step)