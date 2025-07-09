import pychrono as chrono
import pychrono.irrlicht as chronoirr


sys = chrono.ChSystemNSC()


floor = chrono.ChBodyEasyBox(10, 1, 10, 1000, True, True)
floor.SetPos(chrono.ChVector3d(0, -5, 0))
floor.SetBodyFixed(True)
sys.Add(floor)


crankshaft = chrono.ChBodyEasyCylinder(0.5, 2, 1000, True, True)
crankshaft.SetPos(chrono.ChVector3d(0, 0, 0))
crankshaft.SetRot(chrono.ChQuaterniond(1, 0, 0, 0))
sys.Add(crankshaft)


connecting_rod = chrono.ChBodyEasyBox(4, 0.2, 0.2, 1000, True, True)
connecting_rod.SetPos(chrono.ChVector3d(2, 0, 0))
sys.Add(connecting_rod)


piston = chrono.ChBodyEasyCylinder(0.2, 1, 1000, True, True)
piston.SetPos(chrono.ChVector3d(4, 0, 0))
sys.Add(piston)


crankshaft_ground_joint = chrono.ChLinkLockRevolute()
crankshaft_ground_joint.Initialize(crankshaft, floor, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0)))
sys.Add(crankshaft_ground_joint)

crankshaft_connecting_rod_joint = chrono.ChLinkLockRevolute()
crankshaft_connecting_rod_joint.Initialize(crankshaft, connecting_rod, chrono.ChCoordsysd(chrono.ChVector3d(2, 0, 0)))
sys.Add(crankshaft_connecting_rod_joint)

connecting_rod_piston_joint = chrono.ChLinkLockRevolute()
connecting_rod_piston_joint.Initialize(connecting_rod, piston, chrono.ChCoordsysd(chrono.ChVector3d(4, 0, 0)))
sys.Add(connecting_rod_piston_joint)

piston_floor_joint = chrono.ChLinkLockPrismatic()
piston_floor_joint.Initialize(piston, floor, chrono.ChCoordsysd(chrono.ChVector3d(4, 0, 0)))
sys.Add(piston_floor_joint)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(crankshaft, floor, chrono.ChFrame(chrono.ChVector3d(0, 0, 0)))
motor_fun = chrono.ChFunction_Const(chrono.CH_PI)  
motor.SetSpeedFunction(motor_fun)
sys.Add(motor)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Crank-Slider Mechanism Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(5, 5, 5))
vis.AddTypicalLights()


while vis.Run():
    vis.BeginScene()
    vis.Render()
    sys.DoStepDynamics(0.01)
    vis.EndScene()