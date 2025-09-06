import pychrono as chrono
import pychrono.irrlicht as chronoirr


sys = chrono.ChSystemNSC()


material = chrono.ChContactMaterialNSC()
material.SetFriction(0.2)


floor = chrono.ChBodyEasyBox(5, 1, 5, 1000, True, True, material)
floor.SetFixed(True)
floor.SetPos(chrono.ChVector3d(0, -1, 0))
floor.GetVisualShape(0).SetColor(chrono.ChColor(0.6, 0.6, 0.6))  
sys.Add(floor)


crankshaft = chrono.ChBodyEasyCylinder(0.5, 0.2, 1000, True, True, material)
crankshaft.SetPos(chrono.ChVector3d(0, 0, 0))
crankshaft.GetVisualShape(0).SetColor(chrono.ChColor(1, 0, 0))  
sys.Add(crankshaft)


revolute_floor_crank = chrono.ChLinkLockRevolute()
revolute_floor_crank.Initialize(
    floor, crankshaft, 
    chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleZ(0))
)
sys.Add(revolute_floor_crank)


connecting_rod = chrono.ChBodyEasyBox(2.0, 0.2, 0.2, 1000, True, True, material)
connecting_rod.SetPos(chrono.ChVector3d(1.5, 0, 0))  
connecting_rod.GetVisualShape(0).SetColor(chrono.ChColor(0, 1, 0))  
sys.Add(connecting_rod)


revolute_crank_rod = chrono.ChLinkLockRevolute()
revolute_crank_rod.Initialize(
    crankshaft, connecting_rod,
    chrono.ChCoordsysD(chrono.ChVector3d(0.5, 0, 0), chrono.QuatFromAngleZ(0))
)
sys.Add(revolute_crank_rod)


piston = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 1000, True, True, material)
piston.SetPos(chrono.ChVector3d(2.5, 0, 0))  
piston.GetVisualShape(0).SetColor(chrono.ChColor(0, 0, 1))  
sys.Add(piston)


revolute_rod_piston = chrono.ChLinkLockRevolute()
revolute_rod_piston.Initialize(
    connecting_rod, piston,
    chrono.ChCoordsysD(chrono.ChVector3d(2.5, 0, 0), chrono.QuatFromAngleZ(0))
)
sys.Add(revolute_rod_piston)


prismatic_piston_floor = chrono.ChLinkLockPrismatic()
prismatic_piston_floor.Initialize(
    piston, floor,
    chrono.ChCoordsysD(chrono.ChVector3d(2.5, 0, 0), chrono.QuatFromAngleZ(0))
)
sys.Add(prismatic_piston_floor)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(
    crankshaft, floor,
    chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleZ(0))
)

speed_function = chrono.ChFunction_Const(chrono.CH_PI * 2)
motor.SetSpeedFunction(speed_function)
sys.Add(motor)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Crank-Slider Mechanism')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2, -6))  
vis.AddTypicalLights()


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)