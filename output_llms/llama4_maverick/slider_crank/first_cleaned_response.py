import pychrono as chrono
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath("/path/to/chrono/data/")


sys = chrono.ChSystemNSC()


floor_mat = chrono.ChMaterialSurfaceNSC()
floor = chrono.ChBodyEasyBox(10, 0.2, 10, 1000, True, True, floor_mat)
floor.SetPos(chrono.ChVectorD(0, -1, 0))
floor.SetBodyFixed(True)
sys.Add(floor)


crank_mat = chrono.ChMaterialSurfaceNSC()
crank = chrono.ChBodyEasyCylinder(0.1, 0.5, 1000, True, True, crank_mat)
crank.SetPos(chrono.ChVectorD(0, 0, 0))
sys.Add(crank)


rod_mat = chrono.ChMaterialSurfaceNSC()
rod = chrono.ChBodyEasyBox(1, 0.1, 0.1, 1000, True, True, rod_mat)
rod.SetPos(chrono.ChVectorD(0.5, 0, 0))
sys.Add(rod)


piston_mat = chrono.ChMaterialSurfaceNSC()
piston = chrono.ChBodyEasyCylinder(0.1, 0.2, 1000, True, True, piston_mat)
piston.SetPos(chrono.ChVectorD(1.5, 0, 0))
sys.Add(piston)


crank_ground = chrono.ChLinkRevolute()
crank_ground.Initialize(crank, floor, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_ROTATE_Z_TO_Y))
sys.AddLink(crank_ground)

rod_crank = chrono.ChLinkRevolute()
rod_crank.Initialize(rod, crank, chrono.ChCoordsysD(chrono.ChVectorD(0.25, 0, 0), chrono.Q_ROTATE_Z_TO_Y))
sys.AddLink(rod_crank)

piston_rod = chrono.ChLinkRevolute()
piston_rod.Initialize(piston, rod, chrono.ChCoordsysD(chrono.ChVectorD(1.25, 0, 0), chrono.Q_ROTATE_Z_TO_Y))
sys.AddLink(piston_rod)

piston_guide = chrono.ChLinkSlider()
piston_guide.Initialize(piston, floor, chrono.ChCoordsysD(chrono.ChVectorD(1.5, 0, 0), chrono.Q_ROTATE_Z_TO_X))
sys.AddLink(piston_guide)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(crank, floor, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.Q_ROTATE_Z_TO_Y))
motor.SetSpeedFunction(chrono.ChFunction_Const(chrono.CH_C_PI))
sys.AddLink(motor)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Crank-Slider Mechanism')
vis.SetCameraProperties(chrono.ChVectorD(0, 2, -3), chrono.ChVectorD(0, 0, 0))
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)


vis.GetSceneManager().getRootSceneNode().addLightWithShadow(chrono.ChVectorD(2, 4, 2), chrono.ChVectorD(0, 0, 0), 10)
vis.GetSceneManager().getRootSceneNode().addLightWithShadow(chrono.ChVectorD(-2, 4, 2), chrono.ChVectorD(0, 0, 0), 10)


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)