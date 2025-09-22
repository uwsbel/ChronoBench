import PyChrono as chrono
import PyChrono.Irrlicht as chronoirr


chrono.SetChronoDataPath("/path/to/chrono/data/")  
my_system = chrono.ChSystemNSC()


floor_length = 2
floor_thickness = 0.1
floor = chrono.ChBodyEasyBox(floor_length, floor_thickness, floor_length, 1000, True, True)
floor.SetPos(chrono.ChVectorD(0, -floor_thickness, 0))
floor.SetBodyFixed(True)
my_system.Add(floor)


crank_radius = 0.1
crank_length = 0.2
crank = chrono.ChBodyEasyCylinder(crank_radius, crank_length, 1000, True, True)
crank.SetPos(chrono.ChVectorD(0, 0, 0))
my_system.Add(crank)


rod_length = 0.5
rod_radius = 0.05
rod = chrono.ChBodyEasyCylinder(rod_radius, rod_length, 1000, True, True)
rod.SetPos(chrono.ChVectorD(0, 0, rod_length / 2))
my_system.Add(rod)


piston_radius = 0.05
piston_length = 0.1
piston = chrono.ChBodyEasyCylinder(piston_radius, piston_length, 1000, True, True)
piston.SetPos(chrono.ChVectorD(0, 0, rod_length + piston_length / 2))
my_system.Add(piston)


crank_rod_joint = chrono.ChLinkLockRevolute()
crank_rod_joint.Initialize(crank, rod, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, crank_length / 2)))
my_system.Add(crank_rod_joint)

rod_piston_joint = chrono.ChLinkLockPrismatic()
rod_piston_joint.Initialize(rod, piston, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, rod_length)))
my_system.Add(rod_piston_joint)

crank_motor = chrono.ChLinkMotorRotationSpeed()
crank_motor.Initialize(floor, crank, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0)))
crank_motor.Set_speed_function(chrono.ChFunction_Const(10 * chrono.CH_C_PI))  
my_system.Add(crank_motor)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(800, 600)
vis.SetWindowTitle("Crank-Slider Mechanism")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(1, 1, 0.5))
vis.AddLight(chrono.ChVectorD(1, 1, 1))
vis.AddLight(chrono.ChVectorD(-1, -1, -1))


while vis.Run():
    my_system.DoStepDynamics(0.01)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()


vis.Close()