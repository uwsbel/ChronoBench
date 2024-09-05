import pychrono as chrono
import pychrono.irrlicht as chronoirr


my_system = chrono.ChSystemNSC()
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


floor_material = chrono.ChMaterialSurfaceNSC()
floor_material.SetFriction(0.4)
floor_material.SetRestitution(0.2)
floor = chrono.ChBodyEasyBox(2, 0.1, 2, 1000, True, True, floor_material)
floor.SetPos(chrono.ChVectorD(0, -1, 0))
my_system.Add(floor)


crankshaft_radius = 0.2
crankshaft_length = 0.5
crankshaft = chrono.ChBodyEasyCylinder(crankshaft_radius, crankshaft_length, 1000, True, True)
crankshaft.SetPos(chrono.ChVectorD(0, 0, 0))
my_system.Add(crankshaft)


connecting_rod_length = 1
connecting_rod = chrono.ChBodyEasyBox(0.05, connecting_rod_length, 0.05, 100, True, True)
connecting_rod.SetPos(chrono.ChVectorD(crankshaft_radius, 0, 0))
my_system.Add(connecting_rod)


piston_radius = 0.1
piston = chrono.ChBodyEasyCylinder(piston_radius, 0.2, 100, True, True)
piston.SetPos(chrono.ChVectorD(crankshaft_radius + connecting_rod_length, 0, 0))
my_system.Add(piston)



revolute_joint = chrono.ChLinkRevolute()
revolute_joint.Initialize(crankshaft, connecting_rod, chrono.ChVectorD(crankshaft_radius, 0, 0))
my_system.Add(revolute_joint)


spherical_joint = chrono.ChLinkSphere()
spherical_joint.Initialize(connecting_rod, piston, chrono.ChVectorD(crankshaft_radius + connecting_rod_length, 0, 0))
my_system.Add(spherical_joint)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(crankshaft, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0))
motor.SetSpeedFunction(chrono.ChFunction_Const(10))  
my_system.Add(motor)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(my_system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle("Crank-Slider Mechanism")
vis.Initialize()




while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    my_system.DoStepDynamics(0.01)