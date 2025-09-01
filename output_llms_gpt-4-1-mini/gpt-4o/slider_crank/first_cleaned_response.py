import pychrono as chrono
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath('path/to/chrono/data/')


system = chrono.ChSystemNSC()


crank_length = 1.0
rod_length = 2.0
piston_length = 0.5
crank_radius = 0.1
rod_radius = 0.05
piston_radius = 0.2
motor_speed = chrono.CH_C_PI / 2  


floor = chrono.ChBody()
floor.SetBodyFixed(True)
system.Add(floor)


crankshaft = chrono.ChBodyEasyCylinder(crank_radius, crank_length, 1000, True, True)
crankshaft.SetPos(chrono.ChVectorD(0, 0, 0))
system.Add(crankshaft)


connecting_rod = chrono.ChBodyEasyCylinder(rod_radius, rod_length, 1000, True, True)
connecting_rod.SetPos(chrono.ChVectorD(crank_length, 0, 0))
system.Add(connecting_rod)


piston = chrono.ChBodyEasyBox(piston_length, piston_length, piston_length, 1000, True, True)
piston.SetPos(chrono.ChVectorD(crank_length + rod_length, 0, 0))
system.Add(piston)


revolute_joint = chrono.ChLinkLockRevolute()
revolute_joint.Initialize(floor, crankshaft, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
system.Add(revolute_joint)


spherical_joint = chrono.ChLinkLockSpherical()
spherical_joint.Initialize(crankshaft, connecting_rod, chrono.ChCoordsysD(chrono.ChVectorD(crank_length, 0, 0), chrono.QUNIT))
system.Add(spherical_joint)


spherical_joint2 = chrono.ChLinkLockSpherical()
spherical_joint2.Initialize(connecting_rod, piston, chrono.ChCoordsysD(chrono.ChVectorD(crank_length + rod_length, 0, 0), chrono.QUNIT))
system.Add(spherical_joint2)


prismatic_joint = chrono.ChLinkLockPrismatic()
prismatic_joint.Initialize(floor, piston, chrono.ChCoordsysD(chrono.ChVectorD(crank_length + rod_length, 0, 0), chrono.Q_from_AngAxis(chrono.CH_C_PI / 2, chrono.VECT_Z)))
system.Add(prismatic_joint)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(floor, crankshaft, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
motor.SetSpeedFunction(chrono.ChFunction_Const(motor_speed))
system.Add(motor)


application = chronoirr.ChIrrApp(system, 'Crank-Slider Mechanism', chronoirr.dimension2du(800, 600))

application.AddLogo()
application.AddSkyBox()
application.AddTypicalLights()
application.AddCamera(chronoirr.vector3df(2, 2, -2))

application.AssetBindAll()
application.AssetUpdateAll()


application.SetTimestep(0.01)

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()