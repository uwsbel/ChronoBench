import pychrono as chrono
import pychrono.irrlicht as irr


chrono.SetChronoDataPath('/usr/local/share/chrono/data/')


my_system = chrono.ChSystemNSC()


floor = chrono.ChBodyEasyBox(20, 1, 20, 1000, True, True)
floor.SetPos(chrono.ChVectorD(0, -1, 0))
floor.SetBodyFixed(True)
my_system.Add(floor)


crankshaft = chrono.ChBodyEasyCylinder(0.5, 2, 1000, True, True)
crankshaft.SetPos(chrono.ChVectorD(0, 0, 0))
my_system.Add(crankshaft)


connecting_rod = chrono.ChBodyEasyCylinder(0.5, 3, 1000, True, True)
connecting_rod.SetPos(chrono.ChVectorD(2, 0, 0))
my_system.Add(connecting_rod)


piston = chrono.ChBodyEasyCylinder(0.5, 1, 1000, True, True)
piston.SetPos(chrono.ChVectorD(5, 0, 0))
my_system.Add(piston)


joint1 = chrono.ChLinkLockRevolute()
joint1.Initialize(crankshaft, floor, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))))
my_system.Add(joint1)

joint2 = chrono.ChLinkLockRevolute()
joint2.Initialize(connecting_rod, crankshaft, chrono.ChCoordsysD(chrono.ChVectorD(2, 0, 0), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))))
my_system.Add(joint2)

joint3 = chrono.ChLinkLockRevolute()
joint3.Initialize(piston, connecting_rod, chrono.ChCoordsysD(chrono.ChVectorD(5, 0, 0), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))))
my_system.Add(joint3)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(crankshaft, floor, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))))
motor.SetSpeed(1)  
my_system.Add(motor)


my_application = irr.ChIrrApp(my_system, 'Crank-Slider Mechanism', irr.dimension2du(1024, 768))


my_application.SetCameraVertical(chrono.CameraVerticalDir.Z)
my_application.AddTypicalLogo()
my_application.AddTypicalSky()
my_application.AddTypicalLights()


my_application.SetTimestep(0.01)
my_application.AssetBindAll()
my_application.AssetUpdateAll()
my_application.Run()