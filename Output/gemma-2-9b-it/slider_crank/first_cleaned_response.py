import pychrono as chrono
import pychrono.irrlicht as vis


chrono.SetChronoDataPath("path/to/chrono/data")  
system = chrono.ChSystemNSC()


floor = chrono.ChBodyEasyBox(10, 10, 0.1, 1000)
floor.SetPos(chrono.ChVectorD(0, 0, -0.1))
floor.SetBodyFixed(True)
system.Add(floor)


crankshaft = chrono.ChBodyEasyCylinder(0.1, 0.5, 1000)
crankshaft.SetPos(chrono.ChVectorD(0, 0, 0))
system.Add(crankshaft)


connecting_rod = chrono.ChBodyEasyCylinder(0.05, 0.5, 1000)
connecting_rod.SetPos(chrono.ChVectorD(0.5, 0, 0))
system.Add(connecting_rod)


piston = chrono.ChBodyEasyBox(0.2, 0.2, 0.1, 1000)
piston.SetPos(chrono.ChVectorD(1, 0, 0))
system.Add(piston)


crankshaft_joint = chrono.ChLinkLockPrismatic()
crankshaft_joint.Initialize(crankshaft, connecting_rod, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(1, 0, 0))
connecting_rod_joint = chrono.ChLinkLockRevolute()
connecting_rod_joint.Initialize(connecting_rod, piston, chrono.ChVectorD(0.25, 0, 0), chrono.ChVectorD(0, 0, 1))


system.Add(crankshaft_joint)
system.Add(connecting_rod_joint)


motor = chrono.ChMotorRotation(crankshaft_joint.GetJoint())
motor.SetMotorSpeed(10)  


vis.ChIrrApp(system, 'Crank-Slider Mechanism', 'data/irrlicht.cfg')


vis.GetActiveCamera().SetPos(chrono.ChVectorD(2, 2, 2))
vis.GetActiveCamera().SetTarget(chrono.ChVectorD(0, 0, 0))


vis.GetActiveCamera().SetFarClip(10)
vis.GetActiveCamera().SetNearClip(0.1)





vis.Start()