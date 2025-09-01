import pychrono as chrono
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()


floor = chrono.ChBodyEasyBox(10, 1, 1, 1000, True, True)
floor.SetPos(chrono.ChVectorD(0, -1, 0))
system.Add(floor)


crankshaft = chrono.ChBodyEasyCylinder(0.5, 2, 1000, True, True)
crankshaft.SetPos(chrono.ChVectorD(0, 0, 0))
system.Add(crankshaft)


connecting_rod = chrono.ChBodyEasyCylinder(0.5, 2, 1000, True, True)
connecting_rod.SetPos(chrono.ChVectorD(2, 0, 0))
system.Add(connecting_rod)


piston = chrono.ChBodyEasyCylinder(0.5, 2, 1000, True, True)
piston.SetPos(chrono.ChVectorD(4, 0, 0))
system.Add(piston)


crank_rod_joint = chrono.ChLinkLockRevolute()
crank_rod_joint.Initialize(crankshaft, connecting_rod, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))))
system.Add(crank_rod_joint)

rod_piston_joint = chrono.ChLinkLockRevolute()
rod_piston_joint.Initialize(connecting_rod, piston, chrono.ChFrameD(chrono.ChVectorD(2, 0, 0), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))))
system.Add(rod_piston_joint)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(crankshaft, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))))
motor.SetSpeed(10)  
system.Add(motor)


app = irr.ChIrrApp(system, 'Crank-Slider Mechanism', irr.dimension2du(1024, 768))
app.AddTypicalLogo()
app.AddTypicalCamera(irr.vector3df(0, 3, -6))
app.AddTypicalLights()


app.SetTimestep(0.01)
while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()
    app.DoStep()
    app.EndScene()