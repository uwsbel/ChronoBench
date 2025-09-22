import pychrono as chrono
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()


floor = chrono.ChBodyEasyBox(10, 1, 1, 1000, True, True)
floor.SetPos(chrono.ChVectorD(0, -1, 0))
system.Add(floor)


crankshaft = chrono.ChBodyEasyCylinder(0.5, 1, 1000, True, True)
crankshaft.SetPos(chrono.ChVectorD(0, 0, 0))
system.Add(crankshaft)


connecting_rod = chrono.ChBodyEasyCylinder(0.5, 2, 1000, True, True)
connecting_rod.SetPos(chrono.ChVectorD(2, 0, 0))
system.Add(connecting_rod)


piston = chrono.ChBodyEasyCylinder(0.5, 1, 1000, True, True)
piston.SetPos(chrono.ChVectorD(4, 0, 0))
system.Add(piston)


crankshaft_connecting_rod_joint = chrono.ChLinkLockRevolute()
crankshaft_connecting_rod_joint.Initialize(crankshaft, connecting_rod, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))))
system.Add(crankshaft_connecting_rod_joint)

connecting_rod_piston_joint = chrono.ChLinkLockRevolute()
connecting_rod_piston_joint.Initialize(connecting_rod, piston, chrono.ChFrameD(chrono.ChVectorD(2, 0, 0), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))))
system.Add(connecting_rod_piston_joint)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(crankshaft, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))))
motor.SetSpeed(2)  
system.Add(motor)


app = irr.ChIrrApp(system, 'Crank-Slider Mechanism', irr.dimension2du(1024, 768))
app.AddTypicalLights()
app.AddCamera(irr.vector3df(0, 3, -6))
app.AssetBindAll()
app.AssetUpdateAll()


while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()
    app.DoStep()
    app.EndScene()