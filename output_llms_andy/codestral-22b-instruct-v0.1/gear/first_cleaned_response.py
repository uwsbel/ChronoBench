import pychrono as chrono
import pychrono.irrlicht as chronoirr


system = chrono.ChSystemNSC()


ground = chrono.ChBodyEasyBox(10, 1, 10, 1000, True, True)
system.Add(ground)


bar = chrono.ChBodyEasyCylinder(0.5, 2, 1000, True, True)
bar.SetPos(chrono.ChVectorD(0, 1.5, 0))
system.Add(bar)


gear1 = chrono.ChBodyEasyCylinder(0.5, 0.5, 1000, True, True)
gear1.SetPos(chrono.ChVectorD(-2, 1, 0))
system.Add(gear1)

gear2 = chrono.ChBodyEasyCylinder(0.5, 0.5, 1000, True, True)
gear2.SetPos(chrono.ChVectorD(2, 1, 0))
system.Add(gear2)


link1 = chrono.ChLinkLockRevolute()
link1.Initialize(bar, gear1, chrono.ChFrameD(chrono.ChVectorD(-2, 1, 0)))
system.Add(link1)

link2 = chrono.ChLinkLockRevolute()
link2.Initialize(bar, gear2, chrono.ChFrameD(chrono.ChVectorD(2, 1, 0)))
system.Add(link2)

link3 = chrono.ChLinkLockRevolute()
link3.Initialize(gear1, gear2, chrono.ChFrameD(chrono.ChVectorD(0, 1, 0)))
system.Add(link3)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(gear1, chrono.ChFrameD(chrono.ChVectorD(-2, 1, 0)))
motor.SetSpeed(10)  
system.Add(motor)


app = chronoirr.ChIrrApp(system, 'Epicyclic Gears', chronoirr.dimension2du(1024, 768))


camera = chronoirr.ChIrrliCamera(app.GetDevice())
camera.SetCameraPosition(chrono.ChVectorD(0, 5, -10))
camera.SetLookAtPoint(chrono.ChVectorD(0, 0, 0))


app.AddLightSource(chronoirr.ChIrrliLightSource(app.GetDevice(), 101, chrono.ChVectorD(5, 5, -5), chrono.ChColor(1, 1, 1), 500))


app.SetTimestep(0.01)
app.AssetBindAll()
app.AssetUpdateAll()
app.Run()