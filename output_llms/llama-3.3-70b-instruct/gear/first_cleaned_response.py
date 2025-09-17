import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.engine as chronoengine


sys = chrono.ChSystemNSC()


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AddSkyBox()
vis.AddTypicalLights()


ground = chrono.ChBodyEasyBox(sys, 10, 1, 10, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -5, 0))
ground.SetBodyFixed(True)


bar = chrono.ChBodyEasyBox(sys, 1, 10, 1, 100, True, True)
bar.SetPos(chrono.ChVectorD(0, 0, 0))
bar.SetRot(chrono.Q_from_AngX(0))


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(bar, ground, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0), chrono.ChVectorD(0, 1, 0))
motor.SetSpeed(10)


gear1 = chrono.ChBodyEasyCylinder(sys, 2, 1, 100, True, True)
gear1.SetPos(chrono.ChVectorD(-3, 0, 0))
gear1.SetRot(chrono.Q_from_AngZ(0))

gear2 = chrono.ChBodyEasyCylinder(sys, 1, 1, 100, True, True)
gear2.SetPos(chrono.ChVectorD(3, 0, 0))
gear2.SetRot(chrono.Q_from_AngZ(0))


contact1 = chrono.ChLinkMateGeneric()
contact1.Initialize(gear1, bar, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0), chrono.ChVectorD(0, 1, 0))
contact1.SetSlidingMode(chrono.ChLinkMateGeneric.SLIDING_MODE_STICK)

contact2 = chrono.ChLinkMateGeneric()
contact2.Initialize(gear2, bar, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0), chrono.ChVectorD(0, 1, 0))
contact2.SetSlidingMode(chrono.ChLinkMateGeneric.SLIDING_MODE_STICK)


sys.Add(ground)
sys.Add(bar)
sys.Add(motor)
sys.Add(gear1)
sys.Add(gear2)
sys.Add(contact1)
sys.Add(contact2)


vis.AddCamera(chrono.ChCamera())
vis.AddTypicalLights()
vis.SetLogoVisibility(False)
vis.SetSkyBoxVisibility(True)
vis.SetTypicalSceneFloor(True)


vis.Run()


while vis.Run():
    sys.DoStepDynamics(0.01)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()