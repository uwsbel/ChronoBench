import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.engine as chronoengine


mysystem = chrono.ChSystemNSC()


vis = chronoirr.ChVisualSystemIrrlicht(mysystem)


vis.SetWindowSize(800, 600)
vis.SetWindowTitle("Epicyclic Gears Simulation")


truss = chrono.ChBodyEasyBox(mysystem, 1, 1, 1, 1000, True, True)
truss.SetPos(chrono.ChVectorD(0, 0, 0))
truss.SetBodyFixed(True)
mysystem.Add(truss)


bar = chrono.ChBodyEasyBox(mysystem, 5, 0.5, 0.5, 1000, True, True)
bar.SetPos(chrono.ChVectorD(0, 2, 0))
bar.SetRot(chrono.ChQuaternionD(chrono.ChVectorD(0, 1, 0), chrono.ChAngleD(chrono.CH_C_PI_2)))
mysystem.Add(bar)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(bar, truss, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
motor.SetSpeed(chrono.ChVectorD(0, 0, 1))
mysystem.Add(motor)


gear1 = chrono.ChBodyEasyCylinder(mysystem, 1, 1, 1000, True, True)
gear1.SetPos(chrono.ChVectorD(2, 0, 0))
gear1.SetRot(chrono.ChQuaternionD(chrono.ChVectorD(0, 1, 0), chrono.ChAngleD(chrono.CH_C_PI_2)))
mysystem.Add(gear1)

gear2 = chrono.ChBodyEasyCylinder(mysystem, 1, 1, 1000, True, True)
gear2.SetPos(chrono.ChVectorD(-2, 0, 0))
gear2.SetRot(chrono.ChQuaternionD(chrono.ChVectorD(0, 1, 0), chrono.ChAngleD(chrono.CH_C_PI_2)))
mysystem.Add(gear2)


vis.AddSkyBox()
vis.AddTypicalLights()


vis.SetCamera(chrono.ChVectorD(0, 0, 5), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0))


while vis.Run():
    mysystem.DoStepDynamics(0.01)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()