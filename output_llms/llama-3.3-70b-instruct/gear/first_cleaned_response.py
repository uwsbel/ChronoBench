import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.engine as chronoengine


system = chrono.ChSystemNSC()


vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(0, 0, 5))
vis.SetCameraAZEL(0, -45, 0)


ground = chrono.ChBodyEasyBox(system, 10, 1, 10, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -5, 0))
ground.SetMaterial(chrono.ChMaterialSurfaceNSC())


bar = chrono.ChBodyEasyBox(system, 1, 10, 1, 100, True, True)
bar.SetPos(chrono.ChVectorD(0, 0, 0))
bar.SetMaterial(chrono.ChMaterialSurfaceNSC())
bar.SetMass(1)
bar.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))


gear1 = chrono.ChBodyEasyCylinder(system, 2, 1, 100, True, True)
gear1.SetPos(chrono.ChVectorD(-3, 0, 0))
gear1.SetMaterial(chrono.ChMaterialSurfaceNSC())
gear1.SetMass(1)
gear1.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))


gear2 = chrono.ChBodyEasyCylinder(system, 1, 1, 100, True, True)
gear2.SetPos(chrono.ChVectorD(3, 0, 0))
gear2.SetMaterial(chrono.ChMaterialSurfaceNSC())
gear2.SetMass(1)
gear2.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(bar, gear1, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
motor.SetSpeed(10)


system.Add(ground)
system.Add(bar)
system.Add(gear1)
system.Add(gear2)
system.Add(motor)


dt = 0.01
t_end = 10


while system.GetChTime() < t_end:
    system.DoStepDynamics(dt)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()
    vis.Run()

vis.CloseWindow()