import pychrono as chrono


system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVectorD(0, 0, 0))  


ground = chrono.ChBodyEasyBox(4, 4, 0.1, 1000, True, False)
ground.SetPos(chrono.ChVectorD(0, 0, -0.1))
ground.SetFixed(True)
ground.GetVisualShape(0).SetColor(chrono.ChColor(0.3, 0.3, 0.3))
system.Add(ground)


bar = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, 0.05, 0.2, 1000, True, False)
bar.SetPos(chrono.ChVectorD(0, 0, 0))
bar.SetFixed(False)
bar.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.5, 0.8))
system.Add(bar)


rev_bar = chrono.ChLinkLockRevolute()
rev_bar.Initialize(ground, bar, chrono.ChFramed(chrono.ChVectorD(0, 0, 0)))
system.AddLink(rev_bar)


gear2 = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, 0.05, 0.1, 1000, True, False)
gear2.SetPos(chrono.ChVectorD(0.15, 0, 0))  
gear2.SetFixed(False)
gear2.GetVisualShape(0).SetColor(chrono.ChColor(0.8, 0.2, 0.2))
system.Add(gear2)


rev_gear2 = chrono.ChLinkLockRevolute()
rev_gear2.Initialize(ground, gear2, chrono.ChFramed(chrono.ChVectorD(0.15, 0, 0)))
system.AddLink(rev_gear2)


gear_trans = chrono.ChLinkGear()
gear_trans.Initialize(bar, gear2, chrono.ChFramed(chrono.ChVectorD(0, 0, 0)), chrono.ChFramed(chrono.ChVectorD(0.15, 0, 0)))
gear_trans.SetGearRatio(-1.0)
system.AddLink(gear_trans)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(ground, bar, chrono.ChFramed(chrono.ChVectorD(0, 0, 0)))
speed_func = chrono.ChFunctionConst(chrono.CH_PI / 2)  
motor.SetSpeedFunction(speed_func)
system.AddLink(motor)


visual_system = chrono.ChVisualSystemIrrlicht()
visual_system.AttachSystem(system)
visual_system.SetWindowSize(1024, 720)
visual_system.SetWindowTitle('Epicyclic Gears Simulation')
visual_system.Initialize()
visual_system.AddSkyBox()
visual_system.AddCamera(chrono.ChVectorD(0.5, 0.5, 0.8), chrono.ChVectorD(0, 0, 0))
visual_system.AddTypicalLights()


while visual_system.Run():
    visual_system.BeginScene()
    visual_system.Render()
    system.DoStepDynamics(0.01)
    visual_system.EndScene()