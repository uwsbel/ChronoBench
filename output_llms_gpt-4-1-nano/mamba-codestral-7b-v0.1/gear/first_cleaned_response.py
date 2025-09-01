import pychrono


system = pychrono.ChSystemSMC()



fixed_truss = pychrono.ChBodyEasyBox(1, 1, 1, 1000, True, True)
fixed_truss.SetPos(pychrono.ChVectorD(0, 0, 0))
system.AddBody(fixed_truss)


rotating_bar = pychrono.ChBodyEasyBox(1, 1, 1, 1000, True, True)
rotating_bar.SetPos(pychrono.ChVectorD(2, 0, 0))
system.AddBody(rotating_bar)


gear1 = pychrono.ChBodyEasyBox(1, 1, 1, 1000, True, True)
gear1.SetPos(pychrono.ChVectorD(4, 0, 0))
system.AddBody(gear1)


gear2 = pychrono.ChBodyEasyBox(1, 1, 1, 1000, True, True)
gear2.SetPos(pychrono.ChVectorD(6, 0, 0))
system.AddBody(gear2)


gear_motor = pychrono.ChBodyEasyBox(1, 1, 1, 1000, True, True)
gear_motor.SetPos(pychrono.ChVectorD(8, 0, 0))
system.AddBody(gear_motor)





visualization = pychrono.ChVisualSystemIrrlicht()
visualization.AttachSystem(system)
visualization.SetWindowSize(800, 600)
visualization.SetWindowTitle('Epicyclic Gears Simulation')
visualization.SetRenderMode(pychrono.ChVisualization.eSHADED, 40)
visualization.Initialize()
visualization.AddLogo()
visualization.AddCamera(pychrono.ChVectorD(10, 10, 10), pychrono.ChVectorD(0, 0, 0))


while system.GetTime() < 10:
    visualization.BeginScene()
    visualization.Render()
    visualization.EndScene()
    system.DoStepDynamics(0.01)


visualization.Close()