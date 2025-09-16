import pychrono


pychrono.SetChronoDataPath("path_to_chrono_data")
pychrono.SetChronoOutputLevel(pychrono.LogLevel.LOG_WARNING)


system = pychrono.ChSystemSMC()








visualization = pychrono.ChVisualizationIrrlicht()
visualization.AttachSystem(system)
visualization.SetWindowSize(800, 600)
visualization.SetWindowTitle("Beam Finite Elements Simulation")
visualization.SetRenderMode(pychrono.ChVisualizationIrrlicht.RENDER_MODE_OPENGL)


while system.GetTime() < 10.0:
    system.DoStepDynamics(0.001)
    visualization.Render()


system.Clear()