import pychrono as chrono
import pychrono.irrlicht as chronoirr


physics = chrono.ChSystemNSC()


visualization = chronoirr.ChVisualSystemIrrlicht()


physics.SetVisualSystem(visualization)


visualization.SetWindowSize(800, 600)
visualization.SetWindowTitle("HMMWV on Complex Terrain")
visualization.Initialize()