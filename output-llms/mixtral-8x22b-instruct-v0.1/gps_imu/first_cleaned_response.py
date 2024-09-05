import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


chrono.SetChronoDataPath("/path/to/chrono/data")
chrono.SetChronoDataPath("/path/to/irrlicht/data")


vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(800, 600)
vis.SetWindowTitle("HMMWV Simulation")
vis.Initialize()
vis.AddLogo(chronoirr.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(0, 5, -10))