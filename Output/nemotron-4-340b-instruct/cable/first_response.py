import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np

# Initialize the PyChrono environment
chrono.ChSystemSMC()

# Initialize the Irrlicht visualization engine
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(chrono.GetSystem())
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('ANCF Cable Beam Simulation')
vis.Initialize()