import pychrono as chrono
import pychrono.irrlicht as chronoirr

# Initialize the PyChrono environment
chrono.ChSystemSMC()

# Create the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.Attach(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Tablecloth Folding Simulation')
vis.Initialize()