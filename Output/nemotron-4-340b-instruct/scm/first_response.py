import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.scm as scm

# Initialize the PyChrono environment
chrono.ChSystemSMC()

# Initialize the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(chrono.GetSystem())
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('HMMWV on SCM Deformable Terrain')
vis.Initialize()