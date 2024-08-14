import pychrono as chrono
import pychrono.irrlicht as chronoirr

# Initialize the PyChrono environment
chrono.ChSystemSMC()

# Create the Irrlicht visualization window
vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('PyChrono Simulation')
vis.Initialize()
vis.AddLogo(chronoirr.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chronoirr.vector3df(0, 0, -2))
vis.AttachSystem(sys)