import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

# Initialize the PyChrono environment
chrono.ChSystemSMC()

# Initialize the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Gator Vehicle Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 2, -5), chrono.ChVectorD(0, 1, 0))
vis.AttachSystem(chrono.GetSystem())