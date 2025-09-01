import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

# Create the Chrono::Engine physical system
physics_system = chrono.ChSystemNSC()

# Create the Irrlicht visualization system
visualization_system = chronoirr.ChIrrApp(physics_system, 'HMMWV Simulation', chronoirr.Dim3d(800, 600))

# Enable contact visualization
visualization_system.AddTypicalLogo(chronoirr.GetChronoDataFile('logo_pychrono_alpha.png'))
visualization_system.AddTypicalSky(chronoirr.GetChronoDataFile('skybox/dawnmountain_ft.jpg'))
visualization_system.AddTypicalLights()
visualization_system.AddTypicalCamera(chronoirr.vector3df(0, 10, -20))
visualization_system.AddTypicalFPS()
visualization_system.SetChaseCamera(chronoirr.ChTrackballWin(visualization_system.GetSystem(), visualization_system.GetVideoDriver()))
visualization_system.SetTryRealtime(True)