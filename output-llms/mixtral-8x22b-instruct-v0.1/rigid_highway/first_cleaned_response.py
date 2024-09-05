import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


physics_system = chrono.ChSystemNSC()


visualization_system = chronoirr.ChIrrApp(physics_system, 'HMMWV Simulation', chronoirr.Dim3d(800, 600))


visualization_system.AddTypicalLogo(chronoirr.GetChronoDataFile('logo_pychrono_alpha.png'))
visualization_system.AddTypicalSky(chronoirr.GetChronoDataFile('skybox/dawnmountain_ft.jpg'))
visualization_system.AddTypicalLights()
visualization_system.AddTypicalCamera(chronoirr.vector3df(0, 10, -20))
visualization_system.AddTypicalFPS()
visualization_system.SetChaseCamera(chronoirr.ChTrackballWin(visualization_system.GetSystem(), visualization_system.GetVideoDriver()))
visualization_system.SetTryRealtime(True)