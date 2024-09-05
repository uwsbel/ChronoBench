import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.scm as scm


physics_system = chrono.ChSystemNSC()


visualization_system = chronoirr.ChIrrApp(physics_system, "HMMWV on SCM Terrain", chronoirr.Dim3d(800, 600))


visualization_system.SetFPS(50)