import os
import math
import numpy as np
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea
import pychrono.robot as robosimian


chrono.SetChronoDataPath(os.path.join(os.getcwd(), '..', 'data'))


my_system = chrono.ChSystemSMC()
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(my_system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('HMMWV Vehicle Simulation')
vis.Initialize()


hmmwv = veh.HMMWV()
hmmwv.SetContactMethod(veh.ChVehicleContactMethod.SC_ELAS)
hmmwv.SetChassisCollisionType(veh.ChVehicleChassisCollisionType.SC_NOSURF)
hmmwv.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
hmmwv.Initialize()


terrain = veh.SCM_Terrain(hmmwv.GetSystem())
terrain.SetSoilParameters(1.5e6, 0.3, 0.05, 0.05)
terrain.SetHeightMap(chrono.GetChronoDataFile('heightmap.png'), 100, 100, 1, 1)
terrain.Initialize()


driver = veh.ChVehicleDriver()
driver.SetSpeed(20)
driver.SetSteering(0)
hmmwv.SetDriver(driver)


vis.AddCamera(chrono.ChVector3d(0, 10, -10))
vis.AddTypicalLights()


while vis.Run():
    
    my_system.DoStepDynamics(0.01)
    hmmwv.Synchronize()
    terrain.Synchronize()
    driver.Synchronize()
    vis.RunRealTime()
    vis.BeginScene()
    vis.Render()
    vis.EndScene()