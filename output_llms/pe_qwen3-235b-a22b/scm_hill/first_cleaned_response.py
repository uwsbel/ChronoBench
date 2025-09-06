import os
import math
import numpy as np
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "..", "..", "data"))


my_system = chrono.ChSystemSMC()


terrain = veh.SCMDeformableTerrain(my_system)
terrain.SetPlane(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
terrain.SetSoilParameters(
    2e6,    
    0,      
    1e4,    
    0,      
    30,     
    0.01,   
    4e5,    
    2e4     
)

terrain.Initialize()
terrain.SetPlotType(veh.SCMDeformableTerrain.PLOT_PRESSURE, 0, 300)


vehicle = veh.HMMWV(my_system)
vehicle.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0.5, 0), chrono.QuatFromAngleZ(0)))
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
vehicle.SetChassisCollisionType(veh.ChassisCollisionType_PRIMITIVES)
vehicle.Initialize()


driver = veh.ChDriver(vehicle)
driver.SetThrottle(0.5)  
driver.Initialize()


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(my_system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('HMMWV on SCM Deformable Terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 6))  
vis.AddTypicalLights()


time_step = 0.01
realtime_timer = chrono.ChRealtimeStepTimer()


while vis.Run():
    time = my_system.GetChTime()
    
    
    driver.Synchronize(time)
    vehicle.Synchronize(time, driver.GetInputs(), terrain)
    terrain.Synchronize()
    
    
    my_system.DoStepDynamics(time_step)
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    realtime_timer.Spin(time_step)