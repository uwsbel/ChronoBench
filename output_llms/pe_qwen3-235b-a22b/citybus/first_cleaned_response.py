import os
import math
import numpy as np
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath(os.environ.get('CHRONO_DATA_DIR', ''))


sys = chrono.ChSystemSMC()


bus = veh.CityBus(sys)
bus.SetContactMethod(chrono.ChContactMethod_SMC)
bus.SetChassisCollisionType(veh.ChassisCollisionType_MESH)
bus.SetChassisFixed(False)
bus.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT))
bus.SetTireType(veh.TireModelType_TMEASY)
bus.SetTireStepSize(0.005)
bus.Initialize()


terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChMaterialSurfaceSMC()
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysD(), 200, 200)
patch.SetTexture(chrono.GetChronoDataFile("terrain/textures/concrete.jpg"), 200, 200)
terrain.Initialize()


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('CityBus Simulation')
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddTypicalLights()


camera_pos = chrono.ChVector3d(0, 3, -6)
vis.AddCamera(camera_pos, bus.GetChassis().GetPos())


driver = veh.VehicleDriver(bus)
driver.SetSteeringDelta(0.05)
driver.SetThrottleDelta(0.05)
driver.SetBrakingDelta(0.05)
driver.Initialize()


step_timer = chrono.ChRealtimeStepTimer()
time_step = 0.02  


while vis.Run():
    time = sys.GetChTime()
    
    
    driver_inputs = driver.GetInputs()
    driver.Synchronize(time)
    
    
    terrain.Synchronize(time)
    bus.Synchronize(time, driver_inputs, terrain)
    
    
    sys.DoStepDynamics(time_step)
    
    
    chassis_pos = bus.GetChassis().GetPos()
    new_camera_pos = chassis_pos + chrono.ChVector3d(0, 3, -6)
    vis.GetSceneManager().getActiveCamera().setPosition(new_camera_pos)
    vis.GetSceneManager().getActiveCamera().setTarget(chassis_pos)
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()