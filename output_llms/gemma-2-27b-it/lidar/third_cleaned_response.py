```python
import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

import numpy as np
import time
import math






step_size = 1e-3


end_time = 40.0


save = False


vis = True


out_dir = "SENSOR_OUTPUT/"













noise_model = "NONE"  


return_mode = sens.LidarReturnMode_STRONGEST_RETURN





update_rate = 5.0


horizontal_samples = 800
vertical_samples = 300


horizontal_fov = 2 * chrono.CH_PI  
max_vert_angle = chrono.CH_PI / 12
min_vert_angle = -chrono.CH_PI / 6


lag = 0


collection_time = 1. / update_rate  


sample_radius = 2


divergence_angle = 0.003

def main():
    
    
    
    system = chrono.ChSystemNSC()
    vehicle = veh.ARTcar(system)
    vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)

    
    
    
    driver = veh.ChDriver(vehicle.GetChassis())
    vehicle.SetDriver(driver)

    
    
    
    terrain = chrono.ChRigidBody()
    terrain.SetBodyFixed(True)
    terrain.SetCollide(True)
    terrain.GetCollisionModel().ClearModel()
    terrain.GetCollisionModel().AddBox(chrono.ChVector3d(100, 100, 0.1), chrono.ChVector3d(0, 0, -0.05))
    terrain.GetCollisionModel().BuildModel()
    terrain.SetMaterialSurface(chrono.ChMaterialSurfaceNSC.CreateDefault())
    terrain.GetVisualModel().SetTexture(chrono.GetChronoDataFile("textures/grass.png"))
    terrain.GetVisualModel().SetMaterialColor(chrono.ChColor(0.5, 0.5, 0.5))
    system.Add(terrain)

    
    
    
    manager = sens.ChSensorManager(system)

    
    
    
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(1.0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    lidar = sens.ChLidarSensor(
        vehicle.GetChassisBody(),              
        update_rate,            
        offset_pose,            
        horizontal_samples,     
        vertical_samples,       
        horizontal_fov,         
        max_vert_angle,         
        min_vert_angle,         
        100.0,                  
        sens.LidarBeamShape_RECTANGULAR,  
        sample_radius,          
        divergence_angle,       
        divergence_angle,       
        return_mode             
    )
    lidar.SetName("Lidar Sensor")
    lidar.SetLag(lag)
    lidar.SetCollectionWindow(collection_time)

    
    
    
    if noise_model == "CONST_NORMAL_XYZI":
        lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    elif noise_model == "NONE":
        
        pass
    if vis:
        
        lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth Data"))
    
    lidar.PushFilter(sens.ChFilterDIAccess())
    
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    if vis: