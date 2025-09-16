```python
import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as vehicle

import numpy as np
import time
import math

def main():
    
    
    
    mphysicalSystem = chrono.ChSystemNSC()
    
    
    vehicle_model = vehicle.ChVehicleModel()
    vehicle_model.SetChassis(vehicle.ChVehicleChassis(vehicle_model))
    vehicle_model.SetWheels(
        [
            vehicle.ChVehicleWheel(vehicle_model, 0.2, 0.1, 0.1),
            vehicle.ChVehicleWheel(vehicle_model, 0.2, 0.1, 0.1),
            vehicle.ChVehicleWheel(vehicle_model, 0.2, 0.1, 0.1),
            vehicle.ChVehicleWheel(vehicle_model, 0.2, 0.1, 0.1),
        ]
    )
    
    vehicle_system = vehicle.ChVehicleSystem(mphysicalSystem, vehicle_model)
    
    
    driver = vehicle.ChVehicleDriver(vehicle_system)
    driver.SetDriverType(vehicle.ChVehicleDriverType_PID)
    driver.SetController(vehicle.ChVehicleController_PID(
        kp=1.0, ki=0.1, kd=0.01
    ))
    
    
    terrain = chrono.ChBodyEasyBox(100, 100, 1, 1000)
    terrain.SetPos(chrono.ChVector3d(0, 0, -1))
    terrain.GetVisualModel().GetShape(0).SetTexture(chrono.GetChronoDataFile("textures/grass.png"))
    terrain.SetBodyFixed(True)
    mphysicalSystem.Add(terrain)
    
    
    
    
    manager = sens.ChSensorManager(mphysicalSystem)

    
    
    
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(1.0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    lidar = sens.ChLidarSensor(
        vehicle_system.GetChassis(),              
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
        
        lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
    
    lidar.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar)

    
    lidar_2d =  sens.ChLidarSensor(
        vehicle_system.GetChassis(),              
        update_rate,            
        offset_pose,            
        horizontal_samples,     
        1,                      
        horizontal_fov,         
        0.0,                    
        0.0,                    
        100.0,                  
        sens.LidarBeamShape_RECTANGULAR,  
        sample_radius,          
        divergence_angle,       
        divergence_angle,       
        return_mode             
    )
    lidar_2d.SetName("2D Lidar Sensor")
    lidar_2d.SetLag(lag)
    lidar_2d.SetCollectionWindow(collection_time)
    if noise_model == "CONST_NORMAL_XYZI":
        lidar_2d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    elif noise_model == "NONE":
        
        pass
    if vis:
        
        lidar_2d.PushFilter(sens