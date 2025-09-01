import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as vehicle

import numpy as np
import time
import math

def main():
    
    
    
    mphysicalSystem = chrono.ChSystemNSC()

    
    
    
    artcar = vehicle.ChVehicleModelARTCar(mphysicalSystem)
    artcar.SetPos(chrono.ChVector3d(0, 0, 0))
    artcar.SetChassisBody(artcar.GetChassisBody())
    mphysicalSystem.Add(artcar.GetChassisBody())

    
    
    
    driver = vehicle.ChDriver(artcar)
    driver.SetAutoDrive(True)

    
    
    
    terrain = chrono.ChBodyEasyBox(100, 100, 1, 1000)
    terrain.SetPos(chrono.ChVector3d(0, 0, -1))
    terrain.GetVisualModel().GetShape(0).SetTexture(chrono.GetChronoDataFile("textures/grass.png"))
    terrain.GetVisualModel().GetShape(0).SetColor(chrono.ChColor(0.2, 0.8, 0.2))
    terrain.SetFixed(True)
    mphysicalSystem.Add(terrain)

    
    
    
    manager = sens.ChSensorManager(mphysicalSystem)

    
    
    
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(1.0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    lidar = sens.ChLidarSensor(
        artcar.GetChassisBody(),              
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
    lidar.SetName("3D Lidar Sensor")
    lidar.SetLag(lag)
    lidar.SetCollectionWindow(collection_time)
    if noise_model == "CONST_NORMAL_XYZI":
        lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    elif noise_model == "NONE":
        
        pass
    if vis:
        
        lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw 3D Lidar Depth Data"))
    
    lidar.PushFilter(sens.ChFilterDIAccess())
    
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    if vis:
        
        lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "3D Lidar Point Cloud"))
    
    lidar.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar)

    
    
    
    lidar_2d =  sens.ChLidarSensor(
        artcar.GetChassisBody(),              
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
        
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw 2D Lidar Depth Data"))
    
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    
    lidar_2d.Push
print("error happened with only start ```python")