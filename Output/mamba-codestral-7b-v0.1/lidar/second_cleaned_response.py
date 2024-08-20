import pychrono.core as chrono
import pychrono.sensor as sens

import numpy as np
import time
import math

def main():
    
    
    
    mphysicalSystem = chrono.ChSystemNSC()

    
    
    
    side = 1.0
    mbox = chrono.ChBodyEasyBox(side, side, side, 1000)
    mbox.SetPos(chrono.ChVector3d(0, 0, 0))
    mbox.SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"), True)
    mphysicalSystem.Add(mbox)

    
    
    
    manager = sens.ChSensorManager(mphysicalSystem)

    
    
    
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(-12, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    lidar = sens.ChLidarSensor(
        mbox,                   
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

    
    lidar_2d = sens.ChLidarSensor(
        mbox,                   
        update_rate,            
        offset_pose,            
        1,                      
        1,                      
        math.radians(360),      
        math.radians(1),        
        math.radians(1),        
        100.0,                  
        sens.LidarBeamShape_RECTANGULAR,  
        sample_radius,          
        divergence_angle,       
        divergence_angle,       
        return_mode             
    )
    lidar_2d.SetName("Lidar Sensor 2D")
    lidar_2d.SetLag(lag)
    lidar_2d.SetCollectionWindow(collection_time)

    
    
    
    if noise_model == "CONST_NORMAL_XYZI":
        lidar_2d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    elif noise_model == "NONE":
        
        pass

    if vis:
        
        lidar_2d.PushFilter(sens.ChFilterVisualize(1, 1, "Raw Lidar Depth Data 2D"))

    
    lidar_2d.PushFilter(sens.ChFilterDIAccess())

    
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())

    if vis:
        
        lidar_2d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud 2D"))

    
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
print("error happened with only start ```python")