import pychrono.core as chrono
import pychrono.sensor as sens

import numpy as np
import time
import math




box_side = 10.0  

def main():
    
    
    
    mphysicalSystem = chrono.ChSystemNSC()

    
    
    
    
    box_body = chrono.ChBodyEasyBox(box_side / 2, box_side / 2, box_side / 2, 
                                    1000,      
                                    True,      
                                    True)      
    box_body.SetPos(chrono.ChVector3d(0, 0, 0)) 
    box_body.SetFixed(True)                   
    mphysicalSystem.Add(box_body)

    
    
    
    vis_asset_found = False
    for asset in box_body.GetAssets():
        if isinstance(asset, chrono.ChVisualShape):
            
            asset.SetColor(chrono.ChColor(0.5, 0.5, 0.6)) 
            
            
            
            
            
            
            
            
            vis_asset_found = True
            break
    if not vis_asset_found:
        print("Warning: Could not find visual asset to set color/texture for the box.")


    
    
    
    manager = sens.ChSensorManager(mphysicalSystem)
    
    

    
    
    
    
    initial_offset_pose_3d = chrono.ChFramed(
        chrono.ChVector3d(-12, 0, 1), 
        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)) 
    )
    lidar_3d = sens.ChLidarSensor(
        box_body,               
        update_rate,            
        initial_offset_pose_3d, 
        horizontal_samples,     
        vertical_samples,       
        horizontal_fov,         
        max_vert_angle,         
        min_vert_angle,         
        100.0,                  
        sens.LidarBeamShape_RECTANGULAR,  
        sample_radius,          
        sample_radius,          
        divergence_angle,       
        divergence_angle,       
        return_mode             
    )
    lidar_3d.SetName("3D Lidar Sensor")
    lidar_3d.SetLag(lag)
    lidar_3d.SetCollectionWindow(collection_time)

    
    
    
    if noise_model == "CONST_NORMAL_XYZI":
        
        lidar_3d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    elif noise_model == "NONE":
        pass 

    if vis:
        
        lidar_3d.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "3D Lidar Raw Depth Data"))

    
    lidar_3d.PushFilter(sens.ChFilterDIAccess())

    
    lidar_3d.PushFilter(sens.ChFilterPCfromDepth())

    if vis:
        
        lidar_3d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "3D L
print("error happened with only start ```python")