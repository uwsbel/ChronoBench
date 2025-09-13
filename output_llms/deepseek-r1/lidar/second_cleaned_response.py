import pychrono.core as chrono
import pychrono.sensor as sens
import numpy as np
import time
import math




noise_model = "NONE"  
return_mode = sens.LidarReturnMode_STRONGEST_RETURN
update_rate = 5.0  
horizontal_samples = 800
vertical_samples = 300
horizontal_fov = 2 * chrono.CH_PI  
max_vert_angle = chrono.CH_PI / 12
min_vert_angle = -chrono.CH_PI / 6
lag = 0  
collection_time = 1.0 / update_rate
sample_radius = 2
divergence_angle = 0.003  




step_size = 1e-3  
end_time = 40.0  
vis = True  
box_size = 4.0  

def main():
    
    
    
    mphysicalSystem = chrono.ChSystemNSC()

    
    
    
    box_body = chrono.ChBodyEasyBox(box_size, box_size, box_size, 1000)  
    box_body.SetPos(chrono.ChVector3d(0, 0, 0))
    box_body.SetFixed(True)
    mphysicalSystem.Add(box_body)

    
    box_vis = chrono.ChVisualShapeBox(box_size, box_size, box_size)
    box_vis.SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))
    box_body.AddVisualShape(box_vis)

    
    
    
    manager = sens.ChSensorManager(mphysicalSystem)

    
    
    
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(-12, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    
    lidar = sens.ChLidarSensor(
        box_body,               
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
    lidar.SetName("3D Lidar")
    lidar.SetLag(lag)
    lidar.SetCollectionWindow(collection_time)

    
    if noise_model == "CONST_NORMAL_XYZI":
        lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    if vis:
        lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "3D Lidar - Raw Data"))
    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    if vis:
        lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "3D Lidar - Point Cloud"))
    lidar.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar)

    
    
    
    offset_pose_2d = chrono.ChFramed(
        chrono.ChVector3d(-12, 0, 1.5),  
        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    
    lidar_2d = sens.ChLidarSensor(
        box_body,               
        update_rate,            
        offset_pose_2d,         
        horizontal_samples,     
        1,                      
        horizontal_fov,         
        0,                      
        0,                      
        100.0,                  
        sens.LidarBeamShape_RECTANGULAR,
        sample_radius,
        divergence_angle,
        divergence_angle,
        return_mode
    )
    lidar_2d.SetName("2D Lidar")
    lidar_2d.SetLag(lag)
    lidar_2d.SetCollectionWindow(collection_time)

    
    if noise_model == "CONST_NORMAL_XYZI":
        lidar_2d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    if vis:
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, 1, "2D Lidar - Raw Data"))
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    if vis:
        lidar_2d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "2D Lidar - Point Cloud"))
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar_2d)

    
    
    
    orbit_radius = 10
    orbit_rate = 0.1
    ch_time = 0.0
    t1 = time.time()

    while ch_time < end_time:
        
        angle = ch_time * orbit_rate
        new_pos = chrono.ChVector3d(
            -orbit_radius * math.cos(angle),
            -orbit_radius * math.sin(angle),
            1  
        )
        new_rot = chrono.QuatFromAngleAxis(angle, chrono.ChVector3d(0, 0, 1))
        
        
        lidar.SetOffsetPose(chrono.ChFramed(new_pos, new_rot))
        
        
        lidar_2d.SetOffsetPose(chrono.ChFramed(
            new_pos + chrono.ChVector3d(0, 0, 0.5),  
            new_rot
        ))

        
        manager.Update()
        
        
        mphysicalSystem.DoStepDynamics(step_size)
        ch_time = mphysicalSystem.GetChTime()

    print(f"Simulation completed in {time.time()-t1:.2f} seconds")

if __name__ == "__main__":
    main()