import pychrono.core as chrono
import pychrono.sensor as sens
import numpy as np
import time
import math




step_size = 1e-3
end_time = 40.0
vis = True




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




vertical_samples_2d = 1  
min_vert_angle_2d = 0    
max_vert_angle_2d = 0    




box_size = 2.0  

def main():
    
    mphysicalSystem = chrono.ChSystemNSC()

    
    box_body = chrono.ChBodyEasyBox(box_size, box_size, box_size, 1000)  
    box_body.SetPos(chrono.ChVector3d(0, 0, 0))
    box_body.SetFixed(True)
    
    
    box_visual = chrono.ChVisualShapeBox(box_size, box_size, box_size)
    box_visual.SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))
    box_body.AddVisualShape(box_visual)
    mphysicalSystem.Add(box_body)

    
    manager = sens.ChSensorManager(mphysicalSystem)

    
    
    
    lidar_offset_pose = chrono.ChFramed(
        chrono.ChVector3d(-12, 0, 1), 
        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    lidar = sens.ChLidarSensor(
        box_body,               
        update_rate,            
        lidar_offset_pose,      
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
        lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "3D Lidar Depth"))
    
    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    
    if vis:
        lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "3D Lidar Point Cloud"))
    
    lidar.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar)

    
    
    
    lidar2d_offset_pose = chrono.ChFramed(
        chrono.ChVector3d(0, 0, 1),  
        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    lidar2d = sens.ChLidarSensor(
        box_body,               
        update_rate,            
        lidar2d_offset_pose,    
        horizontal_samples,     
        vertical_samples_2d,    
        horizontal_fov,         
        max_vert_angle_2d,      
        min_vert_angle_2d,      
        100.0,                  
        sens.LidarBeamShape_RECTANGULAR,
        sample_radius,
        divergence_angle,
        divergence_angle,
        return_mode
    )
    lidar2d.SetName("2D Lidar")
    lidar2d.SetLag(lag)
    lidar2d.SetCollectionWindow(collection_time)

    
    if noise_model == "CONST_NORMAL_XYZI":
        lidar2d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    
    lidar2d.PushFilter(sens.ChFilterDIAccess())
    lidar2d.PushFilter(sens.ChFilterPCfromDepth())
    
    if vis:
        lidar2d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "2D Lidar Point Cloud"))
    
    lidar2d.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar2d)

    
    orbit_radius = 10
    orbit_rate = 0.1
    ch_time = 0.0
    t_start = time.time()

    while ch_time < end_time:
        
        lidar.SetOffsetPose(chrono.ChFramed(
            chrono.ChVector3d(
                -orbit_radius * math.cos(ch_time * orbit_rate),
                -orbit_radius * math.sin(ch_time * orbit_rate),
                1
            ),
            chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVector3d(0, 0, 1))
        ))

        
        if lidar.GetMostRecentXYZIBuffer().HasData():
            xyzi_data = lidar.GetMostRecentXYZIBuffer().GetXYZIData()
            print(f"3D Lidar: Max intensity = {np.max(xyzi_data[:,3]):.4f}")

        
        if lidar2d.GetMostRecentXYZIBuffer().HasData():
            xyzi_data_2d = lidar2d.GetMostRecentXYZIBuffer().GetXYZIData()
            print(f"2D Lidar: Points captured = {len(xyzi_data_2d)}")

        
        manager.Update()

        
        mphysicalSystem.DoStepDynamics(step_size)
        ch_time = mphysicalSystem.GetChTime()

    print(f"Simulation finished in {time.time()-t_start:.2f} seconds")

if __name__ == "__main__":
    main()