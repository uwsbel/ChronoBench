import pychrono.core as chrono
import pychrono.sensor as sens

import numpy as np
import time
import math


def main():
    
    
    
    system = chrono.ChSystemNSC()

    
    
    
    side_length = 2.0
    density = 1000.0
    
    box_body = chrono.ChBodyEasyBox(
        side_length, side_length, side_length, density, True, False
    )
    box_body.SetPos(chrono.ChVectorD(0, 0, 0))
    box_body.SetBodyFixed(True)
    
    tex = chrono.ChTexture()
    tex.SetTextureFilename(chrono.GetChronoDataFile("textures/concrete.jpg"))
    box_body.AddAsset(tex)

    system.Add(box_body)

    
    
    
    manager = sens.ChSensorManager(system)

    
    
    
    
    offset3D = chrono.ChFrameD(
        chrono.ChVectorD(-12, 0, 1),
        chrono.ChQuaternionD(1, 0, 0, 0)
    )

    lidar_3d = sens.ChLidarSensor(
        box_body,                 
        update_rate,              
        offset3D,                 
        horizontal_samples,       
        vertical_samples,         
        horizontal_fov,           
        max_vert_angle,           
        min_vert_angle,           
        max_range,                
        sens.LidarBeamShape_RECTANGULAR,
        sample_radius,
        divergence_angle,
        divergence_angle,
        return_mode
    )
    lidar_3d.SetName("Lidar3D")
    lidar_3d.SetLag(lag)
    lidar_3d.SetCollectionWindow(collection_time)

    
    if noise_model == "CONST_NORMAL_XYZI":
        lidar_3d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))

    
    if vis:
        lidar_3d.PushFilter(sens.ChFilterVisualize(
            horizontal_samples, vertical_samples, "3D Lidar Depth"
        ))

    
    lidar_3d.PushFilter(sens.ChFilterDIAccess())
    lidar_3d.PushFilter(sens.ChFilterPCfromDepth())
    if vis:
        lidar_3d.PushFilter(sens.ChFilterVisualizePointCloud(
            640, 480, 1.0, "3D Point Cloud"
        ))
    lidar_3d.PushFilter(sens.ChFilterXYZIAccess())

    manager.AddSensor(lidar_3d)

    
    
    
    
    offset2D = chrono.ChFrameD(
        chrono.ChVectorD(0, -5, 1),
        chrono.ChQuaternionD(1, 0, 0, 0)
    )

    lidar_2d = sens.ChLidarSensor(
        box_body,                 
        update_rate,              
        offset2D,                 
        horizontal_samples,       
        1,                        
        horizontal_fov,           
        0.0,                      
        0.0,                      
        max_range,                
        sens.LidarBeamShape_RECTANGULAR,
        sample_radius,
        0.0,                      
        0.0,
        return_mode
    )
    lidar_2d.SetName("Lidar2D")
    lidar_2d.SetLag(lag)
    lidar_2d.SetCollectionWindow(collection_time)

    if noise_model == "CONST_NORMAL_XYZI":
        lidar_2d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))

    if vis:
        lidar_2d.PushFilter(sens.ChFilterVisualize(
            horizontal_samples, 1, "2D Lidar Depth"
        ))

    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    
    manager.AddSensor(lidar_2d)

    
    
    
    orbit_radius = 10
    orbit_rate = 0.1
    ch_time = 0.0

    t_start = time.time()
    while ch_time < end_time:
        
        angle = ch_time * orbit_rate
        pos = chrono.ChVector3d(
            -orbit_radius * math.cos(angle),
            -orbit_radius * math.sin(angle),
            1
        )
        quat = chrono.ChQuaternionD(math.cos(angle/2), 0, 0, math.sin(angle/2))
        new_pose = chrono.ChFrameD(pos, quat)

        lidar_3d.SetOffsetPose(new_pose)
        lidar_2d.SetOffsetPose(new_pose)

        
        buf3 = lidar_3d.GetMostRecentXYZIBuffer()
        if buf3.HasData():
            data3 = buf3.GetXYZIData()
            print(f"[3D] got buffer {buf3.Width}×{buf3.Height}, max={np.max(data3)}")

        
        buf2 = lidar_2d.GetMostRecentDepthBuffer()
        if buf2.HasData():
            d2 = buf2.GetDepthData()
            print(f"[2D] got depth array length {buf2.Width}, max={np.max(d2)}")

        
        manager.Update()
        system.DoStepDynamics(step_size)
        ch_time = system.GetChTime()

    print("Sim time:", end_time, "Wall time:", time.time() - t_start)





noise_model = "NONE"                 
return_mode = sens.LidarReturnMode_STRONGEST_RETURN
update_rate = 5.0
horizontal_samples = 800
vertical_samples = 300
horizontal_fov = 2 * chrono.CH_PI
max_vert_angle = chrono.CH_PI / 12
min_vert_angle = -chrono.CH_PI / 6
max_range = 100.0
sample_radius = 2
divergence_angle = 0.003
lag = 0
collection_time = 1.0 / update_rate




step_size = 1e-3
end_time = 40.0
vis = True

if __name__ == "__main__":
    
    
    main()