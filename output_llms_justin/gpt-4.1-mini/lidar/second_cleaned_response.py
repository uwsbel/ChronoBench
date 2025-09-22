import pychrono.core as chrono
import pychrono.sensor as sens

import numpy as np
import time
import math


def main():
    
    
    
    mphysicalSystem = chrono.ChSystemNSC()

    
    
    
    side = 2.0  
    density = 1000  

    box_body = chrono.ChBodyEasyBox(side, side, side, density, True, True)
    box_body.SetPos(chrono.ChVector3d(0, 0, 0))
    box_body.SetBodyFixed(True)
    
    box_body.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("sensor/textures/box_texture.png"))
    mphysicalSystem.Add(box_body)

    
    
    
    manager = sens.ChSensorManager(mphysicalSystem)

    
    
    
    offset_pose = chrono.ChFrameD(
        chrono.ChVectorD(-12, 0, 1), chrono.ChQuaternionD(1, 0, 0, 0)
    )

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

    
    
    
    
    horizontal_samples_2d = 800
    vertical_samples_2d = 1
    horizontal_fov_2d = 2 * chrono.CH_C_PI  
    max_vert_angle_2d = 0
    min_vert_angle_2d = 0
    max_range_2d = 50.0
    sample_radius_2d = 1
    divergence_angle_2d = 0.003

    offset_pose_2d = chrono.ChFrameD(
        chrono.ChVectorD(0, 0, 1), chrono.ChQuaternionD(1, 0, 0, 0)
    )

    lidar_2d = sens.ChLidarSensor(
        box_body,
        update_rate,
        offset_pose_2d,
        horizontal_samples_2d,
        vertical_samples_2d,
        horizontal_fov_2d,
        max_vert_angle_2d,
        min_vert_angle_2d,
        max_range_2d,
        sens.LidarBeamShape_RECTANGULAR,
        sample_radius_2d,
        divergence_angle_2d,
        divergence_angle_2d,
        return_mode
    )
    lidar_2d.SetName("2D Lidar Sensor")
    lidar_2d.SetLag(lag)
    lidar_2d.SetCollectionWindow(collection_time)

    
    if noise_model == "CONST_NORMAL_XYZI":
        lidar_2d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))

    if vis:
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples_2d, vertical_samples_2d, "Raw 2D Lidar Depth Data"))

    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())

    if vis:
        lidar_2d.PushFilter(sens.ChFilterVisualizePointCloud(640, 80, 1.0, "2D Lidar Point Cloud"))

    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())

    manager.AddSensor(lidar_2d)

    
    
    
    orbit_radius = 10
    orbit_rate = 0.1
    ch_time = 0.0

    t1 = time.time()

    while ch_time < end_time:
        
        pose_offset_3d = chrono.ChFrameD(
            chrono.ChVectorD(
                -orbit_radius * math.cos(ch_time * orbit_rate),
                -orbit_radius * math.sin(ch_time * orbit_rate),
                1
            ),
            chrono.ChQuaternionD().Q_from_AngAxis(ch_time * orbit_rate, chrono.ChVectorD(0, 0, 1))
        )
        lidar.SetOffsetPose(pose_offset_3d)

        
        pose_offset_2d = chrono.ChFrameD(
            chrono.ChVectorD(
                -orbit_radius * math.cos(ch_time * orbit_rate),
                -orbit_radius * math.sin(ch_time * orbit_rate),
                1
            ),
            chrono.ChQuaternionD().Q_from_AngAxis(ch_time * orbit_rate, chrono.ChVectorD(0, 0, 1))
        )
        lidar_2d.SetOffsetPose(pose_offset_2d)

        
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer is not None and xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            print('3D Lidar - XYZI buffer received. Resolution: {0}x{1}'.format(xyzi_buffer.Width, xyzi_buffer.Height))
            print('Max Value:', np.max(xyzi_data))

        
        xyzi_buffer_2d = lidar_2d.GetMostRecentXYZIBuffer()
        if xyzi_buffer_2d is not None and xyzi_buffer_2d.HasData():
            xyzi_data_2d = xyzi_buffer_2d.GetXYZIData()
            print('2D Lidar - XYZI buffer received. Resolution: {0}x{1}'.format(xyzi_buffer_2d.Width, xyzi_buffer_2d.Height))
            print('Max Value 2D:', np.max(xyzi_data_2d))

        
        manager.Update()

        
        mphysicalSystem.DoStepDynamics(step_size)

        
        ch_time = mphysicalSystem.GetChTime()

    print("Sim time:", end_time, "Wall time:", time.time() - t1)









noise_model = "NONE"  


return_mode = sens.LidarReturnMode_STRONGEST_RETURN





update_rate = 5.0


horizontal_samples = 800
vertical_samples = 300


horizontal_fov = 2 * chrono.CH_C_PI  
max_vert_angle = chrono.CH_C_PI / 12
min_vert_angle = -chrono.CH_C_PI / 6


lag = 0


collection_time = 1. / update_rate  


sample_radius = 2


divergence_angle = 0.003






step_size = 1e-3


end_time = 40.0


save = False


vis = True


out_dir = "SENSOR_OUTPUT/"






if __name__ == "__main__":
    
    main()