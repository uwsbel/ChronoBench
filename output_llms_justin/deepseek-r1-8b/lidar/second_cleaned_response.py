import pychrono.core as chrono
import pychrono.sensor as sens

import numpy as np
import time
import math


def main():
    
    
    
    mphysicalSystem = chrono.ChSystemNSC()

    
    
    
    side = 2.0  
    box_body = chrono.ChBodyEasyBox(side, side, side, 1000.0)
    box_body.SetPos(chrono.ChVector3d(0, 0, 0))
    box_body.SetName("Chassis Box")
    
    
    default_texture = chrono.ChVisualShapeBox()
    default_texture.SetName("Default Box Texture")
    box_body.AddVisualShape(default_texture)
    mphysicalSystem.Add(box_body)

    
    
    
    manager = sens.ChSensorManager(mphysicalSystem)

    
    
    
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(-12, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    lidar = sens.ChLidarSensor(
        box_body,              
        5.0,                  
        offset_pose,            
        800,                  
        1,                    
        2 * chrono.CH_PI,     
        0.1,                  
        0,                    
        100.0,                
        sens.LidarBeamShape_RECTANGULAR,  
        2,                    
        0.002,                
        0.002,                
        sens.LidarReturnMode_STRONGEST_RETURN  
    )
    lidar.SetName("3D Lidar Sensor")
    lidar.SetLag(0)
    lidar.SetCollectionWindow(1.0 / 5.0)  

    
    
    
    lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples=800, vertical_samples=1, name="3D Lidar Depth Data"))

    lidar.PushFilter(sens.ChFilterPCfromDepth())

    if True:  
        lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "3D Lidar Point Cloud"))

    
    lidar.PushFilter(sens.ChFilterDIAccess())

    
    lidar.PushFilter(sens.ChFilterXYZIAccess())

    
    manager.AddSensor(lidar)

    
    
    
    offset_pose_2d = chrono.ChFramed(
        chrono.ChVector3d(-12, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    lidar_2d = sens.ChLidarSensor(
        box_body,              
        5.0,                  
        offset_pose_2d,        
        1,                    
        1,                    
        2 * chrono.CH_PI / 2,  
        0.1,                  
        0,                    
        100.0,                
        sens.LidarBeamShape_RECTANGULAR,  
        2,                    
        0.002,                
        0.002,                
        sens.LidarReturnMode_STRONGEST_RETURN  
    )
    lidar_2d.SetName("2D Lidar Sensor")
    lidar_2d.SetLag(0)
    lidar_2d.SetCollectionWindow(1.0 / 5.0)  

    
    
    
    lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples=1, vertical_samples=1, name="2D Lidar Depth Data"))

    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())

    if True:  
        lidar_2d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "2D Lidar Point Cloud"))

    
    lidar_2d.PushFilter(sens.ChFilterDIAccess())

    
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())

    
    manager.AddSensor(lidar_2d)

    
    
    
    orbit_radius = 10
    orbit_rate = 0.1
    ch_time = 0.0

    render_time = 0
    t1 = time.time()

    while ch_time < end_time:
        
        lidar.SetOffsetPose(
            chrono.ChFramed(
                chrono.ChVector3d(
                    -orbit_radius * math.cos(ch_time * orbit_rate),
                    -orbit_radius * math.sin(ch_time * orbit_rate),
                    1
                ),
                chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVector3d(0, 0, 1))
            )
        )

        
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            print('XYZI buffer received from 3D lidar. Lidar resolution: {0}x{1}'.format(xyzi_buffer.Width, xyzi_buffer.Height))
            print('Max Value: {0}'.format(np.max(xyzi_data)))

        
        xyzi_buffer_2d = lidar_2d.GetMostRecentXYZIBuffer()
        if xyzi_buffer_2d.HasData():
            xyzi_data_2d = xyzi_buffer_2d.GetXYZIData()
            print('XYZI buffer received from 2D lidar. Lidar resolution: {0}x{1}'.format(xyzi_buffer_2d.Width, xyzi_buffer_2d.Height))
            print('Max Value: {0}'.format(np.max(xyzi_data_2d)))

        
        manager.Update()

        
        mphysicalSystem.DoStepDynamics(step_size)

        
        ch_time = mphysicalSystem.GetChTime()

    print("Sim time:", end_time, "Wall time:", time.time() - t1)









noise_model = "NONE"  


return_mode = sens.LidarReturnMode_STRONGEST_RETURN





update_rate = 5.0


horizontal_samples = 800
vertical_samples = 1  


horizontal_fov = 2 * chrono.CH_PI / 2  
max_vert_angle = chrono.CH_PI / 12
min_vert_angle = -chrono.CH_PI / 6


lag = 0


collection_time = 1.0 / update_rate  


sample_radius = 2


divergence_angle = 0.003






step_size = 1e-3


end_time = 40.0


save = False


vis = True


out_dir = "SENSOR_OUTPUT/"






main()