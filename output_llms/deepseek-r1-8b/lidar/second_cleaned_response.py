import pychrono.core as chrono
import pychrono.sensor as sens

import numpy as np
import time
import math


def main():
    
    
    
    mphysicalSystem = chrono.ChSystemNSC()

    
    
    
    box_body = chrono.ChBodyEasyBox(side=1, side=1, side=1, mass=1000)
    box_body.SetPos(chrono.ChVector3d(0, 0, 0))
    box_body.SetName("Box")
    mphysicalSystem.Add(box_body)

    
    trimesh_shape = chrono.ChVisualShapeBox()
    trimesh_shape.SetName("Box Visual")
    trimesh_shape.SetMutable(False)
    box_body.AddVisualShape(trimesh_shape)

    
    manager = sens.ChSensorManager(mphysicalSystem)

    
    
    
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(-10, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    lidar_3d = sens.ChLidarSensor(
        box_body,              
        5.0,                  
        offset_pose,            
        800,                  
        1,                    
        2 * chrono.CH_PI,      
        0.1,                  
        100.0,                
        sens.LidarBeamShape_RECTANGULAR,  
        1.0,                  
        0.01,                 
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
        
        lidar_3d.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth Data"))

    
    lidar_3d.PushFilter(sens.ChFilterDIAccess())

    
    lidar_3d.PushFilter(sens.ChFilterPCfromDepth())

    if vis:
        
        lidar_3d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))

    
    lidar_3d.PushFilter(sens.ChFilterXYZIAccess())

    
    manager.AddSensor(lidar_3d)

    
    
    offset_pose_2d = chrono.ChFramed(
        chrono.ChVector3d(-5, 0, 0), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    lidar_2d = sens.ChLidarSensor(
        box_body,              
        5.0,                  
        offset_pose_2d,        
        800,                  
        1,                    
        2 * chrono.CH_PI / 4,  
        0.5 * chrono.CH_PI / 2,  
        100.0,                
        sens.LidarBeamShape_RECTANGULAR,  
        1.0,                  
        0.01,                 
        sens.LidarReturnMode_STRONGEST_RETURN  
    )
    lidar_2d.SetName("2D Lidar Sensor")
    lidar_2d.SetLag(lag)
    lidar_2d.SetCollectionWindow(collection_time)

    
    
    
    if noise_model == "CONST_NORMAL_XYZI":
        lidar_2d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    elif noise_model == "NONE":
        
        pass

    if vis:
        
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth Data"))

    
    lidar_2d.PushFilter(sens.ChFilterDIAccess())

    
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())

    if vis:
        
        lidar_2d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))

    
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())

    
    manager.AddSensor(lidar_2d)

    
    
    
    orbit_radius = 10
    orbit_rate = 0.1
    ch_time = 0.0

    render_time = 0
    t1 = time.time()

    while ch_time < end_time:
        
        lidar_3d.SetOffsetPose(
            chrono.ChFramed(
                chrono.ChVector3d(
                    -orbit_radius * math.cos(ch_time * orbit_rate),
                    -orbit_radius * math.sin(ch_time * orbit_rate),
                    1
                ),
                chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVector3d(0, 0, 1))
            )
        )

        
        xyzi_buffer = lidar_3d.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            print('XYZI buffer received from lidar. Lidar resolution: {0}x{1}'.format(xyzi_buffer.Width, xyzi_buffer.Height))
            print('Max Value: {0}'.format(np.max(xyzi_data)))

        
        manager.Update()

        
        mphysicalSystem.DoStepDynamics(step_size)

        
        ch_time = mphysicalSystem.GetChTime()

    print("Sim time:", end_time, "Wall time:", time.time() - t1)








noise_model = "NONE"  


return_mode = sens.LidarReturnMode_STRONGEST_RETURN





update_rate = 5.0


horizontal_samples = 800
vertical_samples = 1  


horizontal_fov = 2 * chrono.CH_PI / 4  
max_vert_angle = chrono.CH_PI / 2
min_vert_angle = -chrono.CH_PI / 2


lag = 0


collection_time = 1. / update_rate  


sample_radius = 1  


divergence_angle = 0.01  






step_size = 1e-3


end_time = 40.0


save = False


vis = True


out_dir = "SENSOR_OUTPUT/"






main()