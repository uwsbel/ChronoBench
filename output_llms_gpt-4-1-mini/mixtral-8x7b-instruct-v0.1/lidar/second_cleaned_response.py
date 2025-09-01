import pychrono.core as chrono
import pychrono.sensor as sens

import numpy as np
import time
import math


def main():
    
    
    
    mphysicalSystem = chrono.ChSystemNSC()

    
    
    
    side = 1.0
    box_body = chrono.ChBodyEasyBox(side, side, side, 1000, True, True)
    box_body.SetPos(chrono.ChVector3d(0, 0, 0))
    box_body.SetPose(chrono.ChQuaternion(1, 0, 0, 0))
    box_body.SetCollide(True)
    box_body.SetMaterialSurface(chrono.ChMaterialSurface.BoxMat)
    mphysicalSystem.Add(box_body)

    
    box_shape = chrono.ChBoxShape()
    box_shape.SetBox(chrono.ChVectorD(side / 2, side / 2, side / 2))
    box_shape.SetTexture(chrono.GetChronoDataFile("textures/checker.png"))
    box_body.AddVisualShape(box_shape)

    
    
    
    manager = sens.ChSensorManager(mphysicalSystem)

    
    
    
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(-side / 2 - 0.1, 0, 0.1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    lidar = sens.ChLidarSensor(
        box_body,              
        5.0,                   
        offset_pose,            
        800,                  
        1,                    
        2 * chrono.CH_PI,      
        0,                    
        0,                    
        100.0,                
        sens.LidarBeamShape_RECTANGULAR,  
        0.01,                 
        0.003,               
        0.003,               
        sens.LidarReturnMode_STRONGEST_RETURN
    )
    lidar.SetName("Lidar Sensor")
    lidar.SetLag(0)
    lidar.SetCollectionWindow(1 / 5.0)  

    
    
    
    if noise_model == "CONST_NORMAL_XYZI":
        lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    elif noise_model == "NONE":
        
        pass

    if vis:
        
        lidar.PushFilter(sens.ChFilterVisualize(800, 1, "Raw Lidar Depth Data"))

    
    lidar.PushFilter(sens.ChFilterDIAccess())

    
    lidar.PushFilter(sens.ChFilterPCfromDepth())

    if vis:
        
        lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))

    
    lidar.PushFilter(sens.ChFilterXYZIAccess())

    
    manager.AddSensor(lidar)

    
    
    
    offset_pose_2d = chrono.ChFramed(
        chrono.ChVector3d(0, -side / 2 - 0.1, 0.1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    lidar_2d = sens.ChLidarSensor2D(
        box_body,              
        5.0,                   
        offset_pose_2d,            
        800,                  
        1,                    
        2 * chrono.CH_PI,      
        0,                    
        0,                    
        100.0,                
        sens.LidarBeamShape_RECTANGULAR,  
        0.01,                 
        0.003,               
        0.003,               
        sens.LidarReturnMode_STRONGEST_RETURN
    )
    lidar_2d.SetName("Lidar Sensor 2D")
    lidar_2d.SetLag(0)
    lidar_2d.SetCollectionWindow(1 / 5.0)  

    
    
    
    if noise_model == "CONST_NORMAL_XYZI":
        lidar_2d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    elif noise_model == "NONE":
        
        pass

    if vis:
        
        lidar_2d.PushFilter(sens.ChFilterVisualize(800, 1, "Raw 2D Lidar Depth Data"))

    
    lidar_2d.PushFilter(sens.ChFilterDIAccess())

    
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())

    if vis:
        
        lidar_2d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "2D Lidar Point Cloud"))

    
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
                    0.1
                ),
                chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVector3d(0, 0, 1))
            )
        )

        
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            print('XYZI buffer received from lidar. Lidar resolution: {0}x{1}'.format(xyzi_buffer.Width, xyzi_buffer.Height))
            print('Max Value: {0}'.format(np.max(xyzi_data)))

        
        lidar_2d.SetOffsetPose(
            chrono.ChFramed(
                chrono.ChVector3d(
                    0,
                    -orbit_radius * math.cos(ch_time * orbit_rate),
                    -orbit_radius * math.sin(ch_time * orbit_rate) + 0.1
                ),
                chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVector3d(0, 0, 1))
            )
        )

        
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






step_size = 1e-3


end_time = 40.0


save = False


vis = True


out_dir = "SENSOR_OUTPUT/"


main()