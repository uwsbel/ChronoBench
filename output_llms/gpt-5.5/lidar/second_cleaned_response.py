import pychrono.core as chrono
import pychrono.sensor as sens

import numpy as np
import time
import math


def make_orbit_pose(radius, angle, height):
    
    return chrono.ChFramed(
        chrono.ChVector3d(
            -radius * math.cos(angle),
            -radius * math.sin(angle),
            height
        ),
        chrono.QuatFromAngleAxis(angle, chrono.ChVector3d(0, 0, 1))
    )


def main():
    
    
    
    mphysicalSystem = chrono.ChSystemNSC()

    
    
    
    box_body = chrono.ChBodyEasyBox(side, side, side, 1000)
    box_body.SetName("Sensed Box")
    box_body.SetPos(chrono.ChVector3d(0, 0, 0))
    box_body.SetFixed(True)

    
    if box_body.GetVisualShape(0):
        box_body.GetVisualShape(0).SetTexture(
            chrono.GetChronoDataFile("textures/bluewhite.png")
        )

    mphysicalSystem.Add(box_body)

    
    
    
    manager = sens.ChSensorManager(mphysicalSystem)

    
    
    
    offset_pose = make_orbit_pose(orbit_radius, 0.0, lidar_height)

    lidar = sens.ChLidarSensor(
        box_body,                         
        update_rate,                      
        offset_pose,                      
        horizontal_samples,               
        vertical_samples,                 
        horizontal_fov,                   
        max_vert_angle,                   
        min_vert_angle,                   
        max_lidar_range,                  
        sens.LidarBeamShape_RECTANGULAR,  
        sample_radius,                    
        divergence_angle,                 
        divergence_angle,                 
        return_mode                       
    )

    lidar.SetName("3D Lidar Sensor")
    lidar.SetLag(lag)
    lidar.SetCollectionWindow(collection_time)

    
    
    
    if vis:
        lidar.PushFilter(
            sens.ChFilterVisualize(
                horizontal_samples,
                vertical_samples,
                "Raw 3D Lidar Depth Data"
            )
        )

    
    lidar.PushFilter(sens.ChFilterDIAccess())

    
    lidar.PushFilter(sens.ChFilterPCfromDepth())

    
    if noise_model == "CONST_NORMAL_XYZI":
        lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    elif noise_model == "NONE":
        pass

    if vis:
        lidar.PushFilter(
            sens.ChFilterVisualizePointCloud(
                640,
                480,
                1.0,
                "3D Lidar Point Cloud"
            )
        )

    
    lidar.PushFilter(sens.ChFilterXYZIAccess())

    manager.AddSensor(lidar)

    
    
    
    offset_pose_2d = make_orbit_pose(orbit_radius_2d, 0.0, lidar_2d_height)

    lidar_2d = sens.ChLidarSensor(
        box_body,                         
        update_rate_2d,                   
        offset_pose_2d,                   
        horizontal_samples_2d,            
        vertical_samples_2d,              
        horizontal_fov_2d,                
        max_vert_angle_2d,                
        min_vert_angle_2d,                
        max_lidar_range_2d,               
        sens.LidarBeamShape_RECTANGULAR,  
        sample_radius_2d,                 
        divergence_angle_2d,              
        divergence_angle_2d,              
        return_mode                       
    )

    lidar_2d.SetName("2D Lidar Sensor")
    lidar_2d.SetLag(lag_2d)
    lidar_2d.SetCollectionWindow(collection_time_2d)

    
    
    
    if vis:
        lidar_2d.PushFilter(
            sens.ChFilterVisualize(
                horizontal_samples_2d,
                100,
                "Raw 2D Lidar Depth Data"
            )
        )

    
    lidar_2d.PushFilter(sens.ChFilterDIAccess())

    
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())

    if noise_model == "CONST_NORMAL_XYZI":
        lidar_2d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    elif noise_model == "NONE":
        pass

    if vis:
        lidar_2d.PushFilter(
            sens.ChFilterVisualizePointCloud(
                640,
                480,
                2.0,
                "2D Lidar Point Cloud"
            )
        )

    
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())

    manager.AddSensor(lidar_2d)

    
    
    
    ch_time = 0.0
    t1 = time.time()

    while ch_time < end_time:
        angle = ch_time * orbit_rate
        angle_2d = ch_time * orbit_rate_2d

        
        lidar.SetOffsetPose(make_orbit_pose(orbit_radius, angle, lidar_height))
        lidar_2d.SetOffsetPose(make_orbit_pose(orbit_radius_2d, angle_2d, lidar_2d_height))

        
        manager.Update()

        
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            print(
                "3D XYZI buffer received. Resolution: {0}x{1}".format(
                    xyzi_buffer.Width,
                    xyzi_buffer.Height
                )
            )
            print("3D lidar max value:", np.max(xyzi_data))

        
        xyzi_buffer_2d = lidar_2d.GetMostRecentXYZIBuffer()
        if xyzi_buffer_2d.HasData():
            xyzi_data_2d = xyzi_buffer_2d.GetXYZIData()
            print(
                "2D XYZI buffer received. Resolution: {0}x{1}".format(
                    xyzi_buffer_2d.Width,
                    xyzi_buffer_2d.Height
                )
            )
            print("2D lidar max value:", np.max(xyzi_data_2d))

        
        mphysicalSystem.DoStepDynamics(step_size)

        
        ch_time = mphysicalSystem.GetChTime()

    print("Sim time:", end_time, "Wall time:", time.time() - t1)





side = 2.0






noise_model = "NONE"



return_mode = sens.LidarReturnMode_STRONGEST_RETURN


update_rate = 5.0
horizontal_samples = 800
vertical_samples = 300

horizontal_fov = 2 * chrono.CH_PI
max_vert_angle = chrono.CH_PI / 12
min_vert_angle = -chrono.CH_PI / 6

max_lidar_range = 100.0

lag = 0
collection_time = 1.0 / update_rate

sample_radius = 2
divergence_angle = 0.003


update_rate_2d = 10.0
horizontal_samples_2d = 1080
vertical_samples_2d = 1

horizontal_fov_2d = 2 * chrono.CH_PI
max_vert_angle_2d = 0.0
min_vert_angle_2d = 0.0

max_lidar_range_2d = 100.0

lag_2d = 0
collection_time_2d = 1.0 / update_rate_2d

sample_radius_2d = 1
divergence_angle_2d = 0.003




orbit_radius = 10.0
orbit_rate = 0.1
lidar_height = 0.0

orbit_radius_2d = 10.0
orbit_rate_2d = 0.1
lidar_2d_height = 0.0




step_size = 1e-3
end_time = 40.0


vis = True


out_dir = "SENSOR_OUTPUT/"




main()