import pychrono.core as chrono
import pychrono.sensor as sens
import numpy as np
import time
import math


def main():
    
    
    
    mphysicalSystem = chrono.ChSystemNSC()

    
    
    
    box = chrono.ChBodyEasyBox(1, 1, 1, 1000)
    box.SetPos(chrono.ChVector3d(0, 0, 0))
    box.SetBodyFlags(chrono.ChBody::ChBodyBound::kStatic)
    box.SetTexture("box_texture.jpg")  
    mphysicalSystem.Add(box)

    
    
    
    manager = sens.ChSensorManager(mphysicalSystem)

    
    
    
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(-12, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    lidar = sens.ChLidarSensor(
        box,              
        update_rate,            
        offset_pose,            
        vertical_samples=1,      
        horizontal_samples=800,   
        horizontal_fov=2 * chrono.CH_PI,  
        max_vert_angle=chrono.CH_PI / 12,  
        min_vert_angle=-chrono.CH_PI / 6,  
        100.0,                  
        sens.LidarBeamShape_RECTANGULAR,  
        sample_radius=2,          
        divergence_angle=0.003,     
        return_mode=sens.LidarReturnMode_STRONGEST_RETURN,  
        lag=0,                
        collection_time=1. / update_rate  
    )
    lidar.SetName("Lidar Sensor")
    lidar.SetLag(lag)
    lidar.SetCollectionWindow(collection_time)

    
    
    
    lidar_2d = sens.ChLidarSensor(
        box,              
        update_rate,            
        vertical_samples=1,      
        horizontal_samples=800,   
        horizontal_fov=2 * chrono.CH_PI,  
        max_vert_angle=chrono.CH_PI / 12,  
        min_vert_angle=-chrono.CH_PI / 6,  
        100.0,                  
        sens.LidarBeamShape_RECTANGULAR,  
        sample_radius=2,          
        divergence_angle=0.003,     
        return_mode=sens.LidarReturnMode_STRONGEST_RETURN,  
        lag=0,                
        collection_time=1. / update_rate  
    )
    lidar_2d.SetName("2D Lidar Sensor")
    lidar_2d.SetLag(lag)
    lidar_2d.SetCollectionWindow(collection_time)

    
    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar_2d.PushFilter(sens.ChFilterDIAccess())

    
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())

    
    lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth Data"))
    lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth Data"))

    
    lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
    lidar_2d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "2D Lidar Point Cloud"))

    
    manager.AddSensor(lidar)
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


horizontal_fov = 2 * chrono.CH_PI  
max_vert_angle = chrono.CH_PI / 12
min_vert_angle = -chrono.CH_PI / 6


lag = 0


collection_time = 1. / update_rate  


sample_radius = 2


divergence_angle = 0.003






step_size = 1e-3


end_time = 40.0


save = False


vis = True


out_dir = "SENSOR_OUTPUT/"






main()