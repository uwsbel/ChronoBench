import pychrono.core as chrono
import pychrono.sensor as sens

import numpy as np
import time
import math


def main():
    
    
    
    mphysicalSystem = chrono.ChSystemNSC()

    
    
    
    side = 1
    box = chrono.ChBodyEasyBox(side, side, side, 1000)
    box.SetPos(chrono.ChVectorD(0, 0, 0))
    box.SetMaterialSurface(chrono.ChMaterialSurfaceNSC())
    mphysicalSystem.Add(box)

    
    box_shape = chrono.ChVisualShapeBox()
    box_shape.SetMaterial(chrono.ChMaterialSurface())
    box_shape.SetTexture(chrono.GetChronoDataFile("textures/wood.jpg"))
    box.AddVisualShape(box_shape)

    
    
    
    manager = sens.ChSensorManager(mphysicalSystem)

    
    
    
    offset_pose = chrono.ChFrameD(
        chrono.ChVectorD(-12, 0, 1), chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.ChVectorD(0, 1, 0))
    )
    lidar = sens.ChLidarSensor(
        box,  
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
    lidar.SetName("Lidar Sensor")
    lidar.SetLag(lag)
    lidar.SetCollectionWindow(collection_time)

    
    
    
    lidar_2d = sens.ChLidarSensor(
        box,  
        update_rate,  
        offset_pose,  
        horizontal_samples,  
        1,  
        horizontal_fov,  
        max_vert_angle,  
        min_vert_angle,  
        100.0,  
        sens.LidarBeamShape_RECTANGULAR,  
        sample_radius,  
        divergence_angle,  
        divergence_angle,  
        sens.LidarReturnMode_STRONGEST_RETURN  
    )
    lidar_2d.SetName("2D Lidar Sensor")
    lidar_2d.SetLag(lag)
    lidar_2d.SetCollectionWindow(collection_time)

    
    manager.AddSensor(lidar)
    manager.AddSensor(lidar_2d)

    
    
    
    orbit_radius = 10
    orbit_rate = 0.1
    ch_time = 0.0

    render_time = 0
    t1 = time.time()

    while ch_time < end_time:
        
        lidar.SetOffsetPose(offset_pose)

        
        lidar_2d.SetOffsetPose(offset_pose)

        
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            print('XYZI buffer received from lidar. Lidar resolution: {0}x{1}'.format(xyzi_buffer.Width, xyzi_buffer.Height))
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
vertical_samples = 300


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