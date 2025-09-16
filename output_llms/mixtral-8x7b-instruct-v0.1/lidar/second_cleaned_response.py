import pychrono.core as chrono
import pychrono.sensor as sens

import numpy as np
import time
import math


def main():
    
    
    
    mphysicalSystem = chrono.ChSystemNSC()

    
    
    
    side = 2.0  
    box_body = chrono.ChBodyEasyBox(side, side, side, 1000, True, True)
    box_body.SetPos(chrono.ChVectorD(0, 0, 0))
    box_body.SetTexture(chrono.GetChronoDataFile("textures/texture_metal_02.png"))
    mphysicalSystem.Add(box_body)

    
    box_shape = chrono.ChBox(side, side, side)
    box_shape.SetPos(chrono.ChVectorD(0, 0, 0))
    box_shape.SetName("Box Mesh")
    box_shape.SetMutable(False)

    box_body.AddVisualShape(box_shape)

    
    
    
    manager = sens.ChSensorManager(mphysicalSystem)

    
    
    
    offset_pose = chrono.ChFramed(
        chrono.ChVectorD(-side / 2 + 0.1, 0, side / 2 + 0.1), chrono.Q_from_AngZ(0)
    )

    lidar = sens.ChLidarSensor(
        box_body,
        update_rate,
        offset_pose,
        800,
        1,
        2 * math.pi,
        max_vert_angle,
        min_vert_angle,
        100.0,
        sens.LidarBeamShape_RECTANGULAR,
        0.01,
        0.003,
        0.003,
        return_mode,
    )
    lidar.SetName("Lidar Sensor")
    lidar.SetLag(lag)
    lidar.SetCollectionWindow(collection_time)

    
    
    
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
        chrono.ChVectorD(-side / 2 + 0.1, 0, side / 2 + 0.1), chrono.Q_from_AngZ(math.pi / 2)
    )

    lidar_2d = sens.ChLidarSensor2D(
        box_body,
        update_rate,
        offset_pose_2d,
        800,
        1,
        2 * math.pi,
        0.0,
        0.0,
        100.0,
        sens.LidarBeamShape_RECTANGULAR,
        0.01,
        0.003,
        0.003,
        return_mode,
    )
    lidar_2d.SetName("Lidar Sensor 2D")
    lidar_2d.SetLag(lag)
    lidar_2d.SetCollectionWindow(collection_time)

    
    
    
    if noise_model == "CONST_NORMAL_XYZI":
        lidar_2d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    elif noise_model == "NONE":
        
        pass

    if vis:
        
        lidar_2d.PushFilter(sens.ChFilterVisualize(800, 1, "Raw Lidar Depth Data 2D"))

    
    lidar_2d.PushFilter(sens.ChFilterDIAccess())

    
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())

    if vis:
        
        lidar_2d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud 2D"))

    
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
                chrono.ChVectorD(
                    -orbit_radius * math.cos(ch_time * orbit_rate),
                    -orbit_radius * math.sin(ch_time * orbit_rate),
                    1
                ),
                chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVectorD(0, 0, 1))
            )
        )

        
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