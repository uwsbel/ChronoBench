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
    box.SetName("Box")
    mphysicalSystem.Add(box)

    
    box_shape = chrono.ChVisualShapeBox()
    box_shape.SetTexture(chrono.GetChronoDataFile("textures/wood.jpg"))
    box.AddVisualShape(box_shape)

    
    
    
    manager = sens.ChSensorManager(mphysicalSystem)

    
    
    
    lidar_offset = chrono.ChVectorD(-12, 0, 1)
    lidar_rotation = chrono.ChQuaternionD(chrono.GetRotZ(math.radians(90)))
    lidar = sens.ChLidarSensor(box, update_rate, lidar_offset, 1, 1, math.radians(360), math.radians(90), math.radians(-90), 100.0, sens.LidarBeamShape_RECTANGULAR, 1, 0.003, return_mode)
    lidar.SetName("Lidar Sensor")
    lidar.SetLag(lag)
    lidar.SetCollectionWindow(collection_time)

    
    lidar.PushFilter(sens.ChFilterVisualize(1, 1, "Raw Lidar Depth Data"))

    
    lidar.PushFilter(sens.ChFilterPCfromDepth())

    
    lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))

    
    manager.AddSensor(lidar)

    
    
    
    orbit_radius = 10
    orbit_rate = 0.1
    ch_time = 0.0

    render_time = 0
    t1 = time.time()

    while ch_time < end_time:
        
        lidar.SetOffsetPose(chrono.ChFrameD(lidar_offset + chrono.ChVectorD(orbit_radius * math.cos(ch_time * orbit_rate), -orbit_radius * math.sin(ch_time * orbit_rate), 1), lidar_rotation))

        
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


horizontal_samples = 1
vertical_samples = 1


horizontal_fov = 2 * math.pi  
max_vert_angle = math.pi / 12
min_vert_angle = -math.pi / 6


lag = 0


collection_time = 1. / update_rate  


sample_radius = 1


divergence_angle = 0.003






step_size = 1e-3


end_time = 40.0


save = False


vis = True


out_dir = "SENSOR_OUTPUT/"






main()