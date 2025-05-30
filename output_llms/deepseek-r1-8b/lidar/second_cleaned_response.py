import pychrono.core as chrono
import pychrono.sensor as sens

import numpy as np
import time
import math


update_rate = 5.0
horizontal_samples = 800
vertical_samples = 1  
horizontal_fov = 2 * chrono.CH_PI  
max_vert_angle = chrono.CH_PI / 12
min_vert_angle = -chrono.CH_PI / 6
sample_radius = 2
divergence_angle = 0.003
collection_time = 1. / update_rate
lag = 0
return_mode = sens.LidarReturnMode_STRONGEST_RETURN


noise_model = "NONE"


step_size = 1e-3
end_time = 40.0
save = False
vis = True
out_dir = "SENSOR_OUTPUT/"

def main():
    
    mphysicalSystem = chrono.ChSystemNSC()

    
    box = chrono.ChBodyEasyBox(side=1, side=1, side=1, mass=1000)
    box.SetPos(chrono.ChVector3d(0, 0, 0))
    box.AddVisualShape(chrono.ChVisualShapeBox())
    box.SetFixed(True)
    mphysicalSystem.Add(box)

    
    manager = sens.ChSensorManager(mphysicalSystem)

    
    lidar_3d = sens.ChLidarSensor(
        box,  
        update_rate,
        chrono.ChFramed(
            chrono.ChVector3d(-12, 0, 1),
            chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
        ),
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
    lidar_3d.SetName("3D Lidar Sensor")
    lidar_3d.SetLag(lag)
    lidar_3d.SetCollectionWindow(collection_time)

    
    if noise_model == "CONST_NORMAL_XYZI":
        lidar_3d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    elif noise_model == "NONE":
        pass

    if vis:
        lidar_3d.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw 3D Lidar Depth Data"))

    lidar_3d.PushFilter(sens.ChFilterDIAccess())
    lidar_3d.PushFilter(sens.ChFilterPCfromDepth())

    if vis:
        lidar_3d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "3D Lidar Point Cloud"))

    lidar_3d.PushFilter(sens.ChFilterXYZIAccess())

    
    lidar_2d = sens.ChLidarSensor2D(
        box,  
        update_rate,
        1,  
        1,  
        1,  
        0,  
        0,  
        100.0,
        sens.LidarBeamShape_LINE,
        1,  
        0,  
        return_mode
    )
    lidar_2d.SetName("2D Lidar Sensor")
    lidar_2d.SetLag(lag)
    lidar_2d.SetCollectionWindow(collection_time)

    
    if vis:
        lidar_2d.PushFilter(sens.ChFilterVisualize(1, 1, "Raw 2D Lidar Depth Data"))

    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth2D())

    manager.AddSensor(lidar_3d)
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

        
        if lidar_3d.GetMostRecentXYZIBuffer().HasData():
            xyzi_3d = lidar_3d.GetMostRecentXYZIBuffer().GetXYZIData()
            print('3D XYZI buffer received. Resolution: {}x{}'.format(xyzi_3d.Width, xyzi_3d.Height))
            print('Max Value: {}'.format(np.max(xyzi_3d)))
        if lidar_2d.GetMostRecentDepthBuffer().HasData():
            depth_2d = lidar_2d.GetMostRecentDepthBuffer().GetDepthData()
            print('2D Depth buffer received. Resolution: {}x{}'.format(depth_2d.Width, depth_2d.Height))
            print('Max Value: {}'.format(np.max(depth_2d)))

        manager.Update()
        mphysicalSystem.DoStepDynamics(step_size)
        ch_time = mphysicalSystem.GetChTime()

    print("Sim time:", end_time, "Wall time:", time.time() - t1)

if __name__ == "__main__":
    main()