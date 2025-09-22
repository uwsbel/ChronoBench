import pychrono.core as chrono
import pychrono.sensor as sens

import numpy as np
import time
import math

def main():
    # -----------------
    # Create the system
    # -----------------
    mphysicalSystem = chrono.ChSystemNSC()

    # ----------------------------------
    # Add a box to be sensed by a lidar
    # ----------------------------------
    side = 2.0
    box_body = chrono.ChBodyEasyBox(side, side, side, 1000)
    box_body.SetPos(chrono.ChVectorD(0, 0, 0))
    box_body.SetFixed(True)
    mphysicalSystem.Add(box_body)

    # Create a visual representation of the box
    box_shape = chrono.ChVisualShapeBox()
    box_shape.SetBox(side, side, side)
    box_shape.SetColor(chrono.ChColor(1, 0, 0))
    box_body.AddVisualShape(box_shape)

    # -----------------------
    # Create a sensor manager
    # -----------------------
    manager = sens.ChSensorManager(mphysicalSystem)

    # ------------------------------------------------
    # Create a 3D lidar and add it to the sensor manager
    # ------------------------------------------------
    offset_pose = chrono.ChFrameD(
        chrono.ChVectorD(-12, 0, 1), 
        chrono.Q_from_AngX(0) * chrono.Q_from_AngY(0) * chrono.Q_from_AngZ(0)
    )
    update_rate = 5.0
    horizontal_samples = 800
    vertical_samples = 300
    horizontal_fov = 2 * chrono.CH_PI
    max_vert_angle = chrono.CH_PI / 12
    min_vert_angle = -chrono.CH_PI / 6
    sample_radius = 2
    divergence_angle = 0.003
    return_mode = sens.LidarReturnMode_STRONGEST_RETURN
    lag = 0
    collection_time = 1. / update_rate
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
    lidar.SetName("Lidar Sensor")
    lidar.SetLag(lag)
    lidar.SetCollectionWindow(collection_time)

    # -----------------------------------------------------------------
    # Create a filter graph for post-processing the data from the lidar
    # -----------------------------------------------------------------
    noise_model = "NONE"
    if noise_model == "CONST_NORMAL_XYZI":
        lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    elif noise_model == "NONE":
        pass

    vis = True
    if vis:
        lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth Data"))

    lidar.PushFilter(sens.ChFilterDIAccess())

    lidar.PushFilter(sens.ChFilterPCfromDepth())

    if vis:
        lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))

    lidar.PushFilter(sens.ChFilterXYZIAccess())

    manager.AddSensor(lidar)

    # Create a 2D lidar sensor
    offset_pose_2d = chrono.ChFrameD(
        chrono.ChVectorD(-12, 0, 1), 
        chrono.Q_from_AngX(0) * chrono.Q_from_AngY(0) * chrono.Q_from_AngZ(0)
    )
    lidar_2d = sens.ChLidarSensor(
        box_body, 
        update_rate, 
        offset_pose_2d, 
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
        return_mode
    )
    lidar_2d.SetName("Lidar 2D Sensor")
    lidar_2d.SetLag(lag)
    lidar_2d.SetCollectionWindow(collection_time)

    if noise_model == "CONST_NORMAL_XYZI":
        lidar_2d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    elif noise_model == "NONE":
        pass

    if vis:
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, 1, "Raw Lidar 2D Depth Data"))

    lidar_2d.PushFilter(sens.ChFilterDIAccess())

    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())

    if vis:
        lidar_2d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar 2D Point Cloud"))

    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())

    manager.AddSensor(lidar_2d)

    # ---------------
    # Simulate system
    # ---------------
    orbit_radius = 10
    orbit_rate = 0.1
    ch_time = 0.0

    render_time = 0
    t1 = time.time()

    step_size = 1e-3
    end_time = 40.0

    while ch_time < end_time:
        # Set lidar to orbit around the box body
        lidar.SetOffsetFrame(
            chrono.ChFrameD(
                chrono.ChVectorD(
                    -orbit_radius * math.cos(ch_time * orbit_rate),
                    -orbit_radius * math.sin(ch_time * orbit_rate),
                    1
                ),
                chrono.Q_from_AngX(0) * chrono.Q_from_AngY(0) * chrono.Q_from_AngZ(ch_time * orbit_rate)
            )
        )

        lidar_2d.SetOffsetFrame(
            chrono.ChFrameD(
                chrono.ChVectorD(
                    -orbit_radius * math.cos(ch_time * orbit_rate),
                    -orbit_radius * math.sin(ch_time * orbit_rate),
                    1
                ),
                chrono.Q_from_AngX(0) * chrono.Q_from_AngY(0) * chrono.Q_from_AngZ(ch_time * orbit_rate)
            )
        )

        # Access the XYZI buffer from lidar
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            print('XYZI buffer received from lidar. Lidar resolution: {0}x{1}'.format(xyzi_buffer.Width, xyzi_buffer.Height))
            print('Max Value: {0}'.format(np.max(xyzi_data)))

        # Update sensor manager (will render/save/filter automatically)
        manager.Update()

        # Perform step of dynamics
        mphysicalSystem.DoStepDynamics(step_size)

        # Get the current time of the simulation
        ch_time = mphysicalSystem.GetChTime()

    print("Sim time:", end_time, "Wall time:", time.time() - t1)


if __name__ == "__main__":
    main()