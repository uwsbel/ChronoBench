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

    # -----------------------
    # Create a box object
    # -----------------------
    side = 2.0
    box_body = chrono.ChBodyEasyBox(side, side, side, 1000)
    box_body.SetPos(chrono.ChVector3d(0, 0, 0))
    box_body.SetBodyFixed(True)
    mphysicalSystem.Add(box_body)

    # Create a visual representation of the box
    box_shape = chrono.ChVisualShapeBox()
    box_shape.SetBoxDimensions(chrono.ChVector3d(side, side, side))
    box_shape.SetColor(chrono.ChColor(1, 0, 0))
    box_body.AddVisualShape(box_shape)

    # -----------------------
    # Create a sensor manager
    # -----------------------
    manager = sens.ChSensorManager(mphysicalSystem)

    # ------------------------------------------------
    # Create a 3D lidar and add it to the sensor manager
    # ------------------------------------------------
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(-12, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    lidar = sens.ChLidarSensor(
        box_body,              # Body lidar is attached to
        5.0,                   # Scanning rate in Hz
        offset_pose,           # Offset pose
        800,                   # Number of horizontal samples
        300,                   # Number of vertical channels
        2 * chrono.CH_PI,      # Horizontal field of view
        chrono.CH_PI / 12,     # Maximum vertical field of view
        -chrono.CH_PI / 6,     # Minimum vertical field of view
        100.0,                 # Maximum lidar range
        sens.LidarBeamShape_RECTANGULAR,  # Shape of the lidar beam
        2,                     # Sample radius
        0.003,                 # Divergence angle
        0.003,                 # Divergence angle (again, typically same value)
        sens.LidarReturnMode_STRONGEST_RETURN  # Return mode for the lidar
    )
    lidar.SetName("Lidar Sensor")
    lidar.SetLag(0)
    lidar.SetCollectionWindow(1. / 5.0)

    # -----------------------------------------------------------------
    # Create a filter graph for post-processing the data from the lidar
    # -----------------------------------------------------------------
    if True:  # Use a noise model
        lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    else:
        # Don't add any noise models
        pass

    if True:  # Visualize the raw lidar data
        lidar.PushFilter(sens.ChFilterVisualize(800, 300, "Raw Lidar Depth Data"))

    # Provides the host access to the Depth, Intensity data
    lidar.PushFilter(sens.ChFilterDIAccess())

    # Convert Depth, Intensity data to XYZI point cloud data
    lidar.PushFilter(sens.ChFilterPCfromDepth())

    if True:  # Visualize the point cloud
        lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))

    # Provides the host access to the XYZI data
    lidar.PushFilter(sens.ChFilterXYZIAccess())

    # Add the lidar to the sensor manager
    manager.AddSensor(lidar)

    # ------------------------------------------------
    # Create a 2D lidar and add it to the sensor manager
    # ------------------------------------------------
    lidar_2d = sens.ChLidarSensor(
        box_body,              # Body lidar is attached to
        5.0,                   # Scanning rate in Hz
        offset_pose,           # Offset pose
        800,                   # Number of horizontal samples
        1,                     # Number of vertical channels
        2 * chrono.CH_PI,      # Horizontal field of view
        0,                     # Maximum vertical field of view
        0,                     # Minimum vertical field of view
        100.0,                 # Maximum lidar range
        sens.LidarBeamShape_RECTANGULAR,  # Shape of the lidar beam
        2,                     # Sample radius
        0.003,                 # Divergence angle
        0.003,                 # Divergence angle (again, typically same value)
        sens.LidarReturnMode_STRONGEST_RETURN  # Return mode for the lidar
    )
    lidar_2d.SetName("Lidar 2D Sensor")
    lidar_2d.SetLag(0)
    lidar_2d.SetCollectionWindow(1. / 5.0)

    # -----------------------------------------------------------------
    # Create a filter graph for post-processing the data from the lidar
    # -----------------------------------------------------------------
    if True:  # Use a noise model
        lidar_2d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    else:
        # Don't add any noise models
        pass

    if True:  # Visualize the raw lidar data
        lidar_2d.PushFilter(sens.ChFilterVisualize(800, 1, "Raw Lidar 2D Depth Data"))

    # Provides the host access to the Depth, Intensity data
    lidar_2d.PushFilter(sens.ChFilterDIAccess())

    # Convert Depth, Intensity data to XYZI point cloud data
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())

    if True:  # Visualize the point cloud
        lidar_2d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar 2D Point Cloud"))

    # Provides the host access to the XYZI data
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())

    # Add the 2D lidar to the sensor manager
    manager.AddSensor(lidar_2d)

    # ---------------
    # Simulate system
    # ---------------
    orbit_radius = 10
    orbit_rate = 0.1
    ch_time = 0.0

    render_time = 0
    t1 = time.time()

    while ch_time < 40.0:
        # Set lidar to orbit around the box body
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

        # Set 2D lidar to orbit around the box body
        lidar_2d.SetOffsetPose(
            chrono.ChFramed(
                chrono.ChVector3d(
                    -orbit_radius * math.cos(ch_time * orbit_rate),
                    -orbit_radius * math.sin(ch_time * orbit_rate),
                    1
                ),
                chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVector3d(0, 0, 1))
            )
        )

        # Access the XYZI buffer from lidar
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            print('XYZI buffer received from lidar. Lidar resolution: {0}x{1}'.format(xyzi_buffer.Width, xyzi_buffer.Height))
            print('Max Value: {0}'.format(np.max(xyzi_data)))

        # Access the XYZI buffer from 2D lidar
        xyzi_buffer_2d = lidar_2d.GetMostRecentXYZIBuffer()
        if xyzi_buffer_2d.HasData():
            xyzi_data_2d = xyzi_buffer_2d.GetXYZIData()
            print('XYZI buffer received from 2D lidar. Lidar resolution: {0}x{1}'.format(xyzi_buffer_2d.Width, xyzi_buffer_2d.Height))
            print('Max Value: {0}'.format(np.max(xyzi_data_2d)))

        # Update sensor manager (will render/save/filter automatically)
        manager.Update()

        # Perform step of dynamics
        mphysicalSystem.DoStepDynamics(1e-3)

        # Get the current time of the simulation
        ch_time = mphysicalSystem.GetChTime()

    print("Sim time:", 40.0, "Wall time:", time.time() - t1)


# Call the main function
main()