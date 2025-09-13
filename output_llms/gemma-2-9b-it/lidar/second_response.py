```python
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
    side = 1.0  # Set the side length of the box
    box_body = chrono.ChBodyEasyBox(side, side, side, 1000)  
    box_body.SetPos(chrono.ChVector3d(0, 0, 0))
    box_body.SetBodyFixed(True)  # Fix the box in place
    mphysicalSystem.Add(box_body)

    # -----------------------
    # Create a sensor manager
    # -----------------------
    manager = sens.ChSensorManager(mphysicalSystem)

    # ------------------------------------------------
    # Create a lidar and add it to the sensor manager
    # ------------------------------------------------
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(-12, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    lidar = sens.ChLidarSensor(
        box_body,              # Body lidar is attached to
        update_rate,            # Scanning rate in Hz
        offset_pose,            # Offset pose
        horizontal_samples,     # Number of horizontal samples
        vertical_samples,       # Number of vertical channels
        horizontal_fov,         # Horizontal field of view
        max_vert_angle,         # Maximum vertical field of view
        min_vert_angle,         # Minimum vertical field of view
        100.0,                  # Maximum lidar range
        sens.LidarBeamShape_RECTANGULAR,  # Shape of the lidar beam
        sample_radius,          # Sample radius
        divergence_angle,       # Divergence angle
        divergence_angle,       # Divergence angle (again, typically same value)
        return_mode             # Return mode for the lidar
    )
    lidar.SetName("Lidar Sensor")
    lidar.SetLag(lag)
    lidar.SetCollectionWindow(collection_time)

    # -----------------------------------------------------------------
    # Create a filter graph for post-processing the data from the lidar
    # -----------------------------------------------------------------
    if noise_model == "CONST_NORMAL_XYZI":
        lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    elif noise_model == "NONE":
        # Don't add any noise models
        pass

    if vis:
        # Visualize the raw lidar data
        lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth Data"))

    # Provides the host access to the Depth, Intensity data
    lidar.PushFilter(sens.ChFilterDIAccess())

    # Convert Depth, Intensity data to XYZI point cloud data
    lidar.PushFilter(sens.ChFilterPCfromDepth())

    if vis:
        # Visualize the point cloud
        lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))

    # Provides the host access to the XYZI data
    lidar.PushFilter(sens.ChFilterXYZIAccess())

    # Add the lidar to the sensor manager
    manager.AddSensor(lidar)

    # ------------------------------------------------
    # Create a 2D lidar sensor
    # ------------------------------------------------
    lidar_2d = sens.ChLidarSensor2D(
        box_body,              # Body lidar is attached to
        update_rate,            # Scanning rate in Hz
        chrono.ChVector3d(0, 0, 1),  # Offset pose
        horizontal_samples,     # Number of horizontal samples
        1,                      # Number of vertical channels
        horizontal_fov,         # Horizontal field of view
        0,                      # Maximum vertical field of view
        min_vert_angle,         # Minimum vertical field of view
        100.0,                  # Maximum lidar range
        sens.LidarBeamShape_RECTANGULAR,  # Shape of the lidar beam
        sample_radius,          # Sample radius
        divergence_angle,       # Divergence angle
        divergence_angle,       # Divergence angle (again, typically same value)
        return_mode             # Return mode for the lidar
    )
    lidar_2d.SetName("2D Lidar Sensor")
    lidar_2d.SetLag(lag)
    lidar_2d.SetCollectionWindow(collection_time)

    # Add filters and visualization for the 2D lidar
    if noise_model == "CONST_NORMAL_XYZI":
        lidar_2d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    elif noise_model == "NONE":
        pass

    if vis:
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, 1, "Raw 2D Lidar Depth Data"))

    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())

    if vis:
        lidar_2d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "2D Lidar Point Cloud"))

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

    while ch_time < end_time:
        # Set lidar to orbit around the box
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


# -----------------
# Lidar parameters
# -----------------

# Noise model attached to the sensor
# TODO: Noise models haven't been implemented in python
# noise_model = "CONST_NORMAL_XYZI"  # Gaussian noise with constant mean and standard deviation
noise_model = "NONE"  # No noise model

# Lidar return mode
return_mode = sens.LidarReturnMode_STRONGEST_RETURN
# return_mode = sens.MEAN_RETURN
# return_mode = sens.FIRST_RETURN
# return_mode = sens.LAST_RETURN

# Update rate in Hz
update_rate =