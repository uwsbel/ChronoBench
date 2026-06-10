import pychrono.core as chrono
import pychrono.sensor as sens

import numpy as np
import time
import math


def make_orbit_pose(radius, angle, height):
    """Pose for a lidar orbiting the box while looking toward the origin."""
    return chrono.ChFramed(
        chrono.ChVector3d(
            -radius * math.cos(angle),
            -radius * math.sin(angle),
            height
        ),
        chrono.QuatFromAngleAxis(angle, chrono.ChVector3d(0, 0, 1))
    )


def main():
    # -----------------
    # Create the system
    # -----------------
    mphysicalSystem = chrono.ChSystemNSC()

    # -----------------------
    # Add a box to be sensed
    # -----------------------
    box_body = chrono.ChBodyEasyBox(side, side, side, 1000)
    box_body.SetName("Sensed Box")
    box_body.SetPos(chrono.ChVector3d(0, 0, 0))
    box_body.SetFixed(True)

    # Set a texture on the visual shape created by ChBodyEasyBox
    if box_body.GetVisualShape(0):
        box_body.GetVisualShape(0).SetTexture(
            chrono.GetChronoDataFile("textures/bluewhite.png")
        )

    mphysicalSystem.Add(box_body)

    # -----------------------
    # Create a sensor manager
    # -----------------------
    manager = sens.ChSensorManager(mphysicalSystem)

    # ------------------------------------------------
    # Create a 3D lidar and add it to the sensor manager
    # ------------------------------------------------
    offset_pose = make_orbit_pose(orbit_radius, 0.0, lidar_height)

    lidar = sens.ChLidarSensor(
        box_body,                         # Body lidar is attached to
        update_rate,                      # Scanning rate in Hz
        offset_pose,                      # Offset pose relative to box body
        horizontal_samples,               # Number of horizontal samples
        vertical_samples,                 # Number of vertical channels
        horizontal_fov,                   # Horizontal field of view
        max_vert_angle,                   # Maximum vertical field of view
        min_vert_angle,                   # Minimum vertical field of view
        max_lidar_range,                  # Maximum lidar range
        sens.LidarBeamShape_RECTANGULAR,  # Shape of the lidar beam
        sample_radius,                    # Sample radius
        divergence_angle,                 # Horizontal divergence angle
        divergence_angle,                 # Vertical divergence angle
        return_mode                       # Return mode for the lidar
    )

    lidar.SetName("3D Lidar Sensor")
    lidar.SetLag(lag)
    lidar.SetCollectionWindow(collection_time)

    # ----------------------------
    # 3D lidar filter graph
    # ----------------------------
    if vis:
        lidar.PushFilter(
            sens.ChFilterVisualize(
                horizontal_samples,
                vertical_samples,
                "Raw 3D Lidar Depth Data"
            )
        )

    # Host access to Depth/Intensity data
    lidar.PushFilter(sens.ChFilterDIAccess())

    # Convert Depth/Intensity to XYZI point cloud
    lidar.PushFilter(sens.ChFilterPCfromDepth())

    # Optional noise model should be applied to XYZI data after point cloud creation
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

    # Host access to XYZI data
    lidar.PushFilter(sens.ChFilterXYZIAccess())

    manager.AddSensor(lidar)

    # ------------------------------------------------
    # Create an additional 2D lidar sensor
    # ------------------------------------------------
    offset_pose_2d = make_orbit_pose(orbit_radius_2d, 0.0, lidar_2d_height)

    lidar_2d = sens.ChLidarSensor(
        box_body,                         # Body 2D lidar is attached to
        update_rate_2d,                   # Scanning rate in Hz
        offset_pose_2d,                   # Offset pose relative to box body
        horizontal_samples_2d,            # Horizontal samples
        vertical_samples_2d,              # One vertical channel for 2D lidar
        horizontal_fov_2d,                # Horizontal FOV
        max_vert_angle_2d,                # Max vertical angle
        min_vert_angle_2d,                # Min vertical angle
        max_lidar_range_2d,               # Max range
        sens.LidarBeamShape_RECTANGULAR,  # Beam shape
        sample_radius_2d,                 # Sample radius
        divergence_angle_2d,              # Horizontal divergence
        divergence_angle_2d,              # Vertical divergence
        return_mode                       # Return mode
    )

    lidar_2d.SetName("2D Lidar Sensor")
    lidar_2d.SetLag(lag_2d)
    lidar_2d.SetCollectionWindow(collection_time_2d)

    # ----------------------------
    # 2D lidar filter graph
    # ----------------------------
    if vis:
        lidar_2d.PushFilter(
            sens.ChFilterVisualize(
                horizontal_samples_2d,
                100,
                "Raw 2D Lidar Depth Data"
            )
        )

    # Host access to Depth/Intensity data
    lidar_2d.PushFilter(sens.ChFilterDIAccess())

    # Convert Depth/Intensity to XYZI point cloud
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

    # Host access to XYZI data
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())

    manager.AddSensor(lidar_2d)

    # ---------------
    # Simulate system
    # ---------------
    ch_time = 0.0
    t1 = time.time()

    while ch_time < end_time:
        angle = ch_time * orbit_rate
        angle_2d = ch_time * orbit_rate_2d

        # Orbit both lidars around the box
        lidar.SetOffsetPose(make_orbit_pose(orbit_radius, angle, lidar_height))
        lidar_2d.SetOffsetPose(make_orbit_pose(orbit_radius_2d, angle_2d, lidar_2d_height))

        # Update sensor manager
        manager.Update()

        # Access 3D lidar XYZI buffer
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

        # Access 2D lidar XYZI buffer
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

        # Perform step of dynamics
        mphysicalSystem.DoStepDynamics(step_size)

        # Get current simulation time
        ch_time = mphysicalSystem.GetChTime()

    print("Sim time:", end_time, "Wall time:", time.time() - t1)


# -----------------
# Box parameters
# -----------------
side = 2.0

# -----------------
# Lidar parameters
# -----------------

# Noise model attached to the sensor
noise_model = "NONE"
# noise_model = "CONST_NORMAL_XYZI"

# Lidar return mode
return_mode = sens.LidarReturnMode_STRONGEST_RETURN

# 3D lidar parameters
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

# 2D lidar parameters
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

# ---------------------
# Orbit parameters
# ---------------------
orbit_radius = 10.0
orbit_rate = 0.1
lidar_height = 0.0

orbit_radius_2d = 10.0
orbit_rate_2d = 0.1
lidar_2d_height = 0.0

# ---------------------
# Simulation parameters
# ---------------------
step_size = 1e-3
end_time = 40.0

# Render lidar visualizations
vis = True

# Output directory, if save filters are added later
out_dir = "SENSOR_OUTPUT/"

# If running from a non-standard location, uncomment and set:
# chrono.SetChronoDataPath("path/to/chrono/data/")

main()