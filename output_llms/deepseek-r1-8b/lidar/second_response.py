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
    # Create a box object
    # ----------------------------------
    box_body = chrono.ChBodyEasyBox(side=1, side=1, side=1, mass=1000)
    box_body.SetPos(chrono.ChVector3d(0, 0, 0))
    box_body.SetName("Box")
    mphysicalSystem.Add(box_body)

    # Create a visual representation of the box
    trimesh_shape = chrono.ChVisualShapeBox()
    trimesh_shape.SetName("Box Visual")
    trimesh_shape.SetMutable(False)
    box_body.AddVisualShape(trimesh_shape)

    # Create a sensor manager
    manager = sens.ChSensorManager(mphysicalSystem)

    # ------------------------------------------------
    # Create a 3D lidar and add it to the sensor manager
    # ------------------------------------------------
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(-10, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    lidar_3d = sens.ChLidarSensor(
        box_body,              # Body lidar is attached to
        5.0,                  # Update rate in Hz
        offset_pose,            # Offset pose
        800,                  # Number of horizontal samples
        1,                    # Number of vertical channels (for 2D lidar)
        2 * chrono.CH_PI,      # Horizontal field of view (for 2D lidar)
        0.1,                  # Maximum vertical field of view (for 2D lidar)
        100.0,                # Maximum lidar range
        sens.LidarBeamShape_RECTANGULAR,  # Shape of the lidar beam
        1.0,                  # Sample radius (for 2D lidar)
        0.01,                 # Divergence angle (for 2D lidar)
        return_mode             # Return mode for the lidar
    )
    lidar_3d.SetName("3D Lidar Sensor")
    lidar_3d.SetLag(lag)
    lidar_3d.SetCollectionWindow(collection_time)

    # -----------------------------------------------------------------
    # Create a filter graph for post-processing the data from the lidar
    # -----------------------------------------------------------------
    if noise_model == "CONST_NORMAL_XYZI":
        lidar_3d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    elif noise_model == "NONE":
        # Don't add any noise models
        pass

    if vis:
        # Visualize the raw lidar data
        lidar_3d.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth Data"))

    # Provides the host access to the Depth, Intensity data
    lidar_3d.PushFilter(sens.ChFilterDIAccess())

    # Convert Depth, Intensity data to XYZI point cloud data
    lidar_3d.PushFilter(sens.ChFilterPCfromDepth())

    if vis:
        # Visualize the point cloud
        lidar_3d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))

    # Provides the host access to the XYZI data
    lidar_3d.PushFilter(sens.ChFilterXYZIAccess())

    # Add the lidar to the sensor manager
    manager.AddSensor(lidar_3d)

    # Create a 2D lidar and add it to the sensor manager
    # ------------------------------------------------
    offset_pose_2d = chrono.ChFramed(
        chrono.ChVector3d(-5, 0, 0), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    lidar_2d = sens.ChLidarSensor(
        box_body,              # Body lidar is attached to
        5.0,                  # Update rate in Hz
        offset_pose_2d,        # Offset pose
        800,                  # Number of horizontal samples
        1,                    # Number of vertical channels
        2 * chrono.CH_PI / 4,  # Horizontal field of view (for 2D lidar)
        0.5 * chrono.CH_PI / 2,  # Maximum vertical field of view (for 2D lidar)
        100.0,                # Maximum lidar range
        sens.LidarBeamShape_RECTANGULAR,  # Shape of the lidar beam
        1.0,                  # Sample radius (for 2D lidar)
        0.01,                 # Divergence angle (for 2D lidar)
        sens.LidarReturnMode_STRONGEST_RETURN  # Return mode for the lidar
    )
    lidar_2d.SetName("2D Lidar Sensor")
    lidar_2d.SetLag(lag)
    lidar_2d.SetCollectionWindow(collection_time)

    # -----------------------------------------------------------------
    # Create a filter graph for post-processing the data from the lidar
    # -----------------------------------------------------------------
    if noise_model == "CONST_NORMAL_XYZI":
        lidar_2d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    elif noise_model == "NONE":
        # Don't add any noise models
        pass

    if vis:
        # Visualize the raw lidar data
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth Data"))

    # Provides the host access to the Depth, Intensity data
    lidar_2d.PushFilter(sens.ChFilterDIAccess())

    # Convert Depth, Intensity data to XYZI point cloud data
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())

    if vis:
        # Visualize the point cloud
        lidar_2d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))

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

    while ch_time < end_time:
        # Set lidar to orbit around the box body
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

        # Access the XYZI buffer from lidar
        xyzi_buffer = lidar_3d.GetMostRecentXYZIBuffer()
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
# noise_model = "CONST_NORMAL_XYZI"  # Gaussian noise with constant mean and standard deviation
noise_model = "NONE"  # No noise model

# Lidar return mode
return_mode = sens.LidarReturnMode_STRONGEST_RETURN
# return_mode = sens.MEAN_RETURN
# return_mode = sens.FIRST_RETURN
# return_mode = sens.LAST_RETURN

# Update rate in Hz
update_rate = 5.0

# Number of horizontal and vertical samples
horizontal_samples = 800
vertical_samples = 1  # Set to 1 for 2D lidar

# Horizontal and vertical field of view (radians)
horizontal_fov = 2 * chrono.CH_PI / 4  # 90 degrees for 2D lidar
max_vert_angle = chrono.CH_PI / 2
min_vert_angle = -chrono.CH_PI / 2

# Lag time
lag = 0

# Collection window for the lidar
collection_time = 1. / update_rate  # typically 1/update rate

# Radius of samples to use, 1->1 sample, 2->9 samples, 3->25 samples...
sample_radius = 1  # For 2D lidar, this parameter may not be used

# 3mm radius (as cited by velodyne)
divergence_angle = 0.01  # Adjusted for 2D lidar

# ---------------------
# Simulation parameters
# ---------------------

# Simulation step size
step_size = 1e-3

# Simulation end time
end_time = 40.0

# Save camera images
save = False

# Render camera images
vis = True

# Output directory
out_dir = "SENSOR_OUTPUT/"

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with:
# chrono.SetChronoDataPath('path/to/data')

main()