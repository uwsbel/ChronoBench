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
    # Create a box to be sensed by a lidar
    # ----------------------------------
    box = chrono.ChBodyEasyBox(1, 1, 1, 1000)
    box.SetPos(chrono.ChVector3d(0, 0, 0))
    box.SetBodyFlags(chrono.ChBody::ChBodyBound::kStatic)
    box.SetTexture("box_texture.jpg")  # Set the texture for the box
    mphysicalSystem.Add(box)

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
        box,              # Lidar attached to the box
        update_rate,            # Scanning rate in Hz
        offset_pose,            # Offset pose
        vertical_samples=1,      # Number of vertical samples for the 2D lidar
        horizontal_samples=800,   # Number of horizontal samples for the 2D lidar
        horizontal_fov=2 * chrono.CH_PI,  # Horizontal field of view
        max_vert_angle=chrono.CH_PI / 12,  # Maximum vertical field of view
        min_vert_angle=-chrono.CH_PI / 6,  # Minimum vertical field of view
        100.0,                  # Maximum lidar range
        sens.LidarBeamShape_RECTANGULAR,  # Shape of the lidar beam
        sample_radius=2,          # Radius of samples to use
        divergence_angle=0.003,     # Divergence angle
        return_mode=sens.LidarReturnMode_STRONGEST_RETURN,  # Return mode for the lidar
        lag=0,                # Lag time
        collection_time=1. / update_rate  # typically 1/update rate
    )
    lidar.SetName("Lidar Sensor")
    lidar.SetLag(lag)
    lidar.SetCollectionWindow(collection_time)

    # ------------------------------------------------
    # Create a 2D lidar sensor
    # ------------------------------------------------
    lidar_2d = sens.ChLidarSensor(
        box,              # Lidar attached to the box
        update_rate,            # Scanning rate in Hz
        vertical_samples=1,      # Number of vertical samples for the 2D lidar
        horizontal_samples=800,   # Number of horizontal samples for the 2D lidar
        horizontal_fov=2 * chrono.CH_PI,  # Horizontal field of view
        max_vert_angle=chrono.CH_PI / 12,  # Maximum vertical field of view
        min_vert_angle=-chrono.CH_PI / 6,  # Minimum vertical field of view
        100.0,                  # Maximum lidar range
        sens.LidarBeamShape_RECTANGULAR,  # Shape of the lidar beam
        sample_radius=2,          # Radius of samples to use
        divergence_angle=0.003,     # Divergence angle
        return_mode=sens.LidarReturnMode_STRONGEST_RETURN,  # Return mode for the lidar
        lag=0,                # Lag time
        collection_time=1. / update_rate  # typically 1/update rate
    )
    lidar_2d.SetName("2D Lidar Sensor")
    lidar_2d.SetLag(lag)
    lidar_2d.SetCollectionWindow(collection_time)

    # Provides the host access to the Depth, Intensity data
    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar_2d.PushFilter(sens.ChFilterDIAccess())

    # Convert Depth, Intensity data to XYZI point cloud data
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())

    # Visualize the raw lidar data
    lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth Data"))
    lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth Data"))

    # Visualize the point cloud
    lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
    lidar_2d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "2D Lidar Point Cloud"))

    # Add the lidar to the sensor manager
    manager.AddSensor(lidar)
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
noise_model = "NONE"  # No noise model

# Lidar return mode
return_mode = sens.LidarReturnMode_STRONGEST_RETURN

# Update rate in Hz
update_rate = 5.0

# Number of horizontal and vertical samples
horizontal_samples = 800
vertical_samples = 1  # One vertical channel for the 2D lidar

# Horizontal and vertical field of view (radians)
horizontal_fov = 2 * chrono.CH_PI  # 360 degrees
max_vert_angle = chrono.CH_PI / 12
min_vert_angle = -chrono.CH_PI / 6

# Lag time
lag = 0

# Collection window for the lidar
collection_time = 1. / update_rate  # typically 1/update_rate

# Radius of samples to use, 1->1 sample, 2->9 samples, 3->25 samples...
sample_radius = 2

# 3mm radius (as cited by velodyne)
divergence_angle = 0.003

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