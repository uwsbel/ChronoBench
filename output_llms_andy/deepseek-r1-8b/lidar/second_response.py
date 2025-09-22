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
    box_side = 2.0  # Box side length
    box_body = chrono.ChBodyEasyBox(box_side, box_side, box_side, 1000.0)  # Create a box with given side length and height
    box_body.SetPos(chrono.ChVector3d(0, 0, 0))  # Position the box at origin
    box_body.AddVisualShape(chrono.ChVisualShapeBox())  # Add a visual shape for visualization
    mphysicalSystem.Add(box_body)

    # -----------------------
    # Create a sensor manager
    # -----------------------
    manager = sens.ChSensorManager(mphysicalSystem)

    # ------------------------------------------------
    # Create 3D lidar and add it to the sensor manager
    # ------------------------------------------------
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(
            -10, 0, 1,
        ),
        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    lidar_3d = sens.ChLidarSensor(
        box_body,  # Lidar is attached to the box
        5.0,      # Update rate in Hz
        offset_pose,
        800,      # Horizontal samples
        300,      # Vertical samples
        2 * chrono.CH_PI,  # Horizontal field of view (90 degrees)
        max_vert_angle=chrono.CH_PI / 12,
        min_vert_angle=-chrono.CH_PI / 6,
        100.0,    # Maximum lidar range
        sens.LidarBeamShape_RECTANGULAR,
        2,        # Sample radius
        divergence_angle=0.003,
        divergence_angle=0.003,
        return_mode=sens.LidarReturnMode_STRONGEST_RETURN
    )
    lidar_3d.SetName("3D Lidar Sensor")
    lidar_3d.SetLag(0)
    lidar_3d.SetCollectionWindow(1.0 / 5.0)  # Collection window based on update rate

    # -----------------------------------------------------------------
    # Create a filter graph for post-processing the data from the lidar
    # -----------------------------------------------------------------
    if noise_model == "CONST_NORMAL_XYZI":
        lidar_3d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    elif noise_model == "NONE":
        pass

    if vis:
        lidar_3d.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "3D Lidar Depth Data"))

    lidar_3d.PushFilter(sens.ChFilterDIAccess())
    lidar_3d.PushFilter(sens.ChFilterPCfromDepth())

    if vis:
        lidar_3d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "3D Lidar Point Cloud"))

    lidar_3d.PushFilter(sens.ChFilterXYZIAccess())

    manager.AddSensor(lidar_3d)

    # ------------------------------------------------
    # Create 2D lidar and add it to the sensor manager
    # ------------------------------------------------
    offset_pose_2d = chrono.ChFramed(
        chrono.ChVector3d(
            -5, 0, 1,
        ),
        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    lidar_2d = sens.ChLidarSensor(
        box_body,  # Lidar is attached to the box
        20.0,     # Update rate in Hz
        offset_pose_2d,
        1280,     # Horizontal samples
        480,      # Vertical samples (2D lidar)
        1.5708,   # Horizontal field of view (90 degrees in radians)
        max_vert_angle=0,
        min_vert_angle=0,
        100.0,    # Maximum lidar range
        sens.LidarBeamShape_RECTANGULAR,
        1,        # Sample radius
        divergence_angle=0.003,
        divergence_angle=0.003,
        return_mode=sens.LidarReturnMode_STRONGEST_RETURN
    )
    lidar_2d.SetName("2D Lidar Sensor")
    lidar_2d.SetLag(0)
    lidar_2d.SetCollectionWindow(1.0 / 20.0)  # Collection window based on update rate

    if vis:
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "2D Lidar Depth Data"))

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

        # Access the XYZI buffer from lidar_3d
        xyzi_buffer_3d = lidar_3d.GetMostRecentXYZIBuffer()
        if xyzi_buffer_3d.HasData():
            xyzi_data_3d = xyzi_buffer_3d.GetXYZIData()
            print('3D XYZI buffer received from lidar. Lidar resolution: {0}x{1}'.format(xyzi_buffer_3d.Width, xyzi_buffer_3d.Height))
            print('Max Value: {0}'.format(np.max(xyzi_data_3d)))

        # Access the XYZI buffer from lidar_2d
        xyzi_buffer_2d = lidar_2d.GetMostRecentXYZIBuffer()
        if xyzi_buffer_2d.HasData():
            xyzi_data_2d = xyzi_buffer_2d.GetXYZIData()
            print('2D XYZI buffer received from lidar. Lidar resolution: {0}x{1}'.format(xyzi_buffer_2d.Width, xyzi_buffer_2d.Height))
            print('Max Value: {0}'.format(np.max(xyzi_data_2d)))

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
vertical_samples = 300  # Set to 480 for 2D lidar

# Horizontal and vertical field of view (radians)
horizontal_fov = 2 * chrono.CH_PI  # 360 degrees for 3D lidar
max_vert_angle = chrono.CH_PI / 12
min_vert_angle = -chrono.CH_PI / 6

# Lag time
lag = 0

# Collection window for the lidar
collection_time = 1. / update_rate  # typically 1/update rate

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