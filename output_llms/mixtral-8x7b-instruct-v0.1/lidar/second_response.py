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
    side = 1.0  # Set the size of the box
    box_body = chrono.ChBodyEasyBox(side, side, side, 1000, chrono.ChVectorD(0, 0, 0), chrono.QUNIT)
    box_body.SetPos(chrono.ChVectorD(0, 0, 0))
    box_body.SetCollide(True)
    box_body.SetMaterialSurface(chrono.ChMaterialSurface.MakeDielectric(1.0, 0.1))
    mphysicalSystem.Add(box_body)

    # Create a visual representation of the box
    box_shape = chrono.ChBoxShape()
    box_shape.SetSize(chrono.ChVectorD(side, side, side))
    box_shape.SetTexture(chrono.GetChronoDataFile("textures/checker.png"))
    box_body.AddVisualShape(box_shape)

    # -----------------------
    # Create a sensor manager
    # -----------------------
    manager = sens.ChSensorManager(mphysicalSystem)

    # ------------------------------------------------
    # Create a lidar and add it to the sensor manager
    # ------------------------------------------------
    offset_pose = chrono.ChFramed(
        chrono.ChVectorD(-side / 2 - 0.1, 0, side / 2 + 0.1), chrono.QUNIT
    )
    lidar = sens.ChLidarSensor(
        box_body,              # Body lidar is attached to
        5.0,                   # Scanning rate in Hz
        offset_pose,            # Offset pose
        800,                  # Number of horizontal samples
        1,                     # Number of vertical channels
        2 * math.pi,          # Horizontal field of view (2*pi for 360 degrees)
        0,                     # Maximum vertical field of view
        0,                     # Minimum vertical field of view
        100.0,                 # Maximum lidar range
        sens.LidarBeamShape_RECTANGULAR,  # Shape of the lidar beam
        0.05,                 # Sample radius
        0.01,                 # Divergence angle
        0.01,                 # Divergence angle (again, typically same value)
        sens.LidarReturnMode_STRONGEST_RETURN
    )
    lidar.SetName("Lidar Sensor")
    lidar.SetLag(0)
    lidar.SetCollectionWindow(1. / 5)

    # -----------------------------------------------------------------
    # Create a filter graph for post-processing the data from the lidar
    # -----------------------------------------------------------------
    lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    lidar.PushFilter(sens.ChFilterVisualize(800, 1, "Raw Lidar Depth Data"))
    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
    lidar.PushFilter(sens.ChFilterXYZIAccess())

    # Add the lidar to the sensor manager
    manager.AddSensor(lidar)

    # ------------------------------------------------
    # Create a 2D lidar and add it to the sensor manager
    # ------------------------------------------------
    offset_pose_2d = chrono.ChFramed(
        chrono.ChVectorD(side / 2 + 0.1, 0, side / 2 + 0.1), chrono.QUNIT
    )
    lidar_2d = sens.ChLidarSensor2D(
        box_body,              # Body lidar is attached to
        5.0,                   # Scanning rate in Hz
        offset_pose_2d,        # Offset pose
        800,                  # Number of horizontal samples
        1,                     # Number of vertical channels
        2 * math.pi,          # Horizontal field of view (2*pi for 360 degrees)
        0,                     # Maximum vertical field of view
        0,                     # Minimum vertical field of view
        100.0,                 # Maximum lidar range
        sens.LidarBeamShape_RECTANGULAR,  # Shape of the lidar beam
        0.05,                 # Sample radius
        0.01,                 # Divergence angle
        0.01,                 # Divergence angle (again, typically same value)
        sens.LidarReturnMode_STRONGEST_RETURN
    )
    lidar_2d.SetName("Lidar Sensor 2D")
    lidar_2d.SetLag(0)
    lidar_2d.SetCollectionWindow(1. / 5)

    # -----------------------------------------------------------------
    # Create a filter graph for post-processing the data from the 2D lidar
    # -----------------------------------------------------------------
    lidar_2d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    lidar_2d.PushFilter(sens.ChFilterVisualize(800, 1, "Raw Lidar Depth Data 2D"))
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth2D())
    lidar_2d.PushFilter(sens.ChFilterVisualizePointCloud2D(640, 480, 1.0, "Lidar Point Cloud 2D"))
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
        # Set lidar to orbit around the box
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
vertical_samples = 1

# Horizontal and vertical field of view (radians)
horizontal_fov = 2 * math.pi  # 360 degrees
max_vert_angle = 0
min_vert_angle = 0

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

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with:
# chrono.SetChronoDataPath('path/to/data')

main()