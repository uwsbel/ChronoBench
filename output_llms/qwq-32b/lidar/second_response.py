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
    side = 1.0  # Box dimensions
    box_body = chrono.ChBodyEasyBox(side, side, side, 1000)  # density 1000 kg/m³
    box_body.SetPos(chrono.ChVectorD(0, 0, 0))
    box_body.SetBodyFixed(True)
    mphysicalSystem.Add(box_body)

    # -----------------------
    # Create a sensor manager
    # -----------------------
    manager = sens.ChSensorManager(mphysicalSystem)

    # ------------------------------------------------
    # Create a lidar and add it to the sensor manager
    # ------------------------------------------------
    offset_pose = chrono.ChFrameD(
        chrono.ChVectorD(-12, 0, 1),
        chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))
    )
    lidar = sens.ChLidarSensor(
        box_body,              # Attached to the box
        update_rate,           # Scanning rate in Hz
        offset_pose,           # Offset pose
        horizontal_samples,    # Number of horizontal samples
        vertical_samples,      # Number of vertical channels
        horizontal_fov,        # Horizontal field of view
        max_vert_angle,        # Maximum vertical field of view
        min_vert_angle,        # Minimum vertical field of view
        100.0,                 # Maximum lidar range
        sens.LidarBeamShape_RECTANGULAR,  # Beam shape
        sample_radius,         # Sample radius
        divergence_angle,      # Horizontal divergence angle
        divergence_angle,      # Vertical divergence angle
        return_mode            # Return mode
    )
    lidar.SetName("Lidar Sensor")
    lidar.SetLag(lag)
    lidar.SetCollectionWindow(collection_time)

    # Add filters for the first lidar
    if noise_model == "CONST_NORMAL_XYZI":
        lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    elif noise_model == "NONE":
        pass

    if vis:
        lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth Data"))

    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    if vis:
        lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
    lidar.PushFilter(sens.ChFilterXYZIAccess())

    manager.AddSensor(lidar)

    # -------------------------------------
    # Create a 2D lidar sensor
    # -------------------------------------
    offset_pose_2d = chrono.ChFrameD(
        chrono.ChVectorD(0, 0, 1),  # Position relative to box
        chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 0, 1))  # Orientation
    )
    lidar_2d = sens.ChLidarSensor(
        box_body,               # Attached to the box
        update_rate,            # Scanning rate
        offset_pose_2d,         # Offset pose
        horizontal_samples,     # Horizontal samples (same as before)
        1,                     # 2D lidar: 1 vertical channel
        horizontal_fov,        # Horizontal FOV
        0.0,                   # Max vertical angle (0 for horizontal plane)
        0.0,                   # Min vertical angle
        100.0,                 # Max range
        sens.LidarBeamShape_RECTANGULAR,
        sample_radius,
        divergence_angle,
        divergence_angle,
        return_mode
    )
    lidar_2d.SetName("2D Lidar Sensor")
    lidar_2d.SetLag(lag)
    lidar_2d.SetCollectionWindow(collection_time)

    # Add filters for the 2D lidar
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
        # Update lidar position (orbit around the box)
        lidar.SetOffsetPose(
            chrono.ChFrameD(
                chrono.ChVectorD(
                    -orbit_radius * math.cos(ch_time * orbit_rate),
                    -orbit_radius * math.sin(ch_time * orbit_rate),
                    1
                ),
                chrono.Q_from_AngAxis(ch_time * orbit_rate, chrono.ChVectorD(0, 0, 1))
            )
        )

        # Access the XYZI buffer from the first lidar
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            print('XYZI buffer received from lidar. Resolution: {0}x{1}'.format(xyzi_buffer.Width, xyzi_buffer.Height))
            print('Max Value: {0}'.format(np.max(xyzi_data)))

        # Update sensor manager
        manager.Update()

        # Step the simulation
        mphysicalSystem.DoStepDynamics(step_size)
        ch_time = mphysicalSystem.GetChTime()

    print("Sim time:", end_time, "Wall time:", time.time() - t1)


# -----------------
# Lidar parameters
# -----------------
noise_model = "NONE"  # No noise model
return_mode = sens.LidarReturnMode_STRONGEST_RETURN
update_rate = 5.0
horizontal_samples = 800
vertical_samples = 300
horizontal_fov = 2 * chrono.CH_PI  # 360 degrees
max_vert_angle = chrono.CH_PI / 12
min_vert_angle = -chrono.CH_PI / 6
lag = 0
collection_time = 1.0 / update_rate
sample_radius = 2
divergence_angle = 0.003

# ---------------------
# Simulation parameters
# ---------------------
step_size = 1e-3
end_time = 40.0
vis = True
out_dir = "SENSOR_OUTPUT/"
side = 1.0  # Box side length

main()