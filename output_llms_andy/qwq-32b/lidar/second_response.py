import pychrono.core as chrono
import pychrono.sensor as sens
import numpy as np
import time
import math

def main():
    mphysicalSystem = chrono.ChSystemNSC()

    # Create a box instead of the mesh
    side = 2.0  # Box dimensions (each side length)
    box_body = chrono.ChBodyEasyBox(side, side, side, 1000)  # density 1000 kg/m³
    box_body.SetPos(chrono.ChVectorD(0, 0, 0))
    box_body.SetBodyFixed(True)
    mphysicalSystem.Add(box_body)

    # Create sensor manager
    manager = sens.ChSensorManager(mphysicalSystem)

    # Create first lidar (original 3D lidar)
    offset_pose = chrono.ChFrameD(
        chrono.ChVectorD(-12, 0, 1),
        chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))
    )
    lidar = sens.ChLidarSensor(
        box_body,              # Attached to box
        update_rate,           # Scanning rate
        offset_pose,           # Offset pose
        horizontal_samples,     # Horizontal samples
        vertical_samples,      # Vertical channels
        horizontal_fov,        # Horizontal FOV
        max_vert_angle,        # Max vertical angle
        min_vert_angle,        # Min vertical angle
        100.0,                 # Max range
        sens.LidarBeamShape_RECTANGULAR,
        sample_radius,         # Sample radius
        divergence_angle,      # Divergence angles
        divergence_angle,
        return_mode            # Return mode
    )
    lidar.SetName("3D Lidar Sensor")
    lidar.SetLag(lag)
    lidar.SetCollectionWindow(collection_time)

    # Add filters for first lidar
    if noise_model == "CONST_NORMAL_XYZI":
        lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    elif noise_model == "NONE":
        pass

    if vis:
        lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw 3D Lidar Data"))

    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar.PushFilter(sens.ChFilterPCfromDepth())

    if vis:
        lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "3D Lidar Point Cloud"))

    lidar.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar)

    # Create second 2D lidar
    offset_pose_2d = chrono.ChFrameD(
        chrono.ChVectorD(0, 0, 1.5),  # Positioned on top of the box
        chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))
    )
    lidar_2d = sens.ChLidarSensor(
        box_body,                # Attached to box
        update_rate,             # Same update rate
        offset_pose_2d,          # Offset pose
        horizontal_samples,      # Horizontal samples
        1,                      # Single vertical channel (2D)
        horizontal_fov,          # Full horizontal FOV
        0.0,                    # Max vertical angle (0 for 2D)
        0.0,                    # Min vertical angle (0 for 2D)
        100.0,                  # Max range
        sens.LidarBeamShape_RECTANGULAR,
        sample_radius,           # Sample radius
        divergence_angle,        # Divergence angles
        divergence_angle,
        return_mode              # Return mode
    )
    lidar_2d.SetName("2D Lidar Sensor")
    lidar_2d.SetLag(lag)
    lidar_2d.SetCollectionWindow(collection_time)

    # Add filters for second lidar
    if noise_model == "CONST_NORMAL_XYZI":
        lidar_2d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    elif noise_model == "NONE":
        pass

    if vis:
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, 1, "Raw 2D Lidar Data"))

    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())

    if vis:
        lidar_2d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "2D Lidar Point Cloud"))

    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar_2d)

    # Simulation loop
    orbit_radius = 10
    orbit_rate = 0.1
    ch_time = 0.0

    while ch_time < end_time:
        # Update first lidar's orbiting pose
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

        # Access data from first lidar (example)
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            print(f"3D XYZI buffer received. Resolution: {xyzi_buffer.Width}x{xyzi_buffer.Height}")
            print(f"Max Value: {np.max(xyzi_data)}")

        # Update sensor manager
        manager.Update()

        # Simulation step
        mphysicalSystem.DoStepDynamics(step_size)
        ch_time = mphysicalSystem.GetChTime()

    print(f"Sim time: {end_time}, Wall time: {time.time() - t1}")

# Lidar parameters
noise_model = "NONE"
return_mode = sens.LidarReturnMode_STRONGEST_RETURN
update_rate = 5.0
horizontal_samples = 800
vertical_samples = 300
horizontal_fov = 2 * chrono.CH_PI
max_vert_angle = chrono.CH_PI / 12
min_vert_angle = -chrono.CH_PI / 6
lag = 0
collection_time = 1.0 / update_rate
sample_radius = 2
divergence_angle = 0.003

# Simulation parameters
side = 2.0  # Box dimensions
step_size = 1e-3
end_time = 40.0
save = False
vis = True
out_dir = "SENSOR_OUTPUT/"

main()