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
    side = 2.0  # Box dimensions
    box_body = chrono.ChBodyEasyBox(side, side, side, 1000)  # Density 1000 kg/m³
    box_body.SetPos(chrono.ChVectorD(0, 0, 0))  # Position the box at origin
    box_body.SetFixed(True)  # Make it static

    # Add visual shape with texture
    box_shape = chrono.ChVisualShapeBox(chrono.ChVectorD(side, side, side))
    box_shape.SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"), 100, 100)
    box_body.AddVisualShape(box_shape)
    mphysicalSystem.Add(box_body)

    # -----------------------
    # Create a sensor manager
    # -----------------------
    manager = sens.ChSensorManager(mphysicalSystem)

    # ---------------------
    # Create 3D Lidar Sensor
    # ---------------------
    offset_pose = chrono.ChFrameD(
        chrono.ChVectorD(-12, 0, 1),
        chrono.QuatFromAngleAxis(0, chrono.ChVectorD(0, 1, 0))
    )

    lidar = sens.ChLidarSensor(
        box_body,              # Attached to box
        update_rate,           # Scanning rate in Hz
        offset_pose,           # Offset pose
        horizontal_samples,    # Number of horizontal samples
        vertical_samples,      # Number of vertical channels
        horizontal_fov,        # Horizontal field of view
        max_vert_angle,        # Maximum vertical field of view
        min_vert_angle,        # Minimum vertical field of view
        100.0,                # Maximum lidar range
        sens.LidarBeamShape_RECTANGULAR,  # Beam shape
        sample_radius,         # Sample radius
        divergence_angle,      # Divergence angle X
        divergence_angle,      # Divergence angle Y
        return_mode            # Return mode
    )
    lidar.SetName("3D Lidar Sensor")
    lidar.SetLag(lag)
    lidar.SetCollectionWindow(collection_time)

    # Add filters for 3D lidar
    if noise_model == "CONST_NORMAL_XYZI":
        lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    elif noise_model == "NONE":
        pass

    if vis:
        lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "3D Raw Data"))

    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    if vis:
        lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "3D Point Cloud"))
    lidar.PushFilter(sens.ChFilterXYZIAccess())

    manager.AddSensor(lidar)

    # ---------------------
    # Create 2D Lidar Sensor
    # ---------------------
    horizontal_samples_2d = 800
    vertical_samples_2d = 1  # Single vertical channel for 2D

    offset_pose_2d = chrono.ChFrameD(
        chrono.ChVectorD(0, 0, 2),  # Position above the box
        chrono.QUNIT  # No rotation
    )

    lidar_2d = sens.ChLidarSensor(
        box_body,                # Attached to box
        update_rate,             # Scanning rate
        offset_pose_2d,          # Offset pose
        horizontal_samples_2d,   # Horizontal samples
        vertical_samples_2d,    # Vertical samples (1 for 2D)
        horizontal_fov,          # Same FOV
        0.0,                    # Max vertical angle (0 for 2D)
        0.0,                    # Min vertical angle (0 for 2D)
        100.0,                  # Range
        sens.LidarBeamShape_RECTANGULAR,
        sample_radius,
        divergence_angle,
        divergence_angle,
        return_mode
    )
    lidar_2d.SetName("2D Lidar Sensor")
    lidar_2d.SetLag(lag)
    lidar_2d.SetCollectionWindow(collection_time)

    # Add filters for 2D lidar
    if noise_model == "CONST_NORMAL_XYZI":
        lidar_2d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    elif noise_model == "NONE":
        pass

    if vis:
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples_2d, vertical_samples_2d, "2D Raw Data"))

    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    if vis:
        lidar_2d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "2D Point Cloud"))
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())

    manager.AddSensor(lidar_2d)

    # ---------------
    # Simulation Loop
    # ---------------
    orbit_radius = 10
    orbit_rate = 0.1
    ch_time = 0.0

    render_time = 0
    t1 = time.time()

    while ch_time < end_time:
        # Update 3D lidar position (orbit)
        lidar.SetOffsetPose(
            chrono.ChFrameD(
                chrono.ChVectorD(
                    -orbit_radius * math.cos(ch_time * orbit_rate),
                    -orbit_radius * math.sin(ch_time * orbit_rate),
                    1
                ),
                chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVectorD(0, 0, 1))
            )
        )

        # Access lidar data
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZI()
            print('3D XYZI buffer received. Resolution: {0}x{1}'.format(xyzi_buffer.Width, xyzi_buffer.Height))
            print('Max Value: {0}'.format(np.max(xyzi_data)))

        # Update sensor manager
        manager.Update()

        # Step simulation
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
horizontal_fov = 2 * chrono.CH_PI
max_vert_angle = chrono.CH_PI / 12
min_vert_angle = -chrono.CH_PI / 6
lag = 0
collection_time = 1. / update_rate
sample_radius = 2
divergence_angle = 0.003

# ---------------------
# Simulation parameters
# ---------------------
step_size = 1e-3
end_time = 40.0
vis = True
out_dir = "SENSOR_OUTPUT/"

if __name__ == '__main__':
    main()

import pychrono.core as chrono
import pychrono.sensor as sens
import numpy as np
import time
import math

def main():
    # Create the simulation system
    mphysicalSystem = chrono.ChSystemNSC()

    # Create box body
    side = 2.0  # Box dimensions
    box_body = chrono.ChBodyEasyBox(side, side, side, 1000)
    box_body.SetPos(chrono.ChVectorD(0, 0, 0))
    box_body.SetFixed(True)
    mphysicalSystem.Add(box_body)

    # Add visual shape with texture
    box_shape = chrono.ChVisualShapeBox(chrono.ChVectorD(side, side, side))
    box_shape.SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"), 100, 100)
    box_body.AddVisualShape(box_shape)

    # Initialize sensor manager
    manager = sens.ChSensorManager(mphysicalSystem)

    # Create 3D Lidar Sensor
    offset_pose_3d = chrono.ChFrameD(
        chrono.ChVectorD(-12, 0, 1),
        chrono.QuatFromAngleAxis(0, chrono.ChVectorD(0, 1, 0))
    )
    lidar_3d = sens.ChLidarSensor(
        box_body,
        update_rate,
        offset_pose_3d,
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
    lidar_3d.SetName("3D Lidar")
    lidar_3d.SetLag(lag)
    lidar_3d.SetCollectionWindow(collection_time)

    # Add filters for 3D lidar
    if noise_model == "CONST_NORMAL_XYZI":
        lidar_3d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    if vis:
        lidar_3d.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "3D Raw Data"))
    lidar_3d.PushFilter(sens.ChFilterDIAccess())
    lidar_3d.PushFilter(sens.ChFilterPCfromDepth())
    if vis:
        lidar_3d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "3D Point Cloud"))
    lidar_3d.PushFilter(sens.ChFilterXYZIAccess())

    # Create 2D Lidar Sensor
    offset_pose_2d = chrono.ChFrameD(
        chrono.ChVectorD(0, 0, 2),
        chrono.QUNIT
    )
    lidar_2d = sens.ChLidarSensor(
        box_body,
        update_rate,
        offset_pose_2d,
        horizontal_samples_2d,
        1,
        horizontal_fov,
        0.0,
        0.0,
        100.0,
        sens.LidarBeamShape_RECTANGULAR,
        sample_radius,
        divergence_angle,
        divergence_angle,
        return_mode
    )
    lidar_2d.SetName("2D Lidar")
    lidar_2d.SetLag(lag)
    lidar_2d.SetCollectionWindow(collection_time)

    # Add filters for 2D lidar
    if noise_model == "CONST_NORMAL_XYZI":
        lidar_2d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    if vis:
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples_2d, 1, "2D Raw Data"))
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    if vis:
        lidar_2d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "2D Point Cloud"))
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())

    # Add sensors to manager
    manager.AddSensor(lidar_3d)
    manager.AddSensor(lidar_2d)

    # Simulation loop
    orbit_radius = 10
    orbit_rate = 0.1
    ch_time = 0.0
    t1 = time.time()

    while ch_time < end_time:
        # Update 3D lidar position
        lidar_3d.SetOffsetPose(
            chrono.ChFrameD(
                chrono.ChVectorD(
                    -orbit_radius * math.cos(ch_time * orbit_rate),
                    -orbit_radius * math.sin(ch_time * orbit_rate),
                    1
                ),
                chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVectorD(0, 0, 1))
            )
        )

        # Access sensor data
        for sensor in [lidar_3d, lidar_2d]:
            buffer = sensor.GetMostRecentXYZIBuffer()
            if buffer.HasData():
                data = buffer.GetXYZI()
                print(f"{sensor.GetName()} buffer received. Resolution: {buffer.Width}x{buffer.Height}")
                print(f"Max Value: {np.max(data)}")

        # Update and step simulation
        manager.Update()
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
collection_time = 1. / update_rate
sample_radius = 2
divergence_angle = 0.003
horizontal_samples_2d = 800

# Simulation parameters
step_size = 1e-3
end_time = 40.0
vis = True
out_dir = "SENSOR_OUTPUT/"

if __name__ == "__main__":
    main()