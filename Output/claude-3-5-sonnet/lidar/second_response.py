import pychrono.core as chrono
import pychrono.sensor as sens

import numpy as np
import time
import math

def main():
    # Create the system
    mphysicalSystem = chrono.ChSystemNSC()

    # Add a box to be sensed by the lidars
    side = 2.0  # Size of the box
    box = chrono.ChBodyEasyBox(side, side, side, 1000)
    box.SetPos(chrono.ChVectorD(0, 0, 0))
    box.SetBodyFixed(True)
    
    # Add texture to the box
    texture = chrono.ChVisualMaterial()
    texture.SetKdTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
    box.GetVisualShape(0).SetMaterial(0, texture)
    
    mphysicalSystem.Add(box)

    # Create a sensor manager
    manager = sens.ChSensorManager(mphysicalSystem)

    # Create a 3D lidar and add it to the sensor manager
    offset_pose = chrono.ChFrameD(
        chrono.ChVectorD(-12, 0, 1), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))
    )
    lidar = sens.ChLidarSensor(
        box,                    # Body lidar is attached to
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
    lidar.SetName("3D Lidar Sensor")
    lidar.SetLag(lag)
    lidar.SetCollectionWindow(collection_time)

    # Create filters for the 3D lidar
    if vis:
        lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "3D Lidar Depth Data"))
    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    if vis:
        lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "3D Lidar Point Cloud"))
    lidar.PushFilter(sens.ChFilterXYZIAccess())

    # Add the 3D lidar to the sensor manager
    manager.AddSensor(lidar)

    # Create a 2D lidar and add it to the sensor manager
    offset_pose_2d = chrono.ChFrameD(
        chrono.ChVectorD(-12, 0, 1), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))
    )
    lidar_2d = sens.ChLidarSensor(
        box,                    # Body lidar is attached to
        update_rate,            # Scanning rate in Hz
        offset_pose_2d,         # Offset pose
        horizontal_samples,     # Number of horizontal samples
        1,                      # Number of vertical channels (1 for 2D lidar)
        horizontal_fov,         # Horizontal field of view
        0,                      # Maximum vertical field of view (0 for 2D lidar)
        0,                      # Minimum vertical field of view (0 for 2D lidar)
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

    # Create filters for the 2D lidar
    if vis:
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, 1, "2D Lidar Depth Data"))
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    if vis:
        lidar_2d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "2D Lidar Point Cloud"))
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())

    # Add the 2D lidar to the sensor manager
    manager.AddSensor(lidar_2d)

    # Simulate system
    orbit_radius = 10
    orbit_rate = 0.1
    ch_time = 0.0

    t1 = time.time()

    while ch_time < end_time:
        # Set lidars to orbit around the box
        orbit_pose = chrono.ChFrameD(
            chrono.ChVectorD(
                -orbit_radius * math.cos(ch_time * orbit_rate),
                -orbit_radius * math.sin(ch_time * orbit_rate),
                1
            ),
            chrono.Q_from_AngAxis(ch_time * orbit_rate, chrono.ChVectorD(0, 0, 1))
        )
        lidar.SetOffsetPose(orbit_pose)
        lidar_2d.SetOffsetPose(orbit_pose)

        # Access the XYZI buffer from 3D lidar
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            print('3D XYZI buffer received. Resolution: {0}x{1}'.format(xyzi_buffer.Width, xyzi_buffer.Height))
            print('3D Max Value: {0}'.format(np.max(xyzi_data)))

        # Access the XYZI buffer from 2D lidar
        xyzi_buffer_2d = lidar_2d.GetMostRecentXYZIBuffer()
        if xyzi_buffer_2d.HasData():
            xyzi_data_2d = xyzi_buffer_2d.GetXYZIData()
            print('2D XYZI buffer received. Resolution: {0}x{1}'.format(xyzi_buffer_2d.Width, xyzi_buffer_2d.Height))
            print('2D Max Value: {0}'.format(np.max(xyzi_data_2d)))

        # Update sensor manager
        manager.Update()

        # Perform step of dynamics
        mphysicalSystem.DoStepDynamics(step_size)

        # Get the current time of the simulation
        ch_time = mphysicalSystem.GetChTime()

    print("Sim time:", end_time, "Wall time:", time.time() - t1)

# Lidar parameters
noise_model = "NONE"
return_mode = sens.LidarReturnMode_STRONGEST_RETURN
update_rate = 5.0
horizontal_samples = 800
vertical_samples = 300
horizontal_fov = 2 * chrono.CH_C_PI
max_vert_angle = chrono.CH_C_PI / 12
min_vert_angle = -chrono.CH_C_PI / 6
lag = 0
collection_time = 1. / update_rate
sample_radius = 2
divergence_angle = 0.003

# Simulation parameters
step_size = 1e-3
end_time = 40.0
save = False
vis = True
out_dir = "SENSOR_OUTPUT/"

main()