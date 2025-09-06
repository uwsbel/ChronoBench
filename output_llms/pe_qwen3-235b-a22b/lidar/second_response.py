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

    # ---------------------
    # Simulation parameters
    # ---------------------
    # Lidar parameters
    noise_model = "NONE"  # No noise model
    return_mode = sens.LidarReturnMode_STRONGEST_RETURN  # Lidar return mode
    update_rate = 5.0  # Update rate in Hz
    horizontal_samples = 800  # Number of horizontal samples
    vertical_samples = 300  # Number of vertical channels
    horizontal_fov = 2 * chrono.CH_PI  # Horizontal field of view (360 degrees)
    max_vert_angle = chrono.CH_PI / 12  # Maximum vertical field of view
    min_vert_angle = -chrono.CH_PI / 6  # Minimum vertical field of view
    lag = 0  # Lag time
    collection_time = 1. / update_rate  # Collection window for the lidar
    sample_radius = 2  # Radius of samples to use
    divergence_angle = 0.003  # Divergence angle (3mm radius as cited by velodyne)
    step_size = 1e-3  # Simulation step size
    end_time = 40.0  # Simulation end time
    vis = True  # Render camera images

    # -----------------------
    # Create the box object
    # -----------------------
    box_side = 2.0  # Box dimensions
    box_body = chrono.ChBodyEasyBox(box_side, box_side, box_side, 1000)
    box_body.SetPos(chrono.ChVector3d(0, 0, 0))
    box_body.SetFixed(True)
    
    # Set texture for the box
    if len(list(box_body.GetVisualShapes())) > 0:
        vis_shape = list(box_body.GetVisualShapes())[0]
        vis_shape.SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
    
    mphysicalSystem.Add(box_body)

    # -----------------------
    # Create sensor manager
    # -----------------------
    manager = sens.ChSensorManager(mphysicalSystem)

    # ------------------------------------------------
    # Create a 3D lidar and add it to the sensor manager
    # ------------------------------------------------
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(-12, 0, 1), 
        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    
    lidar = sens.ChLidarSensor(
        box_body,              # Body lidar is attached to
        update_rate,           # Scanning rate in Hz
        offset_pose,           # Offset pose
        horizontal_samples,    # Number of horizontal samples
        vertical_samples,      # Number of vertical channels
        horizontal_fov,        # Horizontal field of view
        max_vert_angle,        # Maximum vertical field of view
        min_vert_angle,        # Minimum vertical field of view
        100.0,                 # Maximum lidar range
        sens.LidarBeamShape_RECTANGULAR,  # Shape of the lidar beam
        sample_radius,         # Sample radius
        divergence_angle,      # Divergence angle X
        divergence_angle,      # Divergence angle Y
        return_mode            # Return mode for the lidar
    )
    
    lidar.SetName("Lidar Sensor")
    lidar.SetLag(lag)
    lidar.SetCollectionWindow(collection_time)

    # Add filters to the 3D lidar
    if noise_model == "CONST_NORMAL_XYZI":
        lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    
    if vis:
        lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth Data"))
    
    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    
    if vis:
        lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
    
    lidar.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar)

    # ------------------------------------------------
    # Create a 2D lidar and add it to the sensor manager
    # ------------------------------------------------
    offset_pose_2d = chrono.ChFramed(
        chrono.ChVector3d(-5, 0, 1), 
        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    
    lidar_2d = sens.ChLidarSensor(
        box_body,              # Body lidar is attached to
        update_rate,           # Scanning rate in Hz
        offset_pose_2d,        # Offset pose
        horizontal_samples,    # Number of horizontal samples
        1,                     # Vertical samples (1 for 2D)
        horizontal_fov,        # Horizontal field of view
        0.0,                   # Maximum vertical field of view
        0.0,                   # Minimum vertical field of view
        100.0,                 # Maximum lidar range
        sens.LidarBeamShape_RECTANGULAR,  # Shape of the lidar beam
        sample_radius,         # Sample radius
        divergence_angle,      # Divergence angle X
        divergence_angle,      # Divergence angle Y
        return_mode            # Return mode for the lidar
    )
    
    lidar_2d.SetName("2D Lidar Sensor")
    lidar_2d.SetLag(lag)
    lidar_2d.SetCollectionWindow(collection_time)

    # Add filters to the 2D lidar
    if noise_model == "CONST_NORMAL_XYZI":
        lidar_2d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    
    if vis:
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, 1, "Raw 2D Lidar Depth Data"))
    
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    
    if vis:
        lidar_2d.PushFilter(sens.ChFilterVisualizePointCloud(640, 1, 1.0, "2D Lidar Point Cloud"))
    
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar_2d)

    # ---------------
    # Simulate system
    # ---------------
    orbit_radius = 10
    orbit_rate = 0.1
    ch_time = 0.0

    t1 = time.time()

    while ch_time < end_time:
        # Set lidar to orbit around the box
        new_offset = chrono.ChFramed(
            chrono.ChVector3d(
                -orbit_radius * math.cos(ch_time * orbit_rate),
                -orbit_radius * math.sin(ch_time * orbit_rate),
                1
            ),
            chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVector3d(0, 0, 1))
        )
        
        lidar.SetOffsetPose(new_offset)
        lidar_2d.SetOffsetPose(new_offset)

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

main()