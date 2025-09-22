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
    # Add a box to be sensed by a lidar
    # ----------------------------------
    side = 2.0
    mbody = chrono.ChBodyEasyBox(side, side, side, 1000, True, True)
    mbody.SetPos(chrono.ChVector3d(0, 0, 0))
    mbody.SetMass(100)
    mbody.SetInertia(chrono.ChVector3d(10, 10, 10))
    mphysicalSystem.Add(mbody)

    # Create a visual representation of the box
    mbody.AddVisualization("box", chrono.ChColor(0.2, 0.2, 0.8))
    mbody.SetFixed(True)

    # -----------------------
    # Create a sensor manager
    # -----------------------
    manager = sens.ChSensorManager(mphysicalSystem)

    # ------------------------------------------------
    # Create a lidar and add it to the sensor manager
    # ------------------------------------------------
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(-1.5, 0, 1),
        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    update_rate = 5.0
    horizontal_samples = 800
    vertical_samples = 300
    horizontal_fov = 2 * chrono.CH_PI
    max_vert_angle = chrono.CH_PI / 12
    min_vert_angle = -chrono.CH_PI / 6
    max_range = 100.0
    sample_radius = 2
    divergence_angle = 0.003
    return_mode = sens.LidarReturnMode_STRONGEST_RETURN
    lag = 0
    collection_time = 1. / update_rate

    lidar = sens.ChLidarSensor(
        mbody,
        update_rate,
        offset_pose,
        horizontal_samples,
        vertical_samples,
        horizontal_fov,
        max_vert_angle,
        min_vert_angle,
        max_range,
        sens.LidarBeamShape_RECTANGULAR,
        sample_radius,
        divergence_angle,
        divergence_angle,
        return_mode
    )
    lidar.SetName("Lidar Sensor")
    lidar.SetLag(lag)
    lidar.SetCollectionWindow(collection_time)

    # -----------------------------------------------------------------
    # Create a filter graph for post-processing the data from the lidar
    # -----------------------------------------------------------------
    noise_model = "NONE"
    vis = True

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

    # ------------------------------------------------
    # Create a 2D lidar and add it to the sensor manager
    # ------------------------------------------------
    offset_pose_2d = chrono.ChFramed(
        chrono.ChVector3d(0, 0, 1),
        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    update_rate_2d = 5.0
    horizontal_samples_2d = 400
    vertical_samples_2d = 1
    horizontal_fov_2d = chrono.CH_PI
    max_vert_angle_2d = 0.0
    min_vert_angle_2d = 0.0
    max_range_2d = 50.0
    sample_radius_2d = 1
    divergence_angle_2d = 0.001

    lidar_2d = sens.ChLidarSensor(
        mbody,
        update_rate_2d,
        offset_pose_2d,
        horizontal_samples_2d,
        vertical_samples_2d,
        horizontal_fov_2d,
        max_vert_angle_2d,
        min_vert_angle_2d,
        max_range_2d,
        sens.LidarBeamShape_RECTANGULAR,
        sample_radius_2d,
        divergence_angle_2d,
        divergence_angle_2d,
        return_mode
    )
    lidar_2d.SetName("Lidar 2D Sensor")
    lidar_2d.SetLag(lag)
    lidar_2d.SetCollectionWindow(collection_time)

    # -----------------------------------------------------------------
    # Create a filter graph for post-processing the data from the 2D lidar
    # -----------------------------------------------------------------
    lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples_2d, vertical_samples_2d, "Raw Lidar 2D Depth Data"))
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    lidar_2d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar 2D Point Cloud"))
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
    end_time = 40.0
    step_size = 1e-3

    while ch_time < end_time:
        # Access the XYZI buffer from lidar
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            print('XYZI buffer received from lidar. Lidar resolution: {0}x{1}'.format(xyzi_buffer.Width, xyzi_buffer.Height))
            print('Max Value: {0}'.format(np.max(xyzi_data)))

        # Access the XYZI buffer from lidar_2d
        xyzi_buffer_2d = lidar_2d.GetMostRecentXYZIBuffer()
        if xyzi_buffer_2d.HasData():
            xyzi_data_2d = xyzi_buffer_2d.GetXYZIData()
            print('XYZI buffer received from lidar_2d. Lidar resolution: {0}x{1}'.format(xyzi_buffer_2d.Width, xyzi_buffer_2d.Height))
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
    # TODO: Noise models haven't been implemented in python
    # noise_model = "CONST_NORMAL_XYZI"
    # Gaussian noise with constant mean and standard deviation
    noise_model = "NONE"
    # No noise model

    # Lidar return mode
    return_mode = sens.LidarReturnMode_STRONGEST_RETURN
    # return_mode = sens.MEAN_RETURN
    # return_mode = sens.FIRST_RETURN
    # return_mode = sens.LAST_RETURN

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
    # chrono.SetChonoDataPath('path/to/data')

main()