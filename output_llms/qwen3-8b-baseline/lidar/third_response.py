import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as chrono_veh
import numpy as np
import time
import math

def main():
    # -----------------
    # Create the system
    # -----------------
    mphysicalSystem = chrono.ChSystemNSC()

    # -----------------------
    # Create the vehicle
    # -----------------------
    vehicle = chrono_veh.ChVehicle()
    vehicle.SetName("ARTcar")
    vehicle.Initialize()
    mphysicalSystem.Add(vehicle)

    # ---------------------
    # Initialize the driver
    # ---------------------
    driver = chrono_veh.ChDriver(vehicle)
    driver.Initialize()

    # ---------------------
    # Create the terrain
    # ---------------------
    terrain = chrono.ChTerrain()
    terrain.SetMaterial(chrono.ChMaterialSurface())
    terrain.SetTexture(chrono.GetChronoDataFile("textures/terrain.jpg"))
    terrain.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
    terrain.SetMaterialProperties(0.5, 0.5, 0.5)
    mphysicalSystem.Add(terrain)

    # -----------------------
    # Create sensor manager
    # -----------------------
    manager = sens.ChSensorManager(mphysicalSystem)

    # --------------------------
    # Define lidar parameters
    # --------------------------
    update_rate = 5.0
    horizontal_samples = 800
    vertical_samples = 300
    horizontal_fov = 2 * chrono.CH_PI  # 360 degrees
    max_vert_angle = chrono.CH_PI / 12
    min_vert_angle = -chrono.CH_PI / 6
    sample_radius = 2
    divergence_angle = 0.003
    return_mode = sens.LidarReturnMode_STRONGEST_RETURN
    lag = 0
    collection_time = 1.0 / update_rate
    noise_model = "NONE"
    vis = True

    # -----------------------
    # Create lidar sensors
    # -----------------------
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(1.0, 0, 1), 
        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )

    # 3D Lidar
    lidar = sens.ChLidarSensor(
        vehicle.GetChassisBody(),  # Attach to chassis
        update_rate,
        offset_pose,
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
    lidar.SetName("Lidar Sensor")
    lidar.SetLag(lag)
    lidar.SetCollectionWindow(collection_time)

    # Add filters
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

    # 2D Lidar
    lidar_2d = sens.ChLidarSensor(
        vehicle.GetChassisBody(),
        update_rate,
        offset_pose,
        horizontal_samples,
        1,  # only 1 vertical channel for 2D lidar
        horizontal_fov,
        0.0,  # Maximum vertical field of view
        0.0,  # Minimum vertical field of view
        100.0,
        sens.LidarBeamShape_RECTANGULAR,
        sample_radius,
        divergence_angle,
        divergence_angle,
        return_mode
    )
    lidar_2d.SetName("2D Lidar Sensor")
    lidar_2d.SetLag(lag)
    lidar_2d.SetCollectionWindow(collection_time)
    if noise_model == "CONST_NORMAL_XYZI":
        lidar_2d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    elif noise_model == "NONE":
        pass
    if vis:
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, 1, "Raw 2D Lidar Depth Data"))
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar_2d)

    # ---------------------
    # Add third-person camera
    # ---------------------
    camera_offset = chrono.ChFramed(
        chrono.ChVector3d(0, 0, 2), 
        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    camera = sens.ChCameraSensor(
        vehicle.GetChassisBody(),
        update_rate,
        camera_offset,
        horizontal_fov,
        1.57,  # 90 degrees vertical FOV
        100.0,
        sens.CameraType_PERSPECTIVE,
        0.1,
        100.0,
        1024,
        768
    )
    camera.SetName("Third Person Camera")
    camera.SetLag(lag)
    camera.SetCollectionWindow(collection_time)
    camera.PushFilter(sens.ChFilterVisualize(1024, 768, "Third Person Camera"))
    manager.AddSensor(camera)

    # ---------------------
    # Simulation parameters
    # ---------------------
    step_size = 1e-3
    end_time = 40.0
    save = False
    out_dir = "SENSOR_OUTPUT/"

    # ---------------
    # Simulate system
    # ---------------
    ch_time = 0.0
    render_time = 0
    t1 = time.time()

    while ch_time < end_time:
        # Synchronize driver
        driver.Synchronize(ch_time)

        # Advance vehicle and terrain
        vehicle.Advance(ch_time)
        terrain.Advance(ch_time)

        # Update sensor manager
        manager.Update()

        # Perform step of dynamics
        mphysicalSystem.DoStepDynamics(step_size)

        # Get the current time of the simulation
        ch_time = mphysicalSystem.GetChTime()

    print("Sim time:", end_time, "Wall time:", time.time() - t1)


if __name__ == "__main__":
    main()