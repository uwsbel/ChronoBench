import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as vehicle
import pychrono.irrlicht as chronoirr

import numpy as np
import time
import math

def main():
    # -----------------
    # Create the system
    # -----------------
    # Create vehicle system
    vehicle_system = vehicle.ChVehicleSystem()
    
    # Create and initialize vehicle
    vehicle_params = vehicle.ChVehicleParameters()
    vehicle_params.vehicle_type = vehicle.VehicleType_ARTCAR
    vehicle_params.mass = 1500.0
    vehicle_params.wheel_radius = 0.3
    vehicle_params.wheel_width = 0.2
    vehicle_params.wheel_mass = 10.0
    vehicle_params.chassis_mass = 500.0
    vehicle_params.chassis_width = 2.0
    vehicle_params.chassis_length = 4.0
    vehicle_params.chassis_height = 1.0
    
    artcar = vehicle.ChARTCar(vehicle_system, vehicle_params)
    artcar.Initialize()
    
    # Create driver
    driver = vehicle.ChDriver()
    driver.Initialize(artcar.GetDriverInterface())

    # Create terrain
    terrain = vehicle.ChTerrain()
    terrain.SetMaterial(chrono.ChMaterialSurface())
    terrain.SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
    terrain.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
    terrain.Initialize(vehicle_system, vehicle.ChTerrainType_PLANE, 1000, 1000, 0.1)
    
    # -----------------------
    # Create a sensor manager
    # -----------------------
    manager = sens.ChSensorManager(vehicle_system)

    # ------------------------------------------------
    # Create a lidar and add it to the sensor manager
    # ------------------------------------------------
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(1.0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    
    # Lidar parameters
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
    return_mode = sens.LidarReturnMode_STRONGEST_RETURN
    
    # Create 3D lidar sensor
    lidar = sens.ChLidarSensor(
        artcar.GetChassis(),  # Attached to vehicle chassis
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
    
    # Add noise model if needed
    noise_model = "NONE"
    if noise_model == "CONST_NORMAL_XYZI":
        lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    
    # Add visualization and data filters
    vis = True
    if vis:
        lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth Data"))
    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    if vis:
        lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
    lidar.PushFilter(sens.ChFilterXYZIAccess())
    
    # Add the lidar to the sensor manager
    manager.AddSensor(lidar)
    
    # Create 2D lidar sensor
    lidar_2d = sens.ChLidarSensor(
        artcar.GetChassis(),  # Attached to vehicle chassis
        update_rate,
        offset_pose,
        horizontal_samples,
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
    lidar_2d.SetName("2D Lidar Sensor")
    lidar_2d.SetLag(lag)
    lidar_2d.SetCollectionWindow(collection_time)
    
    if noise_model == "CONST_NORMAL_XYZI":
        lidar_2d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    
    if vis:
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, 1, "Raw 2D Lidar Depth Data"))
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
    
    # Add the 2D lidar to the sensor manager
    manager.AddSensor(lidar_2d)
    
    # Add third person camera
    camera = sens.ChCameraSensor(
        artcar.GetChassis(),
        chrono.ChFramed(
            chrono.ChVector3d(5, 0, 1.5),
            chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
        ),
        640,
        480,
        90.0,
        0.1,
        1000.0
    )
    camera.SetName("Third Person Camera")
    camera.PushFilter(sens.ChFilterVisualizeRGB())
    manager.AddSensor(camera)

    # ---------------
    # Simulate system
    # ---------------
    ch_time = 0.0
    end_time = 40.0
    step_size = 1e-3
    render_time = 0
    t1 = time.time()

    while ch_time < end_time:
        # Update vehicle and driver
        vehicle_system.Synchronize()
        driver.Synchronize()
        terrain.Synchronize()
        
        # Update sensor manager
        manager.Update()
        
        # Access lidar data
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            print('XYZI buffer received from lidar. Lidar resolution: {0}x{1}'.format(xyzi_buffer.Width, xyzi_buffer.Height))
            print('Max Value: {0}'.format(np.max(xyzi_data)))

        # Perform step of dynamics
        vehicle_system.DoStepDynamics(step_size)

        # Get the current time of the simulation
        ch_time = vehicle_system.GetChTime()

    print("Sim time:", end_time, "Wall time:", time.time() - t1)

if __name__ == "__main__":
    main()