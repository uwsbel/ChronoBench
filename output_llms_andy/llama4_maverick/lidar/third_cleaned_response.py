import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh
import numpy as np
import time
import math

def main():
    
    
    
    step_size = 1e-3
    end_time = 40.0
    vis = True

    
    
    
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
    return_mode = sens.LidarReturnMode_STRONGEST_RETURN
    noise_model = "NONE"

    
    init_loc = chrono.ChVector3d(0, 0, 1.0)
    init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
    vehicle = veh.ARTcar(veh.VehicleSide.kRight, False)
    vehicle.Initialize(init_loc, init_rot)
    mphysicalSystem = vehicle.GetSystem()

    
    terrain_material = chrono.ChMaterialSurfaceSMC(0.9, 0.01, 0.2, 0)
    terrain = veh.RigidTerrain(mphysicalSystem)
    terrain.SetContactMaterial(terrain_material)
    terrain.SetTexture(chrono.GetChronoDataFile("textures/grass.jpg"), 200, 200)
    terrain.SetColor(chrono.ChColor(0.5, 0.8, 0.5))
    map_path = chrono.GetChronoDataFile("sensor/textures/hd_map.png")
    terrain.Initialize(chrono.GetChronoDataFile("terrain/height_maps/N_height_map.bmp"), map_path, 0.0, 255.0, 0.0, 255.0, 1.0)

    
    driver = veh.ChDriver(vehicle, True)

    
    
    
    offset_pose = chrono.ChFramed(chrono.ChVector3d(1.0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))

    lidar = sens.ChLidarSensor(
        vehicle.GetChassisBody(),
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

    
    lidar_2d = sens.ChLidarSensor(
        vehicle.GetChassisBody(),
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
    elif noise_model == "NONE":
        pass

    if vis:
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, 1, "Raw 2D Lidar Depth Data"))
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())

    
    offset_pose_cam = chrono.ChFramed(chrono.ChVector3d(-5, 0, 2), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
    cam = sens.ChCameraSensor(
        vehicle.GetChassisBody(),
        30,
        offset_pose_cam,
        1280,
        720,
        sens.CAMERA_VISUALIZATION
    )
    cam.SetName("Third Person Camera")
    cam.SetLag(0)
    cam.SetCollectionWindow(0)

    
    manager = sens.ChSensorManager(mphysicalSystem)
    manager.AddSensor(lidar)
    manager.AddSensor(lidar_2d)
    manager.AddSensor(cam)

    
    
    
    orbit_radius = 10
    orbit_rate = 0.1
    ch_time = 0.0
    t1 = time.time()

    while ch_time < end_time:
        
        driver_inputs = driver.GetInputs()
        driver_inputs.m_throttle = 0.5
        driver_inputs.m_steering = 0.0
        driver_inputs.m_braking = 0.0
        driver.Synchronize(ch_time)

        
        vehicle.Synchronize(ch_time, driver_inputs, terrain)

        
        terrain.Synchronize(ch_time)

        
        manager.Update()

        
        mphysicalSystem.DoStepDynamics(step_size)

        
        ch_time = mphysicalSystem.GetChTime()

        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            print('XYZI buffer received from lidar. Lidar resolution: {0}x{1}'.format(xyzi_buffer.Width, xyzi_buffer.Height))
            print('Max Value: {0}'.format(np.max(xyzi_data)))

        print("Sim time:", ch_time, "Wall time:", time.time() - t1)

if __name__ == "__main__":
    main()