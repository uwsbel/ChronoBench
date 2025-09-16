import pychrono.core as chrono
import pychrono.sensor as sens
import numpy as np
import time
import math

def main():
    
    
    
    mphysicalSystem = chrono.ChSystemNSC()

    
    
    
    artcar = chrono.ARTcar(mphysicalSystem)
    artcar.SetVehicleModel("ARTCAR_SIMPLE")
    artcar.SetTireModel("RIGID")
    artcar.SetEngineType(chrono.ARTcar.EngineType.ELECTRIC)
    artcar.SetInitialPosition(chrono.ChVector3d(0, 0, 0.5))
    artcar.SetInitialVelocity(chrono.ChVector3d(0, 0, 0))

    
    
    
    manager = sens.ChSensorManager(mphysicalSystem)

    
    
    
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(1.0, 0, 1),
        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    lidar = sens.ChLidarSensor(
        artcar.GetChassis(),  
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
    
    manager.AddSensor(lidar)

    
    lidar_2d = sens.ChLidarSensor(
        artcar.GetChassis(),  
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
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw 2D Lidar Depth Data"))
    
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
    
    manager.AddSensor(lidar_2d)

    
    
    
    terrain = chrono.ChTerrain()
    terrain.Create(mphysicalSystem, "terrain", chrono.ChVector3d(0, 0, -1), chrono.ChVector3d(50, 50, 0), 5, 5)
    terrain.SetTexture(chrono.GetChronoDataFile("textures/grass.jpg"), 20, 20)
    terrain.SetMaterialProperties(0.5, 0.8, 1.0)

    
    
    
    camera = sens.ChThirdPersonCamera()
    camera.Attach(artcar.GetChassis())
    camera.SetOffset(chrono.ChVector3d(0, 0, 2))
    camera.SetAimPoint(chrono.ChVector3d(0, 0, 0.5))
    camera.SetVerticalAngle(0.3)
    manager.AddSensor(camera)

    
    
    
    orbit_radius = 10
    orbit_rate = 0.1
    ch_time = 0.0
    render_time = 0
    t1 = time.time()

    driver = chrono.ARTcarDriver(artcar)
    driver.SetThrottle(0.5)

    while ch_time < end_time:
        
        
        
        
        
        
        
        
        
        
        

        
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            print('XYZI buffer received from lidar. Lidar resolution: {0}x{1}'.format(xyzi_buffer.Width, xyzi_buffer.Height))
            print('Max Value: {0}'.format(np.max(xyzi_data)))

        
        manager.Update()

        
        mphysicalSystem.DoStepDynamics(step_size)
        artcar.Synchronize()
        driver.Synchronize()
        terrain.Synchronize()

        
        ch_time = mphysicalSystem.GetChTime()
        print("Sim time:", end_time, "Wall time:", time.time() - t1)

    
    
    
    
    
    noise_model = "CONST_NORMAL_XYZI"  
    

    
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

    
    
    
    
    step_size = 1e-3

    
    end_time = 40.0

    
    save = False

    
    vis = True

    
    out_dir = "SENSOR_OUTPUT/"

    
    
    
    

    main()