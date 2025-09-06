import pychrono.core as chrono
import pychrono.sensor as sens
import numpy as np
import time
import math
from pychrono.vehicle import Vehicle, ArticulatedVehicle, ChWheeledVehicle

def main():
    
    
    
    mphysicalSystem = chrono.ChSystemNSC()

    
    
    
    car = veh.ARTcar(mphysicalSystem)
    car.SetContactMethod(chrono.ChContactMethod.SMC)
    car.SetChassisCollisionType(chrono.ChCollisionType.BOX)
    car.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
    car.Initialize()

    
    
    
    driver = veh.ChDriver(car)
    driver.SetInput(veh.ChDriver.INPUT_ACCELERATOR, 0.0)
    driver.SetInput(veh.ChDriver.INPUT_BRAKE, 0.0)
    driver.SetInput(veh.ChDriver.INPUT_STEERING, 0.0)

    
    
    
    terrain = veh.RigidTerrain(mphysicalSystem)
    patch = terrain.AddPatch(chrono.ChMaterialSurfaceSMC(), chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 100, 100)
    patch.SetTexture(chrono.GetChronoDataFile("textures/terrain.png"), 100, 100)
    terrain.Initialize()

    
    
    
    manager = sens.ChSensorManager(mphysicalSystem)

    
    
    
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(1.0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    lidar = sens.ChLidarSensor(
        car.GetChassis(),  
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
        car.GetChassis(),  
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

    
    
    
    orbit_radius = 10
    orbit_rate = 0.1
    ch_time = 0.0
    render_time = 0
    t1 = time.time()

    while ch_time < end_time:
        
        
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            print('XYZI buffer received from lidar. Lidar resolution: {0}x{1}'.format(xyzi_buffer.Width, xyzi_buffer.Height))
            print('Max Value: {0}'.format(np.max(xyzi_data)))

        
        manager.Update()

        
        mphysicalSystem.DoStepDynamics(step_size)
        car.Synchronize(step_size)
        terrain.Synchronize(step_size)
        driver.Synchronize(step_size)

        
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

    
    
    
    

import pychrono.vehicle as veh

main()