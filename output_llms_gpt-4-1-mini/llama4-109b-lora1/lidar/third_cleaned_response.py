import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh

import numpy as np
import time
import math


def main():
    
    
    
    mphysicalSystem = veh.ChSystemNSC_artcar()

    
    vehicle = mphysicalSystem.GetVehicle()
    vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
    vehicle.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    vehicle.SetChassisFixed(False)
    vehicle.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 1.0)))

    
    vehicle.Initialize()

    
    driver = veh.ChDriver(vehicle)
    driver.Initialize()

    
    terrain = veh.ChTerrain()
    terrain.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    terrain.SetContactMethod(chrono.ChContactMethod_NSC)
    terrain.Initialize(1000, 0.005, 0.5)

    
    terrain.GetGroundBody().GetVisualModel().GetShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
    terrain.GetGroundBody().GetVisualModel().GetShape(0).SetColor(chrono.ChColor(0.8, 0.8, 0.8))

    
    
    
    manager = sens.ChSensorManager(mphysicalSystem)

    
    
    
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(1.0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
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
    
    manager.AddSensor(lidar)

    
    lidar_2d =  sens.ChLidarSensor(
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
    
    manager.AddSensor(lidar_2d)

    
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(0.0, -5, 1.5), chrono.QuatFromAngleAxis(chrono.CH_PI / 6, chrono.ChVector3d(1, 0, 0))
    )
    camera = sens.ChCameraSensor(
        vehicle.GetChassisBody(),  
        update_rate,            
        offset_pose,            
        800,                    
        600                     
    )
    camera.SetName("Third Person Camera Sensor")
    camera.SetLag(lag)
    camera.SetCollectionWindow(collection_time)
    if vis:
        camera.PushFilter(sens.ChFilterVisualize(800, 600, "Third Person Camera"))
    manager.AddSensor(camera)

    
    
    
    orbit_radius = 10
    orbit_rate = 0.1
    ch_time = 0.0

    render_time = 0
    t1 = time.time()

    while ch_time < end_time:
        
        driver.Advance(step_size)
        terrain.Advance(step_size)
        manager.Update()

        
        vehicle.Synchronize(time, step_size)
        terrain.Synchronize(time, step_size)
        manager.Synchronize(time)

        
        mphysicalSystem.DoStepDynamics(step_size)

        
        ch_time = mphysicalSystem.GetChTime()

    print("Sim time:", end_time, "Wall time:", time.time() - t1)









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


print("error happened with only start ```python")