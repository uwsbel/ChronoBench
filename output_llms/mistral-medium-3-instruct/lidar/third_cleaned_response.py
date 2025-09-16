import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh
import pychrono.vehicle.artcar as artcar

import numpy as np
import time
import math

def main():
    
    
    
    mphysicalSystem = chrono.ChSystemNSC()

    
    
    
    vehicle = artcar.ChVehicleARTcar()
    vehicle.SetChassisFixed(False)
    vehicle.SetChassisMass(1000)
    vehicle.SetChassisInertiaXX(chrono.ChVector3d(100, 100, 100))
    vehicle.SetChassisCOM(chrono.ChVector3d(0, 0, 0.5))
    vehicle.SetChassisDimensions(chrono.ChVector3d(2.0, 1.5, 0.5))
    vehicle.SetWheelRadius(0.3)
    vehicle.SetWheelWidth(0.2)
    vehicle.SetWheelMass(10)
    vehicle.SetWheelInertia(0.1)
    vehicle.SetWheelSuspensionTravel(0.1)
    vehicle.SetWheelSuspensionStiffness(10000)
    vehicle.SetWheelSuspensionDamping(1000)
    vehicle.SetWheelTireStiffness(50000)
    vehicle.SetWheelTireDamping(500)
    vehicle.SetWheelTireFriction(0.8)

    
    vehicle.Initialize(chrono.ChCoordinatorys(chrono.ChVector3d(0, 0, 0.5), chrono.QUNIT))

    
    mphysicalSystem.Add(vehicle.GetSystem())

    
    
    
    driver = veh.ChDriver()
    driver.SetSteering(0.0)
    driver.SetThrottle(0.5)
    driver.SetBraking(0.0)

    
    
    
    terrain = veh.ChTerrain(mphysicalSystem)
    terrain.SetContactMaterialProperties(1e6, 0.8, 0.4)
    terrain.SetTexture(chrono.GetChronoDataFile("textures/terrain.jpg"))
    terrain.SetColor(chrono.ChColor(0.8, 0.8, 0.8))
    terrain.SetHeightField(0, 0, 100, 100, 0, 0)  

    
    
    
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
        
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw 2D Lidar Depth Data"))
    
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
    
    manager.AddSensor(lidar_2d)

    
    
    
    camera_offset = chrono.ChFramed(
        chrono.ChVector3d(-2.0, 0, 1.5), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    camera = sens.ChCameraSensor(
        vehicle.GetChassisBody(),  
        update_rate,               
        camera_offset,             
        640,                       
        480,                       
        chrono.CH_PI / 3,          
        100.0                      
    )
    camera.SetName("Third Person Camera")
    camera.SetLag(lag)
    camera.SetCollectionWindow(collection_time)
    if vis:
        camera.PushFilter(sens.ChFilterVisualize(640, 480, "Camera View"))
    camera.PushFilter(sens.ChFilterRGBA8Access())
    manager.AddSensor(camera)

    
    
    
    orbit_radius = 10
    orbit_rate = 0.1
    ch_time = 0.0

    render_time = 0
    t1 = time.time()

    while ch_time < end_time:
        
        driver.SetThrottle(0.3)
        driver.SetSteering(0.1 * math.sin(ch_time * 0.5))

        
        vehicle.Synchronize(ch_time)
        vehicle.Update()

        
        terrain.Synchronize(ch_time)

        
        manager.Update()

        
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


sample_radius = 2


divergence_angle = 0.003






step_size = 1e-3


end_time = 40.0


save = False


vis = True


out_dir = "SENSOR_OUTPUT/"






main()