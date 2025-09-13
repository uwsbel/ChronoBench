import pychrono.core as chrono
import pychrono.sensor as sens

import numpy as np
import time
import math

def main():
    
    
    
    vehicle = chrono.ChARTcar()
    vehicle.SetInitPosition(chrono.ChVectorD(0, 0, 0.5))
    vehicle.SetInitVelocity(chrono.ChVectorD(0, 0, 0))
    vehicle.SetBodyFixed(True)
    vehicle.Initialize()

    
    vehicle.SetLength(4.5)
    vehicle.SetWidth(1.8)
    vehicle.SetHeight(1.6)
    vehicle.SetWheelbase(2.7)
    vehicle.SetTrack(1.5)

    
    vehicle_system = chrono.ChSystemNSC()
    vehicle_system.Add(vehicle)

    
    
    
    driver = chrono.ChIrrNodeApplicationDriver()
    driver.Initialize()
    driver.SetSteeringMode(chrono.ChIrrNodeApplicationDriver.STEERING_MODE_CURVATURE)
    driver.SetThrottleMode(chrono.ChIrrNodeApplicationDriver.THROTTLE_MODE_TORQUE)
    driver.SetBrakingMode(chrono.ChIrrNodeApplicationDriver.BRAKING_MODE_TORQUE)
    driver.SetGearMode(chrono.ChIrrNodeApplicationDriver.GEAR_MODE_MANUAL)

    
    
    
    terrain = chrono.ChBodyEasyBox(100, 100, 0.1, 1000)
    terrain.SetPos(chrono.ChVector3d(0, 0, -0.1))
    terrain.GetVisualModel().GetShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.png"))
    terrain.SetFixed(True)
    vehicle_system.Add(terrain)

    
    
    
    manager = sens.ChSensorManager(vehicle_system)

    
    
    
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
        
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw 2D Lidar Depth Data"))
    
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
    
    manager.AddSensor(lidar_2d)

    
    camera = sens.ChCameraSensor(
        vehicle.GetChassisBody(),  
        640,  
        480,  
        1.0,  
        0.01,  
        1.0 / 60.0  
    )
    camera.SetName("Third Person Camera")
    camera.SetOffsetPose(chrono.ChFramed(chrono.ChVector3d(-5, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))))
    manager.AddSensor(camera)

    
    
    
    orbit_radius = 10
    orbit_rate = 0.1
    ch_time = 0.0

    render_time = 0
    t1 = time.time()

    while ch_time < end_time:
        
        vehicle.Synchronize(driver.GetInput(), driver.GetOutput())
        driver.Advance()
        terrain.Synchronize()

        
        manager.Update()

        
        vehicle_system.DoStepDynamics(step_size)

        
        ch_time = vehicle_system.GetChTime()

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