import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh

import numpy as np
import time
import math






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

def main():
    
    
    
    system = chrono.ChSystemNSC()

    
    vehicle = veh.ARTCar()
    vehicle.Initialize(system)

    
    vehicle.SetChassisCollisionShape(veh.ChCollisionShapeType_BOX, 2.5, 0.8, 0.5)
    vehicle.SetChassisMass(1500)
    vehicle.SetChassisInertia(1000, 1000, 1000)

    
    driver = veh.ChDriver(system, vehicle.GetChassisBody())
    driver.Initialize()

    
    terrain = veh.RigidTerrain(system)
    terrain.Initialize(500, 500, 0.1)
    terrain.SetTexture(chrono.GetChronoDataFile("textures/pavement.jpg"))
    terrain.SetColor(chrono.ChColor(0.5, 0.5, 0.5))

    
    
    
    manager = sens.ChSensorManager(system)

    
    
    
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

    
    offset_pose_camera = chrono.ChFramed(
        chrono.ChVector3d(0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    camera = sens.ChCameraSensor(
        vehicle.GetChassisBody(),  
        30,                      
        offset_pose_camera,      
        640, 480,                
        1.0                      
    )
    camera.SetName("Camera Sensor")
    camera.SetLag(0)
    camera.SetCollectionWindow(1./30.)
    if vis:
        camera.PushFilter(sens.ChFilterVisualize(640, 480, "Camera View"))
    manager.AddSensor(camera)

    
    
    
    ch_time = 0.0

    render_time = 0
    t1 = time.time()

    while ch_time < end_time:
        
        driver.Advance(step_size)
        vehicle.Advance(step_size)

        
        manager.Update()

        
        system.DoStepDynamics(step_size)

        
        ch_time = system.GetChTime()

    print("Sim time:", end_time, "Wall time:", time.time() - t1)


if __name__ == "__main__":
    main()