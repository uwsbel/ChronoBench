import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh

import numpy as np
import time
import math




step_size = 1e-3
end_time = 40.0
vis = True
out_dir = "SENSOR_OUTPUT/"




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

def main():
    
    
    
    car = veh.ARTcar()
    car.SetContactMethod(chrono.ChSystemDynamics.ContactMethod.NSC)
    car.SetChassisCollisionType(veh.ChassisCollisionType_NONE)
    car.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT))
    car.Initialize()

    
    terrain = veh.RigidTerrain(car.GetSystem())
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch_mat.SetFriction(0.9)
    terrain.AddPatch(patch_mat, chrono.ChCoordsysD(chrono.ChVectorD(0,0,0)), 100, 100)
    terrain.Initialize()

    
    driver = veh.RoboticDriver()
    car.SetDriver(driver)
    driver.Initialize(car.GetVehicle(), step_size)

    mphysicalSystem = car.GetSystem()

    
    
    
    manager = sens.ChSensorManager(mphysicalSystem)

    
    lidar_offset = chrono.ChFrameD(
        chrono.ChVectorD(1.0, 0, 1),  
        chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))
    )

    
    
    
    lidar = sens.ChLidarSensor(
        car.GetChassisBody(),          
        update_rate,                   
        lidar_offset,                  
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
    lidar.SetName("3D Lidar Sensor")
    lidar.SetLag(lag)
    lidar.SetCollectionWindow(collection_time)

    
    if noise_model == "CONST_NORMAL_XYZI":
        lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    if vis:
        lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "3D Raw Data"))
    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    if vis:
        lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "3D Point Cloud"))
    lidar.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar)

    
    
    
    lidar_2d = sens.ChLidarSensor(
        car.GetChassisBody(),          
        update_rate,                   
        lidar_offset,                  
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
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, 1, "2D Raw Data"))
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    if vis:
        lidar_2d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "2D Point Cloud"))
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar_2d)

    
    
    
    camera_offset = chrono.ChFrameD(
        chrono.ChVectorD(3.0, 0, 1.5),  
        chrono.Q_from_AngAxis(-chrono.CH_C_PI_2, chrono.ChVectorD(1,0,0))  
    )
    camera = sens.ChCameraSensor(
        car.GetChassisBody(),  
        30,                    
        camera_offset,          
        90,                    
        640, 480               
    )
    camera.SetName("Third Person Camera")
    if vis:
        camera.PushFilter(sens.ChFilterVisualizeCamera())
    manager.AddSensor(camera)

    
    
    
    ch_time = 0.0
    t1 = time.time()

    while ch_time < end_time:
        
        driver.Update(ch_time)

        
        car.Synchronize(driver.GetInput())
        car.Update(ch_time)
        terrain.Synchronize(ch_time)
        terrain.Update()

        
        manager.Update()

        
        mphysicalSystem.DoStepDynamics(step_size)
        ch_time = mphysicalSystem.GetChTime()

        
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData():
            data = xyzi_buffer.GetXYZIData()
            print(f"3D Lidar Data: Max Value = {np.max(data)}")

    print(f"Sim time: {end_time} | Wall time: {time.time()-t1}")

if __name__ == "__main__":
    main()

import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh
import numpy as np
import time
import math


step_size = 1e-3
end_time = 40.0
vis = True
out_dir = "SENSOR_OUTPUT/"


noise_model = "NONE"
return_mode = sens.LidarReturnMode_STRONGEST_RETURN
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

def main():
    
    car = veh.ARTcar()
    car.SetContactMethod(chrono.ChSystemDynamics.ContactMethod.NSC)
    car.SetChassisCollisionType(veh.ChassisCollisionType_NONE)
    car.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT))
    car.Initialize()

    
    terrain = veh.RigidTerrain(car.GetSystem())
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch_mat.SetFriction(0.9)
    terrain.AddPatch(patch_mat, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0)), 100, 100)
    terrain.Initialize()

    
    driver = veh.RoboticDriver()
    car.SetDriver(driver)
    driver.Initialize(car.GetVehicle(), step_size)

    mphysicalSystem = car.GetSystem()

    
    manager = sens.ChSensorManager(mphysicalSystem)

    
    lidar_offset = chrono.ChFrameD(
        chrono.ChVectorD(1.0, 0, 1),
        chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))
    )

    
    lidar = sens.ChLidarSensor(
        car.GetChassisBody(),
        update_rate,
        lidar_offset,
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
    lidar.SetName("3D Lidar")
    lidar.SetLag(lag)
    lidar.SetCollectionWindow(collection_time)

    
    if noise_model == "CONST_NORMAL_XYZI":
        lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    if vis:
        lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "3D Raw"))
    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    if vis:
        lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "3D Point Cloud"))
    lidar.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar)

    
    lidar_2d = sens.ChLidarSensor(
        car.GetChassisBody(),
        update_rate,
        lidar_offset,
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
    lidar_2d.SetName("2D Lidar")
    lidar_2d.SetLag(lag)
    lidar_2d.SetCollectionWindow(collection_time)

    
    if noise_model == "CONST_NORMAL_XYZI":
        lidar_2d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    if vis:
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, 1, "2D Raw"))
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    if vis:
        lidar_2d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "2D Point Cloud"))
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar_2d)

    
    camera_offset = chrono.ChFrameD(
        chrono.ChVectorD(3.0, 0, 1.5),
        chrono.Q_from_AngAxis(-chrono.CH_C_PI_2, chrono.ChVectorD(1, 0, 0))
    )
    camera = sens.ChCameraSensor(
        car.GetChassisBody(),
        30,
        camera_offset,
        90,
        640,
        480
    )
    camera.SetName("Third Person Camera")
    if vis:
        camera.PushFilter(sens.ChFilterVisualizeCamera())
    manager.AddSensor(camera)

    
    ch_time = 0.0
    t1 = time.time()

    while ch_time < end_time:
        driver.Update(ch_time)
        car.Synchronize(driver.GetInput())
        car.Update(ch_time)
        terrain.Synchronize(ch_time)
        terrain.Update()
        manager.Update()
        mphysicalSystem.DoStepDynamics(step_size)
        ch_time = mphysicalSystem.GetChTime()

        
        if lidar.GetMostRecentXYZIBuffer().HasData():
            data = lidar.GetMostRecentXYZIBuffer().GetXYZIData()
            print(f"Max 3D Lidar Value: {np.max(data)}")

    print(f"Simulation Time: {end_time} | Wall Time: {time.time() - t1}")

if __name__ == "__main__":
    main()