import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

import numpy as np
import time
import math

def main():
    
    
    
    
    vehicleSystem = veh.ChVehicleSystem()
    vehicleSystem.SetChTime(0)
    vehicleSystem.SetStepSize(step_size)

    
    
    
    
    artcar = veh.ARTcar(vehicleSystem)
    artcar.SetContactMethod(chrono.ChContactMethod_NSC)
    artcar.SetChassisFixed(False)
    artcar.SetInitPosition(chrono.ChCoorSysd(chrono.ChVector3d(0, 0, 0.5), chrono.ChQuaterniond(1, 0, 0, 0)))
    artcar.SetInitFwdVel(10)
    artcar.Initialize()

    
    
    
    driver = veh.ChDataDriver()
    driver.SetSteering(0)
    driver.SetThrottle(0.3)
    driver.SetBraking(0)
    artcar.SetDriver(driver)

    
    
    
    terrain = veh.Terrain(vehicleSystem)
    terrain.SetContactMaterialProperties(2e6, 0.8)
    terrain.SetContactFriction(0.9)
    terrain.SetContactRestitution(0.1)
    terrain.SetTexture(chrono.GetChronoDataFile("textures/terrain.jpg"))
    terrain.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize(0, 0, 100, 100)

    
    
    
    manager = sens.ChSensorManager(vehicleSystem)

    
    
    
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(1.0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
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

    
    
    
    camera = sens.ChCameraSensor(
        artcar.GetChassis(),  
        update_rate,          
        chrono.ChFramed(chrono.ChVector3d(0, 0, 1.75), chrono.QuatFromAngleAxis(chrono.CH_PI, chrono.ChVector3d(0, 1, 0))),
        1280,                 
        720,                  
        chrono.CH_PI / 3,     
        chrono.CH_PI / 3      
    )
    camera.SetName("Third Person Camera")
    camera.SetLag(lag)
    camera.SetCollectionWindow(collection_time)
    if vis:
        camera.PushFilter(sens.ChFilterVisualize(1280, 720, "Third Person Camera"))
    camera.PushFilter(sens.ChFilterRGBA8Access())
    manager.AddSensor(camera)

    
    
    
    ch_time = 0.0
    render_time = 0
    t1 = time.time()

    while ch_time < end_time:
        
        ch_time = vehicleSystem.GetChTime()

        
        driver.Synchronize(ch_time)
        artcar.Synchronize(ch_time, driver.GetSteering(), driver.GetThrottle(), driver.GetBraking())

        
        terrain.Synchronize(ch_time)

        
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            print('XYZI buffer received from lidar. Lidar resolution: {0}x{1}'.format(xyzi_buffer.Width, xyzi_buffer.Height))
            print('Max Value: {0}'.format(np.max(xyzi_data)))

        
        manager.Update()

        
        vehicleSystem.DoStepDynamics(step_size)

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