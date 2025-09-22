import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

import numpy as np
import time
import math

def main():
    
    
    
    
    vehicle_system = veh.ChVehicleSystem()
    vehicle_system.SetChTime(0)

    
    artcar = veh.ARTcar(vehicle_system)
    artcar.SetInitPosition(chrono.ChCoordsys(chrono.ChVector3d(0, 0, 0.5)))
    artcar.SetInitFwdVel(10)
    artcar.Initialize()

    
    driver = veh.ChDataDriver(artcar.GetVehicle())
    driver.SetSteering(0)
    driver.SetThrottle(0.5)
    driver.SetBraking(0)

    
    
    
    manager = sens.ChSensorManager(vehicle_system)

    
    
    
    terrain = veh.RigidTerrain(vehicle_system)
    terrain.SetContactMaterialProperties(2e6, 0.8)
    terrain.SetTexture(chrono.GetChronoDataFile("terrain/textures/tile4.jpg"), 200, 200)
    terrain.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize(0, 0, 0)

    
    
    
    side = 4
    box = chrono.ChBodyEasyBox(side, side, side, 1000)
    box.SetPos(chrono.ChVector3d(0, 0, 0))
    box.GetVisualModel().GetShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
    box.SetFixed(True)
    vehicle_system.Add(box)

    
    
    
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(1.0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    lidar = sens.ChLidarSensor(
        artcar.GetChassisBody(),  
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
        artcar.GetChassisBody(),  
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
        artcar.GetChassisBody(),  
        update_rate,              
        chrono.ChFramed(chrono.ChVector3d(0, 0, 2), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))),  
        640,                      
        480,                      
        chrono.CH_PI / 4,         
        100.0                     
    )
    camera.SetName("Third Person Camera")
    camera.SetLag(lag)
    camera.SetCollectionWindow(collection_time)
    if vis:
        camera.PushFilter(sens.ChFilterVisualize(640, 480, "Third Person Camera"))
    camera.PushFilter(sens.ChFilterRGBA8Access())
    manager.AddSensor(camera)

    
    
    
    ch_time = 0.0

    render_time = 0
    t1 = time.time()

    while ch_time < end_time:
        
        ch_time = vehicle_system.GetChTime()

        
        driver.Synchronize(ch_time)
        driver.Advance(step_size)

        
        artcar.Synchronize(ch_time, driver.GetSteering(), driver.GetThrottle(), driver.GetBraking())
        artcar.Advance(step_size)

        
        terrain.Synchronize(ch_time)
        terrain.Advance(step_size)

        
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            print('XYZI buffer received from lidar. Lidar resolution: {0}x{1}'.format(xyzi_buffer.Width, xyzi_buffer.Height))
            print('Max Value: {0}'.format(np.max(xyzi_data)))

        
        manager.Update()

        
        vehicle_system.DoStepDynamics(step_size)

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