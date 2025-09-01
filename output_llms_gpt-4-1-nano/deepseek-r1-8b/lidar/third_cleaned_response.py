import pychrono.core as chrono
import pychrono.sensor as sens

import numpy as np
import time
import math

def main():
    
    
    
    mphysicalSystem = chrono.ChSystemNSC()
    
    
    artcar = chrono.ChVehicle()
    artcar.SetName("ARTcar")
    
    
    driver = chrono.ChVehicleDriver()
    artcar.SetDriver(driver)
    
    
    
    
    terrain_size = 50
    terrain = chrono.ChBodyEasyBox(terrain_size, terrain_size, terrain_size, 1)
    terrain.SetPos(chrono.ChVector3d(0, -5, -1))
    terrain.GetVisualModel().GetShape(0).SetTexture(chrono.GetChronoDataFile("textures/ground.png"))
    terrain.SetMaterialProperties(chrono.ChMaterialProperties(
        young_modulus=1000000000,
        poisson_ratio=0.2,
        friction=1.0
    ))
    mphysicalSystem.Add(terrain)
    
    
    
    
    
    
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
    lidar.SetName("3D Lidar Sensor")
    lidar.SetLag(lag)
    lidar.SetCollectionWindow(collection_time)
    
    
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
        lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
        lidar_2d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    elif noise_model == "NONE":
        pass
    
    
    third_person_camera = sens.ChSensor(
        artcar.GetChassis(),  
        chrono.ChVector3d(5, 5, 5),  
        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(1, 0, 0)),  
        sens.SensorType_Camera_Perspective,
        45,  
        1024,  
        768,  
        1000.0  
    )
    third_person_camera.SetName("Third Person Camera")
    mphysicalSystem.Add(third_person_camera)
    
    
    if vis:
        lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw 3D Lidar Depth Data"))
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, 1, "Raw 2D Lidar Depth Data"))
        third_person_camera.PushFilter(sens.ChFilterVisualizeCamera("Third Person View"))
    
    
    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    lidar.PushFilter(sens.ChFilterXYZIAccess())
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
    
    
    manager.AddSensor(lidar)
    manager.AddSensor(lidar_2d)
    manager.AddSensor(third_person_camera)
    
    
    
    
    orbit_radius = 10
    orbit_rate = 0.1
    ch_time = 0.0
    end_time = 40.0
    
    render_time = 0
    t1 = time.time()
    
    while ch_time < end_time:
        
        artcar.Update()
        driver.Update()
        terrain.UpdateDynamics()
        terrain.UpdateGeometry()  
        
        
        lidar.SetOffsetPose(
            chrono.ChFramed(
                chrono.ChVector3d(
                    -orbit_radius * math.cos(ch_time * orbit_rate),
                    -orbit_radius * math.sin(ch_time * orbit_rate),
                    1
                ),
                chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVector3d(0, 0, 1))
            )
        )
        
        
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            print('XYZI buffer received from lidar. Lidar resolution: {0}x{1}'.format(xyzi_buffer.Width, xyzi_buffer.Height))
            print('Max Value: {0}'.format(np.max(xyzi_data)))
        
        
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