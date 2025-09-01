import pychrono.core as chrono
import pychrono.sensor as sens

import numpy as np
import time
import math



mphysicalSystem = chrono.ChSystemNSC()


car_mass = 1000.0  
car_length = 4.0  
car_width = 1.5  
car_height = 1.0  
car_center_of_mass = chrono.ChVector3d(0.0, 0.5, 0.0)  
car_wheel_base = 2.0  


terrain_material = chrono.ChMaterial("terrain", 
                                     density=1000.0, 
                                     friction=0.5, 
                                     restitution=0.0)
terrain_texture = chrono.GetChronoDataFile("textures/terrain.png")
terrain = chrono.ChRigidBody(chrono.ChVector3d(0, 0, 0), terrain_material, terrain_texture)
terrain.SetPos(chrono.ChVector3d(0, 0, 0))
terrain.SetFixed(True)



lidar_3d = sens.ChLidarSensor(
    terrain,  
    update_rate,            
    chrono.ChFramed(
        chrono.ChVector3d(1.0, 0, 1),
        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    ),
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
lidar_3d.SetName("3D Lidar Sensor")
lidar_3d.SetLag(lag)
lidar_3d.SetCollectionWindow(collection_time)


lidar_2d = sens.ChLidarSensor(
    terrain,  
    update_rate,            
    chrono.ChFramed(
        chrono.ChVector3d(1.0, 0, 1),
        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    ),
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


driver = chrono.ChVehicleDriver(mphysicalSystem, car_mass, car_length, car_width, car_height, car_center_of_mass, car_wheel_base)
driver.SetSpeed(10.0)  


camera = sens.ChCameraSensor(
    mphysicalSystem,
    chrono.ChVector3d(0, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
)
camera.SetName("Third Person Camera")
camera.SetLag(lag)
camera.SetCollectionWindow(collection_time)


t1 = time.time()
ch_time = 0.0
while ch_time < end_time:
    
    driver.Update()
    mphysicalSystem.DoStepDynamics(step_size)

    
    manager.Update()

    
    camera.Update()

    
    lidar_3d.Update()
    lidar_2d.Update()

    
    xyzi_buffer = lidar_3d.GetMostRecentXYZIBuffer()
    if xyzi_buffer.HasData():
        xyzi_data = xyzi_buffer.GetXYZIData()
        print('XYZI buffer received from lidar. Lidar resolution: {0}x{1}'.format(xyzi_buffer.Width, xyzi_buffer.Height))
        print('Max Value: {0}'.format(np.max(xyzi_data)))

    
    if vis:
        camera.Render()

    
    mphysicalSystem.DoStepDynamics(step_size)

    
    ch_time = mphysicalSystem.GetChTime()

    
    print("Sim time:", end_time, "Wall time:", time.time() - t1)