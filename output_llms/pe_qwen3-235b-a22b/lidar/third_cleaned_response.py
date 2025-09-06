import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh

import numpy as np
import time
import math


def main():
    
    
    
    
    car = veh.ARTcar()
    car.SetContactMethod(contact_method)
    car.SetChassisCollisionType(chassis_collision_type)
    car.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
    car.Initialize()

    
    terrain = veh.RigidTerrain(car.GetSystem())
    patch_mat = veh.MaterialSurfaceNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(), 100, 100)
    patch.SetTexture(chrono.GetChronoDataFile("terrain/textures/tile4.jpg"), 100, 100)
    terrain.Initialize()

    
    driver = veh.ChDriver(car.GetChassisBody())
    driver.Initialize()

    
    
    
    manager = sens.ChSensorManager(car.GetSystem())

    
    
    
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(1.0, 0, 1),  
        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    lidar = sens.ChLidarSensor(
        car.GetChassisBody(),           
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
        car.GetChassisBody(),           
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
    
    
    camera_offset = chrono.ChFramed(
        chrono.ChVector3d(-3, 0, 2),  
        chrono.QuatFromAngleAxis(math.pi, chrono.ChVector3d(0, 1, 0))  
    )
    camera = sens.ChCameraSensor(
        car.GetChassisBody(),
        60,  
        camera_offset,
        1280, 720,  
        chrono.CH_PI / 4  
    )
    camera.SetName("Third Person Camera")
    if vis:
        camera.PushFilter(sens.ChFilterVisualize(1280, 720, "Third Person View"))
    manager.AddSensor(camera)
    
    
    
    
    ch_time = 0.0
    render_time = 0
    t1 = time.time()

    while ch_time < end_time:
        
        current_time = ch_time

        
        driver_inputs = driver.GetInputs()
        car.Synchronize(current_time, driver_inputs, terrain)
        terrain.Synchronize(current_time)
        driver.Synchronize(current_time)
        
        
        manager.Update()

        
        car.Advance(step_size)
        terrain.Advance(step_size)
        driver.Advance(step_size)

        
        car.GetSystem().DoStepDynamics(step_size)

        
        ch_time = car.GetSystem().GetChTime()

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


contact_method = chrono.ChContactMethod_NSC
chassis_collision_type = veh.ChassisCollisionType_NONE
initLoc = chrono.ChVector3d(0, 0, 0)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "data"))


main()