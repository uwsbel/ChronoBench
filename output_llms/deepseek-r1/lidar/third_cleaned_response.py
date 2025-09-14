import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import numpy as np
import time
import math

def main():
    
    
    
    noise_model = "NONE"  
    return_mode = sens.LidarReturnMode_STRONGEST_RETURN
    update_rate = 5.0                 
    horizontal_samples = 800          
    vertical_samples = 300            
    horizontal_fov = 2 * chrono.CH_PI 
    max_vert_angle = chrono.CH_PI / 12
    min_vert_angle = -chrono.CH_PI / 6
    lag = 0.0                         
    collection_time = 1.0 / update_rate
    sample_radius = 2                 
    divergence_angle = 0.003          
    lidar_range = 100.0               

    
    
    
    step_size = 1e-3
    end_time = 40.0
    vis = True                        
    out_dir = "SENSOR_OUTPUT/"

    
    
    
    vehicle = veh.ARTcar()
    vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
    vehicle.SetChassisFixed(False)
    vehicle.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.5), chrono.QUNIT))
    vehicle.Initialize()
    vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    
    mphysicalSystem = vehicle.GetSystem()

    
    
    
    terrain = veh.RigidTerrain(mphysicalSystem)
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 1), 100, 100)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    terrain.Initialize()

    
    
    
    driver = veh.ChDriver(vehicle.GetVehicle())
    driver.Initialize()

    
    
    
    manager = sens.ChSensorManager(mphysicalSystem)
    manager.scene.AddPointLight(chrono.ChVector3d(2, 2.5, 100), chrono.ChColor(1, 1, 1), 5000)

    
    
    
    chassis = vehicle.GetChassisBody()

    
    
    
    lidar_offset_pose = chrono.ChFramed(
        chrono.ChVector3d(1.0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )

    
    lidar = sens.ChLidarSensor(
        chassis,              
        update_rate,
        lidar_offset_pose,
        horizontal_samples,
        vertical_samples,
        horizontal_fov,
        max_vert_angle,
        min_vert_angle,
        lidar_range,
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
    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    lidar.PushFilter(sens.ChFilterXYZIAccess())
    if vis:
        lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "3D Lidar Depth"))
        lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "3D Lidar Points"))
    manager.AddSensor(lidar)

    
    lidar_2d = sens.ChLidarSensor(
        chassis,
        update_rate,
        lidar_offset_pose,
        horizontal_samples,
        1,  
        horizontal_fov,
        0.0,  
        0.0,
        lidar_range,
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
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
    if vis:
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, 1, "2D Lidar Depth"))
    manager.AddSensor(lidar_2d)

    
    
    
    camera_offset = chrono.ChFramed(
        chrono.ChVector3d(-3, 0, 2), 
        chrono.QuatFromAngleAxis(0.3, chrono.ChVector3d(0, 1, 0))
    )
    camera = sens.ChCameraSensor(
        chassis,  
        30,       
        camera_offset,
        1280,     
        720,
        chrono.CH_PI/3  
    )
    camera.SetName("Third Person View")
    camera.PushFilter(sens.ChFilterVisualize(1280, 720, "Third Person Camera"))
    manager.AddSensor(camera)

    
    
    
    ch_time = 0.0
    t1 = time.time()

    while ch_time < end_time:
        
        driver_inputs = driver.GetInputs()
        driver.Synchronize(ch_time)
        
        
        vehicle.Synchronize(ch_time, driver_inputs)
        terrain.Synchronize(ch_time)
        manager.Update()

        
        mphysicalSystem.DoStepDynamics(step_size)
        ch_time = mphysicalSystem.GetChTime()

    print("Sim time:", end_time, "Wall time:", time.time() - t1)

if __name__ == "__main__":
    main()