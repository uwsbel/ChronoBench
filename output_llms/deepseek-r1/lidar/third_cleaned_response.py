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
    lag = 0
    collection_time = 1.0 / update_rate
    sample_radius = 2
    divergence_angle = 0.003
    step_size = 1e-3
    end_time = 40.0
    vis = True
    out_dir = "SENSOR_OUTPUT/"

    
    
    
    mphysicalSystem = chrono.ChSystemNSC()

    
    
    
    
    vehicle = veh.ARTcar()
    vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
    vehicle.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.5), chrono.ChVector3d(1, 0, 0)))
    vehicle.Initialize()

    
    vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)

    
    vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    mphysicalSystem = vehicle.GetSystem()

    
    
    
    driver = veh.ChDriver(vehicle.GetVehicle())
    driver.Initialize()

    
    
    
    terrain = veh.RigidTerrain(mphysicalSystem)
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), 200, 200)
    patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    
    
    
    manager = sens.ChSensorManager(mphysicalSystem)
    manager.scene.AddPointLight(chrono.ChVector3d(100, 100, 100), chrono.ChColor(1, 1, 1), 5000)

    
    
    
    
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(1.0, 0, 1),  
        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
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
        
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, 1, "Raw 2D Lidar Depth Data"))
    
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar_2d)
    
    
    
    
    camera_offset = chrono.ChFramed(
        chrono.ChVector3d(-8, 0, 3),  
        chrono.Q_from_AngAxis(0.3, chrono.ChVector3d(0, 1, 0))  
    )
    camera = sens.ChCameraSensor(
        vehicle.GetChassisBody(),  
        30,                        
        camera_offset,             
        1280,                      
        720,                       
        chrono.CH_PI / 3           
    )
    camera.SetName("Third Person Camera")
    camera.SetLag(0.1)
    camera.SetCollectionWindow(0.02)
    
    if vis:
        camera.PushFilter(sens.ChFilterVisualize(1280, 720, "Third Person View"))
    camera.PushFilter(sens.ChFilterRGBA8Access())
    manager.AddSensor(camera)

    
    
    
    ch_time = 0.0
    render_time = 0
    t1 = time.time()

    while ch_time < end_time:
        
        driver.Synchronize(ch_time)
        vehicle.Synchronize(ch_time, driver.GetInputs(), terrain)
        terrain.Synchronize(ch_time)
        
        
        manager.Update()
        
        
        driver.Advance(step_size)
        vehicle.Advance(step_size)
        terrain.Advance(step_size)
        
        
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            print(f'Lidar point count: {len(xyzi_data)} | Max Z: {np.max(xyzi_data[:,2]):.2f}')

        
        ch_time = mphysicalSystem.GetChTime()

    print("Simulation time:", end_time, "seconds")
    print("Wall time:", time.time() - t1, "seconds")

if __name__ == "__main__":
    main()