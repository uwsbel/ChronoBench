import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens

import numpy as np
import time
import math

def main():
    
    
    
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    veh.SetDataPath(chrono.GetChronoDataFile("vehicle/"))

    
    init_pos = chrono.ChCoordsysd(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT)
    vehicle = veh.Artcar()
    vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
    vehicle.SetChassisFixed(False)
    vehicle.SetInitPosition(init_pos)
    vehicle.SetInitFwdVel(0.0)
    vehicle.Initialize()

    
    mphysicalSystem = vehicle.GetSystem()

    
    
    
    terrain = veh.RigidTerrain(mphysicalSystem)
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(
        patch_mat,
        chrono.ChVectorD(0, 0, 0),
        chrono.ChVectorD(0, 0, 1),
        200, 200
    )
    patch.SetTexture(chrono.GetChronoDataFile("terrain/textures/grass.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    
    
    
    driver = veh.ChDriver(vehicle)
    driver.Initialize()

    
    
    
    manager = sens.ChSensorManager(mphysicalSystem)

    
    manager.scene.AddPointLight(chrono.ChVectorF(100, 100, 100), chrono.ChColor(1, 1, 1), 1000)
    manager.scene.AddPointLight(chrono.ChVectorF(-100, -100, 100), chrono.ChColor(1, 1, 1), 1000)
    manager.scene.SetAmbientLight(chrono.ChColor(0.2, 0.2, 0.2))
    manager.scene.SetBackground(chrono.ChColor(0.8, 0.8, 1.0))

    
    
    
    chassis = vehicle.GetChassisBody()

    
    offset_pose = chrono.ChFrameD(
        chrono.ChVectorD(1.0, 0, 1),
        chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))
    )

    
    lidar = sens.ChLidarSensor(
        chassis,              
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
    if vis:
        lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth Data"))
    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    if vis:
        lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
    lidar.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar)

    
    lidar_2d = sens.ChLidarSensor(
        chassis,
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
    if vis:
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, 1, "Raw 2D Lidar Depth Data"))
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar_2d)

    
    cam_offset_pose = chrono.ChFrameD(
        chrono.ChVectorD(-8, 0, 3),
        chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))
    )
    camera = sens.ChCameraSensor(
        chassis,                 
        update_rate,             
        cam_offset_pose,         
        1280,                    
        720,                     
        chrono.CH_C_PI / 3       
    )
    camera.SetName("Third Person Camera")
    camera.SetLag(lag)
    camera.SetCollectionWindow(collection_time)
    if vis:
        camera.PushFilter(sens.ChFilterVisualize(1280, 720, "Camera View"))
    camera.PushFilter(sens.ChFilterRGBA8Access())
    manager.AddSensor(camera)

    
    
    
    ch_time = 0.0
    t1 = time.time()

    while ch_time < end_time:
        
        driver_inputs = driver.GetInputs()

        
        driver.Synchronize(ch_time)
        vehicle.Synchronize(ch_time, driver_inputs, terrain)
        terrain.Synchronize(ch_time)

        
        driver.Advance(step_size)
        vehicle.Advance(step_size)
        terrain.Advance(step_size)

        
        manager.Update()

        
        mphysicalSystem.DoStepDynamics(step_size)

        
        ch_time = mphysicalSystem.GetChTime()

        
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer and xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            print('XYZI buffer received from lidar. Lidar resolution: {0}x{1}'.format(xyzi_buffer.Width, xyzi_buffer.Height))
            print('Max Value: {0}'.format(np.max(xyzi_data)))

    print("Sim time:", end_time, "Wall time:", time.time() - t1)







noise_model = "NONE"  


return_mode = sens.LidarReturnMode_STRONGEST_RETURN


update_rate = 5.0


horizontal_samples = 800
vertical_samples = 300


horizontal_fov = 2 * chrono.CH_C_PI  
max_vert_angle = chrono.CH_C_PI / 12
min_vert_angle = -chrono.CH_C_PI / 6


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