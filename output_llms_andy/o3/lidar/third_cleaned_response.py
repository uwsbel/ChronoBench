import math
import time
import numpy as np

import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh










noise_model        = "NONE"                                 
return_mode        = sens.LidarReturnMode_STRONGEST_RETURN  
update_rate        = 5.0                                    
horizontal_samples = 800
vertical_samples   = 300                                    
horizontal_fov     = 2 * chrono.CH_PI                       
max_vert_angle     =  chrono.CH_PI / 12
min_vert_angle     = -chrono.CH_PI / 6
lag                = 0.0                                    
collection_time    = 1.0 / update_rate
sample_radius      = 2                                      
divergence_angle   = 0.003                                  


step_size  = 1e-3                                           
end_time   = 40.0                                           
vis        = True                                           
out_dir    = "SENSOR_OUTPUT/"                               





def main():
    
    
    
    contact_method = chrono.ChContactMethod_NSC

    
    
    
    
    
    
    vehicle_json     = chrono.GetChronoDataFile("sensor/ARTcar/ARTcar_Vehicle.json")
    powertrain_json  = chrono.GetChronoDataFile("sensor/ARTcar/ARTcar_SimplePowertrain.json")
    tire_json        = chrono.GetChronoDataFile("sensor/ARTcar/ARTcar_RigidTire.json")

    vehicle = veh.ArticulatedVehicle(vehicle_json,
                                     powertrain_json,
                                     tire_json,
                                     contact_method)

    init_pos = chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0),
                                  chrono.QUNIT)
    vehicle.Initialize(init_pos, 0.0)             

    
    sys = vehicle.GetSystem()

    
    
    
    terrain = veh.RigidTerrain(sys)
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)

    patch_size = 200.0
    patch = terrain.AddPatch(patch_mat,
                             chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
                             patch_size, patch_size)

    patch.SetTexture(chrono.GetChronoDataFile("textures/grass.jpg"),
                     patch_size, patch_size)
    patch.SetColor(chrono.ChColor(0.2, 0.5, 0.3))
    terrain.Initialize()

    
    
    
    driver = veh.ChDriver(vehicle)      
    driver.Initialize()

    
    
    
    manager = sens.ChSensorManager(sys)

    
    manager.scene.AddPointLight(chrono.ChVectorF(10, 10, 10), chrono.ChColor(1, 1, 1), 500.0)

    
    
    
    chassis = vehicle.GetChassisBody()

    
    lidar_pose = chrono.ChFrameD(
        chrono.ChVectorD(1.0, 0.0, 1.0),                 
        chrono.Q_from_AngAxis(0.0, chrono.ChVectorD(0, 1, 0))
    )

    
    lidar = sens.ChLidarSensor(
        parent          = chassis,
        update_rate     = update_rate,
        offset_pose     = lidar_pose,
        h_samples       = horizontal_samples,
        v_samples       = vertical_samples,
        h_fov           = horizontal_fov,
        v_fov_upper     = max_vert_angle,
        v_fov_lower     = min_vert_angle,
        max_distance    = 100.0,
        beam_shape      = sens.LidarBeamShape_RECTANGULAR,
        sample_radius   = sample_radius,
        divergence      = divergence_angle,
        gain            = divergence_angle,
        return_mode     = return_mode
    )
    lidar.SetName("3-D Lidar")
    lidar.SetLag(lag)
    lidar.SetCollectionWindow(collection_time)

    
    if noise_model == "CONST_NORMAL_XYZI":
        lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))

    if vis:
        lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples,
                                                "Raw 3-D Lidar Depth"))

    lidar.PushFilter(sens.ChFilterDIAccess())       
    lidar.PushFilter(sens.ChFilterPCfromDepth())    
    if vis:
        lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0,
                                                          "3-D Lidar Point Cloud"))

    xyzi_access = sens.ChFilterXYZIAccess()
    lidar.PushFilter(xyzi_access)

    manager.AddSensor(lidar)

    
    lidar2d = sens.ChLidarSensor(
        parent          = chassis,
        update_rate     = update_rate,
        offset_pose     = lidar_pose,
        h_samples       = horizontal_samples,
        v_samples       = 1,                        
        h_fov           = horizontal_fov,
        v_fov_upper     = 0.0,
        v_fov_lower     = 0.0,
        max_distance    = 100.0,
        beam_shape      = sens.LidarBeamShape_RECTANGULAR,
        sample_radius   = sample_radius,
        divergence      = divergence_angle,
        gain            = divergence_angle,
        return_mode     = return_mode
    )
    lidar2d.SetName("2-D Lidar")
    lidar2d.SetLag(lag)
    lidar2d.SetCollectionWindow(collection_time)

    if noise_model == "CONST_NORMAL_XYZI":
        lidar2d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))

    if vis:
        lidar2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, 1,
                                                  "Raw 2-D Lidar Depth"))

    lidar2d.PushFilter(sens.ChFilterDIAccess())
    lidar2d.PushFilter(sens.ChFilterPCfromDepth())
    lidar2d.PushFilter(sens.ChFilterXYZIAccess())

    manager.AddSensor(lidar2d)

    
    cam_pose = chrono.ChFrameD(chrono.ChVectorD(-10, 0, 3),   
                               chrono.QUNIT)

    camera = sens.ChCameraSensor(
        parent       = chassis,
        update_rate  = update_rate,
        offset_pose  = cam_pose,
        width        = 1280,
        height       = 720,
        fov          = chrono.CH_C_PI / 4
    )
    camera.SetName("Chase Camera")
    camera.PushFilter(sens.ChFilterRGBA8Access())
    manager.AddSensor(camera)

    
    
    
    print("\n*** Simulation start ***\n")
    t_start_wall = time.time()

    while sys.GetChTime() < end_time:

        sim_time = sys.GetChTime()

        
        driver.Synchronize(sim_time)

        
        driver_inputs = veh.DriverInputs()     
        vehicle.Synchronize(sim_time, driver_inputs, terrain)
        terrain.Synchronize(sim_time)

        
        driver.Advance(step_size)
        vehicle.Advance(step_size)

        
        manager.Update()

        
        sys.DoStepDynamics(step_size)

        
        xyzi = xyzi_access.GetMostRecentBuffer()
        if xyzi and xyzi.HasData():
            data = xyzi.GetXYZIData()
            print("t = {:7.3f}  |  3-D lidar cloud   {}x{}   max-val {:> .3f}"
                  .format(sim_time,
                          xyzi.GetWidth(), xyzi.GetHeight(),
                          np.max(data)))

    wall_time = time.time() - t_start_wall
    print("\n*** Simulation completed ***")
    print("Simulated time: {:.2f} s | wall-clock time: {:.2f} s".format(end_time, wall_time))





if __name__ == "__main__":
    main()