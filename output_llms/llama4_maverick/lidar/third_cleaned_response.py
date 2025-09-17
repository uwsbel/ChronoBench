import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh
import numpy as np
import time
import math

def main():
    
    
    
    veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')
    my_ARTcar = veh.ARTcar()
    my_ARTcar.SetContactMethod(chrono.ChContactMethod_NSC)
    my_ARTcar.SetChassisFixed(False)
    my_ARTcar.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 1.0)))
    my_ARTcar.Initialize()

    
    
    
    manager = sens.ChSensorManager(my_ARTcar.GetSystem())

    
    
    
    
    noise_model = "NONE"
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
    return_mode = sens.LidarReturnMode_STRONGEST_RETURN

    offset_pose = chrono.ChFrame(chrono.ChVector3d(1.0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
    lidar = sens.ChLidarSensor(my_ARTcar.GetChassisBody(), update_rate, offset_pose, horizontal_samples, vertical_samples, horizontal_fov, max_vert_angle, min_vert_angle, 100.0, sens.LidarBeamShape_RECTANGULAR, sample_radius, divergence_angle, divergence_angle, return_mode)
    lidar.SetName("Lidar Sensor")
    lidar.SetLag(lag)
    lidar.SetCollectionWindow(collection_time)

    if noise_model == "CONST_NORMAL_XYZI":
        lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    elif noise_model == "NONE":
        pass

    lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth Data"))
    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
    lidar.PushFilter(sens.ChFilterXYZIAccess())

    manager.AddSensor(lidar)

    
    lidar_2d = sens.ChLidarSensor(my_ARTcar.GetChassisBody(), update_rate, offset_pose, horizontal_samples, 1, horizontal_fov, 0.0, 0.0, 100.0, sens.LidarBeamShape_RECTANGULAR, sample_radius, divergence_angle, divergence_angle, return_mode)
    lidar_2d.SetName("2D Lidar Sensor")
    lidar_2d.SetLag(lag)
    lidar_2d.SetCollectionWindow(collection_time)

    if noise_model == "CONST_NORMAL_XYZI":
        lidar_2d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    elif noise_model == "NONE":
        pass

    lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, 1, "Raw 2D Lidar Depth Data"))
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())

    manager.AddSensor(lidar_2d)

    
    offset_pose_cam = chrono.ChFrame(chrono.ChVector3d(-5, 0, 2), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
    cam = sens.ChCameraSensor(my_ARTcar.GetChassisBody(), 30, offset_pose_cam, 1280, 720)
    cam.SetName("Third Person Camera")
    cam.PushFilter(sens.ChFilterVisualize(1280, 720, "Third Person Camera"))
    manager.AddSensor(cam)

    
    terrain_material = chrono.ChMaterialSurfaceNSC()
    terrain_material.SetFriction(0.9)
    terrain = veh.RigidTerrain(my_ARTcar.GetSystem())
    terrain.SetContactMaterial(terrain_material)
    patch = terrain.AddPatch(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), chrono.ChVector3d(100, 100, 1))
    patch.SetTexture(chrono.GetChronoDataFile("textures/grass.jpg"), 100, 100)
    patch.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
    terrain.Initialize()

    
    driver = veh.ChDriver(my_ARTcar.GetVehicle())

    
    
    
    step_size = 1e-3
    end_time = 40.0

    my_ARTcar.GetSystem().SetStep(step_size)

    t1 = time.time()
    ch_time = 0.0
    while ch_time < end_time:
        
        manager.Update()

        
        ch_time = my_ARTcar.GetSystem().GetChTime()

        
        driver_inputs = driver.GetInputs()
        my_ARTcar.Synchronize(ch_time, driver_inputs, terrain)
        terrain.Synchronize(ch_time)

        
        my_ARTcar.Advance(step_size)
        my_ARTcar.GetSystem().DoStepDynamics(step_size)

        print("Sim time:", ch_time, "Wall time:", time.time() - t1)

if __name__ == "__main__":
    main()