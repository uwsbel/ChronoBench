import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh
import numpy as np
import time
import math

def main():
    
    
    
    veh.SetDataPath('path/to/chrono/data')  
    art_car = veh.ARTcar()
    art_car.SetContactMethod(chrono.ChContactMethod_NSC)
    art_car.SetChassisFixed(False)
    art_car.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 1.0)))
    art_car.Initialize()

    
    terrain = veh.RigidTerrain(art_car.GetSystem())
    terrain.SetContactFrictionCoefficient(0.9)
    terrain.SetContactRestitutionCoefficient(0.01)
    terrain.SetContactMaterial(chrono.GetMatrDataFile("materials/GT_Iso_terrain.mat"))
    patch = terrain.AddPatch(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.Q_from_AngX(-chrono.CH_PI / 2)), chrono.ChVector3d(100, 100, 1))
    patch.SetTexture(chrono.GetChronoDataFile("textures/grass.jpg"), 100, 100)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    
    driver = veh.ChDriver(art_car.GetVehicle())

    
    
    
    manager = sens.ChSensorManager(art_car.GetSystem())

    
    
    
    offset_pose = chrono.ChFrame(chrono.ChVector3d(1.0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
    update_rate = 5.0
    horizontal_samples = 800
    vertical_samples = 300
    horizontal_fov = 2 * chrono.CH_PI
    max_vert_angle = chrono.CH_PI / 12
    min_vert_angle = -chrono.CH_PI / 6
    max_range = 100.0
    noise_model = "NONE"
    sample_radius = 2
    divergence_angle = 0.003
    return_mode = sens.LidarReturnMode_STRONGEST_RETURN
    lag = 0
    collection_time = 1. / update_rate

    lidar = sens.ChLidarSensor(
        art_car.GetChassisBody(),
        update_rate,
        offset_pose,
        horizontal_samples,
        vertical_samples,
        horizontal_fov,
        max_vert_angle,
        min_vert_angle,
        max_range,
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

    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    lidar.PushFilter(sens.ChFilterXYZIAccess())

    
    vis = True
    if vis:
        lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth Data"))
        lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))

    manager.AddSensor(lidar)

    
    lidar_2d = sens.ChLidarSensor(
        art_car.GetChassisBody(),
        update_rate,
        offset_pose,
        horizontal_samples,
        1,
        horizontal_fov,
        0.0,
        0.0,
        max_range,
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

    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())

    if vis:
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, 1, "Raw 2D Lidar Depth Data"))

    manager.AddSensor(lidar_2d)

    
    camera = sens.ChCameraSensor(
        art_car.GetChassisBody(),
        10.0,
        chrono.ChFrame(chrono.ChVector3d(-10, 0, 3), chrono.QuatFromAngleAxis(chrono.CH_PI / 20, chrono.ChVector3d(0, 1, 0))),
        640,
        480,
        0
    )
    camera.SetName("Third Person Camera")
    manager.AddSensor(camera)

    
    
    
    step_size = 1e-3
    end_time = 40.0
    ch_time = 0.0
    t1 = time.time()

    while ch_time < end_time:
        
        manager.Update()

        
        driver_inputs = driver.GetInputs()

        
        art_car.Synchronize(time.time(), driver_inputs, terrain)
        art_car.Advance(step_size)

        
        terrain.Synchronize(time.time())
        terrain.Advance(step_size)

        
        art_car.GetSystem().DoStepDynamics(step_size)

        
        ch_time = art_car.GetSystem().GetChTime()

        
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            print('XYZI buffer received from lidar. Lidar resolution: {0}x{1}'.format(xyzi_buffer.Width, xyzi_buffer.Height))
            print('Max Value: {0}'.format(np.max(xyzi_data)))

        print("Sim time:", ch_time, "Wall time:", time.time() - t1)

if __name__ == "__main__":
    main()