import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import numpy as np
import time
import math

def main():
    
    veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')
    my_ARTcar = veh.ARTcar()
    my_ARTcar.SetContactMethod(chrono.ChContactMethod_NSC)
    my_ARTcar.SetChassisFixed(False) 
    my_ARTcar.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 1.0)))
    my_ARTcar.SetPowertrainType(veh.PowertrainModelType_SIMPLE)
    my_ARTcar.SetDriveType(veh.DrivelineTypeWV_AWD)
    my_ARTcar.SetTireType(veh.TireModelType_TMEASY)
    my_ARTcar.Initialize()

    
    driver = veh.ChDriver(my_ARTcar.GetVehicle())

    
    terrain = veh.RigidTerrain(my_ARTcar.GetSystem())
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    size = (100, 100)
    box = terrain.AddPatch(patch_mat, chrono.CSYSNORM, size[0], size[1])
    box.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), size[0], size[1])
    box.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    
    manager = sens.ChSensorManager(my_ARTcar.GetSystem())

    
    offset_pose = chrono.ChFrameD(
        chrono.ChVector3d(1.0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
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

    lidar = sens.ChLidarSensor(
        my_ARTcar.GetChassisBody(), 
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
    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    lidar.PushFilter(sens.ChFilterXYZIAccess())
    vis = True
    if vis:
        lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth Data"))
        lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
    manager.AddSensor(lidar)

    
    lidar_2d = sens.ChLidarSensor(
        my_ARTcar.GetChassisBody(), 
        update_rate, 
        offset_pose, 
        horizontal_samples, 
        1, 
        horizontal_fov, 
        0, 
        0, 
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
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
    if vis:
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, 1, "Raw 2D Lidar Depth Data"))
    manager.AddSensor(lidar_2d)

    
    camera = sens.ChCameraSensor(
        my_ARTcar.GetChassisBody(),
        update_rate,
        offset_pose,
        1280,
        720,
        chrono.CH_PI / 2
    )
    camera.SetName("Third Person Camera")
    camera.SetLag(lag)
    camera.SetCollectionWindow(collection_time)
    if vis:
        camera.PushFilter(sens.ChFilterVisualize(1280, 720, "Third Person Camera"))
    manager.AddSensor(camera)

    
    end_time = 40.0
    step_size = 1e-3
    ch_time = 0.0
    t1 = time.time()
    while ch_time < end_time:
        
        driver_inputs = driver.GetInputs()

        
        terrain.Synchronize(ch_time)
        my_ARTcar.Synchronize(ch_time, driver_inputs, terrain)
        manager.Synchronize(ch_time)

        
        terrain.Advance(step_size)
        my_ARTcar.Advance(step_size)
        manager.Update()
        my_ARTcar.GetSystem().DoStepDynamics(step_size)

        
        ch_time = my_ARTcar.GetSystem().GetChTime()

        
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            print('XYZI buffer received from lidar. Lidar resolution: {0}x{1}'.format(xyzi_buffer.Width, xyzi_buffer.Height))
            print('Max Value: {0}'.format(np.max(xyzi_data)))

    print("Sim time:", end_time, "Wall time:", time.time() - t1)

if __name__ == "__main__":
    main()