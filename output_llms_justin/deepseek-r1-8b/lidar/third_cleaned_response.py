import pychrono.core as chrono
import pychrono.sensor as sens

import numpy as np
import time
import math

def main():
    
    
    vehicle = chrono.ChVehicleART()
    vehicle.SetName("ARTcar")
    vehicle.SetMass(1000)  
    vehicle.SetInertia(chrono.ChVector3d(0, 0, 100))  
    vehicle.SetWidth(0.5)  
    vehicle.SetHeight(1.0)  
    vehicle.SetHeightOffset(1.0)  
    vehicle.SetWheelbase(2.0)  
    vehicle.SetSuspension(1.0)  
    vehicle.SetDamping(0.5, 0.5)  
    vehicle.SetFriction(0.1)  
    vehicle.SetMaxTorque(100)  
    vehicle.SetMaxSpeed(10.0)  
    vehicle.SetSteeringRatio(0.5)  
    vehicle.SetDriver(driver=chrono.ChDriverDefault())  
    mphysicalSystem = vehicle  

    
    terrain = chrono.ChTerrain()
    terrain.SetMaterial(chrono.ChMaterial(chrono.ChMaterialType.Terrain, 1.0, 1.0, 1.0))
    terrain.SetTexture(chrono.GetChronoDataFile("textures/ground.png"))
    terrain.SetPos(chrono.ChVector3d(0, 0, 0))
    mphysicalSystem.Add(terrain)

    
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(1.0, 0, 1),  
        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))  
    )
    
    
    lidar_3d = sens.ChLidarSensor(
        vehicle,  
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
    lidar_3d.SetName("3D Lidar Sensor")
    lidar_3d.SetLag(lag)
    lidar_3d.SetCollectionWindow(collection_time)

    
    lidar_2d = sens.ChLidarSensor(
        vehicle,  
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
        lidar_3d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    elif noise_model == "NONE":
        pass

    
    if vis:
        lidar_3d.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw 3D Lidar Depth Data"))
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, 1, "Raw 2D Lidar Depth Data"))
    
    
    lidar_3d.PushFilter(sens.ChFilterDIAccess())
    lidar_3d.PushFilter(sens.ChFilterPCfromDepth())
    lidar_3d.PushFilter(sens.ChFilterXYZIAccess())

    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())

    
    manager.AddSensor(lidar_3d)
    manager.AddSensor(lidar_2d)

    
    camera = sens.ChSensorCamera(
        vehicle,  
        "Third Person Camera",  
        0,  
        5,  
        10,  
        0,  
        0,  
        0,  
        45,  
        1,  
        1000  
    )
    camera.SetName("Third Person Camera")
    manager.AddSensor(camera)

    
    orbit_radius = 10
    orbit_rate = 0.1
    ch_time = 0.0
    end_time = 40.0
    step_size = 1e-3
    t1 = time.time()

    while ch_time < end_time:
        
        vehicle.Update()
        terrain.Update()
        
        
        lidar_3d.SetOffsetPose(offset_pose)
        lidar_2d.SetOffsetPose(offset_pose)

        
        xyzi_buffer = lidar_3d.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            print('3D Lidar buffer received. Resolution: {0}x{1}'.format(xyzi_buffer.Width, xyzi_buffer.Height))
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