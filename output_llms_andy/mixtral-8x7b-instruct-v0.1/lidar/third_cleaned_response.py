import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicles as veh
import pychrono.fea as fea

import numpy as np
import time
import math


def main():
    
    
    
    
    chrono.SetChronoDataPath('path/to/data')

    
    vehicle_params = veh.VehicleCreateParams()
    vehicle_params.vehicle_model = veh.VehicleModelType.ARCHETYPE_CAR
    vehicle_params.chassis_filename = 'ARCHETYPE_CAR/chassis.obj'
    vehicle_params.wheel_filename = 'ARCHETYPE_CAR/wheel.obj'
    vehicle_params.wheel_radius = 0.35
    vehicle_params.wheel_width = 0.2
    vehicle_params.wheel_mass = 10
    vehicle_params.wheel_friction = 1.2
    vehicle_params.wheel_rolling_friction = 0.01
    vehicle_params.wheel_spring_stiffness = 1e5
    vehicle_params.wheel_damper_stiffness = 1e3
    vehicle_params.steering_ratio = 14.0
    vehicle_params.steering_limit = 0.5
    vehicle_params.steering_spring_stiffness = 500
    vehicle_params.steering_damper_stiffness = 50
    vehicle_params.brake_force = 4000
    vehicle_params.max_brake_torque = 50
    vehicle_params.clutch_stiffness = 500
    vehicle_params.drivetrain_efficiency = 0.95
    vehicle = veh.Vehicle(vehicle_params)

    
    vehicle.InitializeDriver()

    
    terrain_mat = fea.ChMaterialSurfaceNSC(chrono.ChVector(0.5, 0.5, 0.5))
    terrain_mat.SetFriction(0.5)
    terrain_mat.SetRestitution(0.1)
    terrain_tex = chrono.ChTexture(chrono.GetChronoDataFile('textures/terrain.jpg'))
    terrain_tex.SetWrapS(chrono.ChTexture.TextureWrap_REPEAT)
    terrain_tex.SetWrapT(chrono.ChTexture.TextureWrap_REPEAT)
    terrain = fea.ChBodyEasyBox(100, 1, 10, 1000, True, True, terrain_mat, terrain_tex)
    terrain.SetPos(chrono.ChVector3(0, -1, 0))

    
    mphysicalSystem = chrono.ChSystemNSC()
    mphysicalSystem.AddBody(terrain)

    
    
    
    
    side = 4
    box = chrono.ChBodyEasyBox(side, side, side, 1000)
    box.SetPos(chrono.ChVector3(0, 0, 0))
    box.GetVisualModel().GetShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
    box.SetFixed(True)
    mphysicalSystem.Add(box)

    
    
    
    manager = sens.ChSensorManager(mphysicalSystem)

    
    
    
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(1.0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    lidar = sens.ChLidarSensor(
        vehicle.GetChassis(),  
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
    lidar.SetName("3D Lidar Sensor")
    lidar.SetLag(lag)
    lidar.SetCollectionWindow(collection_time)

    
    
    
    lidar_2d = sens.ChLidarSensor(
        vehicle.GetChassis(),  
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

    
    
    
    camera_offset_pose = chrono.ChFramed(
        chrono.ChVector3d(0, 5, 5), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 0, 1))
    )
    camera = sens.ChCameraSensor(
        vehicle.GetChassis(),  
        update_rate,  
        camera_offset_pose,  
        640,  
        480,  
        1.0,  
        100.0,  
        0.1,  
        sens.CameraReturnMode_RGB,  
    )
    camera.SetName("Third Person View Camera")
    camera.SetLag(lag)
    camera.SetCollectionWindow(collection_time)

    
    
    
    if noise_model == "CONST_NORMAL_XYZI":
        lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
        lidar_2d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    elif noise_model == "NONE":
        
        pass
    if vis:
        
        lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw 3D Lidar Depth Data"))
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, 1, "Raw 2D Lidar Depth Data"))
    
    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    
    lidar.PushFilter(sens.ChFilterXYZIAccess())
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
    
    manager.AddSensor(lidar)
    manager.AddSensor(lidar_2d)
    manager.AddSensor(camera)

    
    
    
    orbit_radius = 10
    orbit_rate = 0.1
    ch_time = 0.0

    render_time = 0
    t1 = time.time()

    while ch_time < end_time:
        
        vehicle.Synchronize(ch_time)
        vehicle.Advance(step_size)
        vehicle.Driver().Synchronize(ch_time)
        terrain.Synchronize(ch_time)
        terrain.Advance(step_size)

        
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


main()