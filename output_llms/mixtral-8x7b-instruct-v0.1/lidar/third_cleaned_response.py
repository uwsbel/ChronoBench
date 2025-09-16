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
    vehicle_params.wheel_radius = 0.3
    vehicle_params.wheel_width = 0.2
    vehicle_params.wheel_mass = 10
    vehicle_params.wheel_friction = 1
    vehicle_params.wheel_rolling_resistance = 0.01
    vehicle_params.wheel_spring_rate = 200000
    vehicle_params.wheel_damper_rate = 10000
    vehicle_params.steering_ratio = 14
    vehicle_params.steering_limit = 0.5
    vehicle_params.brake_force = 5000
    vehicle_params.drive_torque = 500
    vehicle_params.max_brake_bias = 0.9
    vehicle = veh.Vehicle(vehicle_params)

    
    vehicle.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0)))

    
    terrain_mat = fea.ChMaterialSurfaceNSC()
    terrain_mat.SetFriction(0.5)
    terrain_mat.SetYoungsModulus(5e6)
    terrain_mat.SetRestitution(0.1)
    terrain_mat.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
    terrain_tex = chrono.ChTexture(chrono.GetChronoDataFile('textures/grass.jpg'))
    terrain = fea.ChTerrain(terrain_mat, 100, 100, 0.1)
    terrain.SetTexture(terrain_tex)

    
    mphysicalSystem = chrono.ChSystemNSC()
    mphysicalSystem.SetSolverType(chrono.ChSolver.Type.SOR)
    mphysicalSystem.SetSolverForceAndTorque(True)
    mphysicalSystem.SetMaxIterations(100)
    mphysicalSystem.SetSolverMinIterations(50)

    
    mphysicalSystem.AddBody(vehicle.GetChassisBody())

    
    mphysicalSystem.Add(terrain)

    
    
    
    manager = sens.ChSensorManager(mphysicalSystem)

    
    
    
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(1.0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
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
    
    lidar.PushFilter(sens.ChFilterXYZIAccess())
    
    manager.AddSensor(lidar)

    
    lidar_2d =  sens.ChLidarSensor(
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
        
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw 2D Lidar Depth Data"))
    
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
    
    manager.AddSensor(lidar_2d)

    
    
    
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(1.0, 1.5, 1.5), chrono.QuatFromAngleAxis(chrono.CH_C_PI / 4, chrono.ChVector3d(0, 1, 0))
    )
    camera = sens.ChCameraSensor(
        vehicle.GetChassisBody(),              
        30,                                   
        offset_pose,                            
        800, 600,                               
        0.1, 100.0,                             
        True,                                  
        True                                   
    )
    camera.SetName("Camera Sensor")
    camera.SetLag(lag)
    camera.SetCollectionWindow(collection_time)

    
    
    
    if vis:
        
        camera.PushFilter(sens.ChFilterVisualize(800, 600, "Camera View"))
    
    camera.PushFilter(sens.ChFilterImageAccess())
    
    manager.AddSensor(camera)

    
    
    
    orbit_radius = 10
    orbit_rate = 0.1
    ch_time = 0.0

    render_time = 0
    t1 = time.time()

    
    driver = veh.ChVehicleDriver(vehicle)

    while ch_time < end_time:
        
        driver.Synchronize(mphysicalSystem, step_size)
        driver.Advance(step_size)

        
        terrain.Synchronize(mphysicalSystem, step_size)
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