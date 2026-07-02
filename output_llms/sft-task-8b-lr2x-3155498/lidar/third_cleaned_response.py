import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens

import numpy as np
import time
import math


def main():
    
    
    
    veh.SetVehicleDataPath(chrono.GetChronoDataPath() + "vehicle/")
    vehicle = veh.ArticulatedCar()
    vehicle.SetChassisMass(1000.0)
    vehicle.SetSuspensionStiffness(100000.0)
    vehicle.SetSuspensionDamping(1000.0)
    vehicle.SetWheelMass(20.0)
    vehicle.SetWheelInertia(0.01)
    vehicle.SetTireFriction(1.0)
    vehicle.SetTireRestitution(0.3)
    vehicle.SetTireRollingResistance(0.01)
    system = vehicle.GetSystem()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

    
    
    
    driver = veh.ChDriver(vehicle.GetVehicle())
    driver.Initialize()

    
    
    
    terrain = veh.ChRigidTerrain(system)
    terrain.Initialize()
    mat = terrain.GetMaterial()
    mat.SetFriction(0.9)
    mat.SetRestitution(0.01)
    terrain.AddTexture(chrono.GetChronoDataFile("textures/tarmac.png"), 100, 100)
    terrain.SetColor(chrono.ChColor(0.8, 0.8, 0.8))

    
    
    
    manager = sens.ChSensorManager(system)

    
    lidar = sens.ChLidarSensor(
        vehicle.GetChassisBody(),  
        update_rate,               
        chrono.ChFramed(chrono.ChVector3d(1.0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))),
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
        chrono.ChFramed(chrono.ChVector3d(1.0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))),
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

    
    
    
    camera = sens.ChCameraSensor(
        vehicle.GetChassisBody(),
        update_rate,
        chrono.ChFramed(chrono.ChVector3d(1.5, -1.0, 1.5), chrono.QuatFromAngleAxis(-0.3, chrono.ChVector3d(0, 1, 0))),
        640,
        480,
        0.1,
        100.0
    )
    camera.SetName("Third-Person Camera")
    camera.SetImageFormat(sens.ChImageFormat_R8G8B8)
    camera.SetFocalPoint(chrono.ChVector3d(0, 0, 0))
    camera.PushFilter(sens.ChFilterVisualize(640, 480, "Third-Person Camera Image"))
    manager.AddSensor(camera)

    
    
    
    step_size = 1e-3
    end_time = 40.0
    render_time = 0
    t1 = time.time()

    while system.GetChTime() < end_time:
        
        driver.Synchronize(system.GetChTime())
        driver.Advance(step_size)

        
        vehicle.Advance(step_size)
        terrain.Update()

        
        manager.Update()

        
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            print('XYZI buffer received from lidar. Lidar resolution: {0}x{1}'.format(xyzi_buffer.Width, xyzi_buffer.Height))
            print('Max Value: {0}'.format(np.max(xyzi_data)))

        
        ch_time = system.GetChTime()

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
collection_time = 1.0 / update_rate
sample_radius = 2
divergence_angle = 0.003




step_size = 1e-3
end_time = 40.0
vis = True

main()