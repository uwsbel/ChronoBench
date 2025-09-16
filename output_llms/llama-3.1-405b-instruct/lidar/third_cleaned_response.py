import pychrono.core as chrono
import pychrono.sensor as sens
import numpy as np
import time
import math

def main():
    
    vehicle = chrono.ChVehicle()
    vehicle.SetInitPosition(chrono.ChVectorD(0, 0, 0.5))
    vehicle.SetInitRotation(chrono.ChQuaternionD(1, 0, 0, 0))
    vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
    vehicle.SetChassisCollisionType(chrono.ChCollisionType_BOX)
    vehicle.SetChassisDimensions(chrono.ChVectorD(1.5, 0.8, 0.8))
    vehicle.SetSuspensionType(chrono.ChSuspensionType_RIGID)
    vehicle.SetWheelType(chrono.ChWheelType_RIGID)
    vehicle.SetTireType(chrono.ChTireType_RIGID)
    vehicle.SetTireStepSize(1e-3)
    vehicle.SetWheelStepSize(1e-3)
    vehicle.SetTireFriction(chrono.ChVectorD(0.8, 0.7, 0.6))
    vehicle.SetBrakeFriction(chrono.ChVectorD(0.8, 0.7, 0.6))
    vehicle.SetMinBrakeTorque(10)
    vehicle.SetMaxBrakeTorque(100)
    vehicle.SetMinThrottleTorque(10)
    vehicle.SetMaxThrottleTorque(100)
    vehicle.SetMinSteeringAngle(-chrono.CH_PI / 4)
    vehicle.SetMaxSteeringAngle(chrono.CH_PI / 4)
    vehicle.SetMaxSpeed(50)
    vehicle.SetEngineDocument("engine.json")

    
    vehicle.Initialize()

    
    driver = chrono.ChIrrNodeDriver(vehicle)
    driver.Initialize()

    
    terrain = chrono.ChBodyEasyBox(10, 10, 1, 1000)
    terrain.SetPos(chrono.ChVectorD(0, 0, -1))
    terrain.GetVisualModel().GetShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
    terrain.GetVisualModel().GetShape(0).SetColor(chrono.ChColor(0.5, 0.5, 0.5))
    terrain.SetBodyFixed(True)

    
    vehicle.GetSystem().Add(terrain)

    
    manager = sens.ChSensorManager(vehicle.GetSystem())

    
    offset_pose = chrono.ChFramed(
        chrono.ChVectorD(1.0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVectorD(0, 1, 0))
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
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw 2D Lidar Depth Data"))
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar_2d)

    
    camera = sens.ChCameraSensor(
        vehicle.GetChassisBody(),  
        640,  
        480,  
        1.0,  
        10.0,  
        chrono.ChVectorD(0, 0, 1),  
        chrono.ChQuaternionD(1, 0, 0, 0)  
    )
    camera.SetName("Third Person Camera")
    manager.AddSensor(camera)

    
    orbit_radius = 10
    orbit_rate = 0.1
    ch_time = 0.0

    render_time = 0
    t1 = time.time()

    while ch_time < end_time:
        
        vehicle.Synchronize(driver, terrain, ch_time)

        
        manager.Update()

        
        vehicle.Advance(driver, terrain, step_size)

        
        vehicle.GetSystem().DoStepDynamics(step_size)

        
        ch_time = vehicle.GetSystem().GetChTime()

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