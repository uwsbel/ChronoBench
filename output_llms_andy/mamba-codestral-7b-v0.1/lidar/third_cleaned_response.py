```python
import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as vehicle

import numpy as np
import time
import math




def main():
    
    vehicle_system = vehicle.ChVehicleSystem()
    vehicle_system.SetStepSize(step_size)

    
    driver = vehicle.ChDriver(vehicle_system)
    driver.SetSteeringMode(vehicle.ChSteeringMode.LOCKED)
    driver.SetSpeedMode(vehicle.ChSpeedMode.LIMITED)
    driver.SetSpeed(10)

    
    terrain = vehicle.ChTerrainRigid(vehicle_system)
    terrain.SetContactFrictionCoefficient(0.9)
    terrain.SetContactRestitution(0.01)
    terrain.SetTexture(chrono.GetChronoDataFile("textures/tile4.jpg"))
    terrain.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

    
    manager = sens.ChSensorManager(vehicle_system)

    
    lidar = sens.ChLidarSensor(
        vehicle_system.GetVehicleChassisBody(),
        update_rate,
        chrono.ChVector3d(1.0, 0, 1),
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
    manager.AddSensor(lidar)

    
    lidar_3d = sens.ChLidarSensor(
        vehicle_system.GetVehicleChassisBody(),
        update_rate,
        chrono.ChVector3d(1.0, 0, 1),
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
    manager.AddSensor(lidar_3d)

    
    camera = sens.ChCameraSensor(vehicle_system.GetVehicleChassisBody())
    camera.SetMode(sens.ChCameraMode.EYE)
    camera.SetPosition(chrono.ChVector3d(0, 1.8, -4))
    camera.SetRotation(chrono.ChQuaternionD(0, 0, 0))
    camera.SetResolution(1280, 720)
    camera.SetOutputTexture(True)
    camera.SetOutputDepth(True)
    camera.SetOutputPointCloud(True)
    manager.AddSensor(camera)

    
    orbit_radius = 10
    orbit_rate = 0.1
    ch_time = 0.0

    render_time = 0
    t1 = time.time()

    while ch_time < end_time:
        
        lidar.SetOffsetPose(
            chrono.ChFramed(
                chrono.ChVector3d(
                    -orbit_radius * math.cos(ch_time * orbit_rate),
                    -orbit_radius * math.sin(ch_time * orbit_rate),
                    1
                ),
                chrono.ChQuaternionD(0, 0, 0)
            )
        )

        
        manager.Update()

        
        vehicle_system.DoStepDynamics(step_size)
        driver.Synchronize(vehicle_system.GetVehicleChassisBody().GetPos(), vehicle_system.GetVehicleChassisBody().GetRot(), vehicle_system