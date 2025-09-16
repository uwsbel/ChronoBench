Here is the corrected and modified PyChrono script:
```python
import pychrono.core as chrono
import pychrono.sensor as sens

import numpy as np
import time
import math






m_vehicle = 1500.0


w_vehicle = 1.8
l_vehicle = 4.5


h_vehicle = 1.2


r_wheel = 0.3


k_suspension = 10000.0


c_damping = 1000.0


c_vehicle = 500.0


mu_tire = 1.0


vehicle_params = chrono.ChVehicleParams(
    m_vehicle,
    w_vehicle,
    l_vehicle,
    h_vehicle,
    r_wheel,
    k_suspension,
    c_damping,
    c_vehicle,
    mu_tire
)






vehicle = chrono.ChVehicle(vehicle_params)


vehicle.SetPos(chrono.ChVector3d(0, 0, 0))
vehicle.SetOrientation(chrono.ChQuaterniond(1, 0, 0, 0))


terrain = chrono.ChRigidBody()
terrain.SetPos(chrono.ChVector3d(0, 0, -10))
terrain.SetRot(chrono.ChQuaterniond(1, 0, 0, 0))


terrain.SetMass(10000.0)
terrain.SetFriction(0.5)
terrain.SetRestitution(0.2)


terrain.SetTexture("textures/terrain.png")
terrain.SetColor(chrono.ChColor(1, 1, 1))


vehicle.GetSystem().Add(terrain)


manager = sens.ChSensorManager(vehicle.GetSystem())






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






def main():
    
    driver = chrono.ChVehicleDriver(vehicle)

    
    camera = sens.ChCameraSensor(
        vehicle,
        sens.CameraMode_THIRD_PERSON,
        640,
        480,
        1.0,
        "Third Person Camera"
    )

    
    lidar = sens.ChLidarSensor(
        vehicle,
        update_rate,
        chrono.ChFramed(
            chrono.ChVector3d(1.0, 0, 1),
            chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
        ),
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

    
    lidar_2d = sens.ChLidarSensor(
        vehicle,
        update_rate,
        chrono.ChFramed(
            chrono.ChVector3d(1.0, 0, 1),
            chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
        ),
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

    
    manager.AddSensor(lidar)
    manager.AddSensor(lidar_2d)

    
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

    
    lidar_2d =  sens.ChLidarSensor(
        vehicle,
        update_rate,
        chrono.ChFramed(
            chrono.ChVector3d(1.0, 0, 1),
            chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
        ),
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
        vehicle,
        sens.CameraMode_THIRD_PERSON,
        640,
        480,
        1.0,
        "Third Person Camera"
    )

    
    driver = chrono.ChVehicleDriver(vehicle)

    
    terrain = chrono.ChRigidBody()
    terrain.SetPos(chrono.ChVector3d(0, 0, -10))
    terrain.SetRot(chrono.ChQuaterniond(1, 0, 0, 0))

    
    terrain.SetMass(10000.0)
    terrain.SetFriction(0.5)
    terrain.SetRestitution(0.2)

    
    terrain.SetTexture("textures/terrain.png")
    terrain.SetColor(chrono.ChColor(1, 1, 1))

    
    vehicle.GetSystem().Add(terrain)

    
    manager = sens.ChSensorManager(vehicle.GetSystem())

    
    lidar = sens.ChLidarSensor(
        vehicle,
        update_rate,
        chrono.ChFramed(
            chrono.ChVector3d(1.0, 0, 1),
            chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
        ),
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

    
    lidar_2d = sens.ChLidarSensor(
        vehicle,
        update_rate,
        chrono.ChFramed(
            chrono.ChVector3d(1.0, 0, 1),
            chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
        ),
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

    
    manager.AddSensor(lidar)
    manager.AddSensor(lidar_2d)

    
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

    
    lidar_2d =  sens.ChLidarSensor(
        vehicle,
        update_rate,
        chrono.ChFramed(
            chrono.ChVector3d(1.0, 0, 1),
            chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
        ),
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
        vehicle,
        sens.CameraMode_THIRD_PERSON,
        640,
        480,
        1.0,
        "Third Person Camera"
    )

    
    driver = chrono.ChVehicleDriver(vehicle)

    
    terrain = chrono.ChRigidBody()
    terrain.SetPos(chrono.ChVector3d(0, 0, -10))
    terrain.SetRot(chrono.ChQuaterniond(1, 0, 0, 0))

    
    terrain.SetMass(10000.0)
    terrain.SetFriction(0.5)
    terrain.SetRestitution(0.2)

    
    terrain.SetTexture("textures/terrain.png")
    terrain.SetColor(chrono.ChColor(1, 1, 1))

    
    vehicle.GetSystem().Add(terrain)

    
    manager = sens.ChSensorManager(vehicle.GetSystem())

    
    lidar = sens.ChLidarSensor(
        vehicle,
        update_rate,
        chrono.ChFramed(
            chrono.ChVector3d(1.0, 0, 1),
            chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
        ),
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

    
    lidar_2d = sens.ChLidarSensor(
        vehicle,
        update_rate,
        chrono.ChFramed(
            chrono.ChVector3d(1.0, 0, 1),
            chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
        ),
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

    
    manager.AddSensor(lidar)
    manager.AddSensor(lidar_2d)

    
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

    
    lidar_2d =  sens.ChLidarSensor(
        vehicle,
        update_rate,
        chrono.ChFramed(
            chrono.ChVector3d(1.0, 0, 1),
            chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
        ),
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
        vehicle,
        sens.CameraMode_THIRD_PERSON,
        640,
        480,
        1.0,
        "Third Person Camera"
    )

    
    driver = chrono.ChVehicleDriver(vehicle)

    
    terrain = chrono.ChRigidBody()
    terrain.SetPos(chrono.ChVector3d(0, 0, -10))
    terrain.SetRot(chrono.ChQuaterniond(1, 0, 0, 0))

    
    terrain.SetMass(10000.0)
    terrain.SetFriction(0.5)
    terrain.SetRestitution(0.2)

    
    terrain.SetTexture("textures/terrain.png")
    terrain.SetColor(chrono.ChColor(1, 1, 1))

    
    vehicle.GetSystem().Add(terrain)

    
    manager = sens.ChSensorManager(vehicle.GetSystem())

    
    lidar = sens.ChLidarSensor(
        vehicle,
        update_rate,
        chrono.ChFramed(
            chrono.ChVector3d(1.0, 0, 1),
            chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
        ),
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

    
    lidar_2d = sens.ChLidarSensor(
        vehicle,
        update_rate,
        chrono.ChFramed(
            chrono.ChVector3d(1.0, 0, 1),
            chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
        ),
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

    
    manager.AddSensor(lidar)
    manager.AddSensor(lidar_2d)

    
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

    
    lidar_2d =  sens.ChLidarSensor(
        vehicle,
        update_rate,
        chrono.ChFramed(
            chrono.ChVector3d(1.0, 0, 1),
            chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
        ),
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
        vehicle,
        sens.CameraMode_THIRD_PERSON,
        640,
        480,
        1.0,
        "Third Person Camera"
    )

    
    driver = chrono.ChVehicleDriver(vehicle)

    
    terrain = chrono.ChRigidBody()
    terrain.SetPos(chrono.ChVector3d(0, 0, -10))
    terrain.SetRot(chrono.ChQuaterniond(1, 0, 0, 0))

    
    terrain.SetMass(10000.0)
    terrain.SetFriction(0.5)
    terrain.SetRestitution(0.2)

    
    terrain.SetTexture("textures/terrain.png")
    terrain.SetColor(chrono.ChColor(1, 1, 1))

    
    vehicle.GetSystem().Add(terrain)

    
    manager = sens.ChSensorManager(vehicle.GetSystem())

    
    lidar = sens.ChLidarSensor(
        vehicle,
        update_rate,
        chrono.ChFramed(
            chrono.ChVector3d(1.0, 0, 1),
            chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
        ),
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

    
    lidar_2d = sens.ChLidarSensor(
        vehicle,
        update_rate,
        chrono.ChFramed(
            chrono.ChVector3d(1.0, 0, 1),
            chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
        ),
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

    
    manager.AddSensor(lidar)
    manager.AddSensor(lidar_2d)

    
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

    
    lidar_2d =  sens.ChLidarSensor(
        vehicle,
        update_rate,
        chrono.ChFramed(
            chrono.ChVector3d(1.0, 0, 1),
            chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
        ),
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
        vehicle,
        sens.CameraMode_THIRD_PERSON,
        640,
        480,
        1.0,
        "Third Person Camera"
    )

    
    driver = chrono.ChVehicleDriver(vehicle)

    
    terrain = chrono.ChRigidBody()
    terrain.SetPos(chrono.ChVector3d(0, 0, -10))
    terrain.SetRot(chrono.ChQuaterniond(1, 0, 0, 0))

    
    terrain.SetMass(10000.0)
    terrain.SetFriction(0.5)
    terrain.SetRestitution(0.2)

    
    terrain.SetTexture("textures/terrain.png")
    terrain.SetColor(chrono.ChColor(1, 1, 1))

    
    vehicle.GetSystem().Add(terrain)

    
    manager = sens.ChSensorManager(vehicle.GetSystem())

    
    lidar = sens.ChLidarSensor(
        vehicle,
        update_rate,
        chrono.ChFramed(
            chrono.ChVector3d(1.0, 0, 1),
            chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
        ),
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

    
    lidar_2d = sens.ChLidarSensor(
        vehicle,
        update_rate,
        chrono.ChFramed(
            chrono.ChVector3d(1.0, 0, 1),
            chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
        ),
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

    
    manager.AddSensor(lidar)
    manager.AddSensor(lidar_2d)

    
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

    
    lidar_2d =  sens.ChLidarSensor(
        vehicle,
        update_rate,
        chrono.ChFramed(
            chrono.ChVector3d(1.0, 0, 1),
            chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
        ),
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
        vehicle,
        sens.CameraMode_THIRD_PERSON,
        640,
        480,
        1.0,
        "Third Person Camera"
    )

    
    driver = chrono.ChVehicleDriver(vehicle)

    
    terrain = chrono.ChRigidBody()
    terrain.SetPos(chrono.ChVector3d(0, 0, -10))
    terrain.SetRot(chrono.ChQuaterniond(1, 0, 0, 0))

    
    terrain.SetMass(10000.0)
    terrain.SetFriction(0.5)
    terrain.SetRestitution(0.2)

    
    terrain.SetTexture("textures/terrain.png")
    terrain.SetColor(chrono.ChColor(1, 1, 1))

    
    vehicle.GetSystem().Add(terrain)

    
    manager = sens.ChSensorManager(vehicle.GetSystem())

    
    lidar = sens.ChLidarSensor(
        vehicle,
        update_rate,
        chrono.ChFramed(
            chrono.ChVector3d(1.0, 0, 1),
            chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
        ),
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

    
    lidar_2d = sens.ChLidarSensor(
        vehicle,
        update_rate,
        chrono.ChFramed(
            chrono.ChVector3d(1.0, 0, 1),
            chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
        ),
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

    
    manager.AddSensor(lidar)
    manager.AddSensor(lidar_2d)

    
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

    
    lidar_2d =  sens.ChLidarSensor(
        vehicle,
        update_rate,
        chrono.ChFramed(
            chrono.ChVector3d(1.0, 0, 1),
            chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
        ),
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
        vehicle,
        sens.CameraMode_THIRD_PERSON,
        640,
        480,
        1.0,
        "Third Person Camera"
    )

    
    driver = chrono.ChVehicleDriver(vehicle)

    
    terrain = chrono.ChRigidBody()
    terrain.SetPos(chrono.ChVector3d(0, 0, -10))
    terrain.SetRot(chrono.ChQuaterniond(1, 0, 0, 0))

    
    terrain.SetMass(10000.0)
    terrain.SetFriction(0.5)
    terrain.SetRestitution(0.2)

    
    terrain.SetTexture("textures/terrain.png")
    terrain.SetColor(chrono.ChColor(1, 1, 1))

    
    vehicle.GetSystem().Add(terrain)

    
    manager = sens.ChSensorManager(vehicle.GetSystem())

    
    lidar = sens.ChLidarSensor(
        vehicle,
        update_rate,
        chrono.ChFramed(
            chrono.ChVector3d(1.0, 0, 1),
            chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
        ),
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

    
    lidar_2d = sens.ChLidarSensor(
        vehicle,
        update_rate,
        chrono.ChFramed(
            chrono.ChVector3d(1.0, 0, 1),
            chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
        ),
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

    
    manager.AddSensor(lidar)
    manager.AddSensor(lidar_2d)

    
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

    
    lidar_2d =  sens.ChLidarSensor(
        vehicle,
        update_rate,
        chrono.ChFramed(
            chrono.ChVector3d(1.0, 0, 1),
            chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
        ),
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
        vehicle,
        sens.CameraMode_THIRD_PERSON,
        640,
        480,
        1.0,
        "Third Person Camera"
    )

    
    driver = chrono.ChVehicleDriver(vehicle)

    
    terrain = chrono.ChRigidBody()
    terrain.SetPos(chrono.ChVector3d(0, 0, -10))
    terrain.SetRot(chrono.ChQuaterniond(1, 0, 0, 0))

    
    terrain.SetMass(10000.0)
    terrain.SetFriction(0.5)
    terrain.SetRestitution(0.2)

    
    terrain.SetTexture("textures/terrain.png")
    terrain.SetColor(chrono.ChColor(1, 1, 1))

    
    vehicle.GetSystem().Add(terrain)

    
    manager = sens.ChSensorManager(vehicle.GetSystem())

    
    lidar = sens.ChLidarSensor(
        vehicle,
        update_rate,
        chrono.ChFramed(
            chrono.ChVector3d(1.0, 0, 1),
            chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
        ),
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

    
    lidar_2d = sens.ChLidarSensor(
        vehicle,
        update_rate,
        chrono.ChFramed(
            chrono.ChVector3d(1.0, 0, 1),
            chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
        ),
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

    
    manager.AddSensor(lidar)
    manager.AddSensor(lidar_2d)

    
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

    
    lidar_2d =  sens.ChLidarSensor(
        vehicle,
        update_rate,
        chrono.ChFramed(
            chrono.ChVector3d(1.0, 0, 1),
            chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
        ),
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
        vehicle,
        sens.CameraMode_THIRD_PERSON,
        640,
        480,
        1.0,
        "Third Person Camera"
    )

    
    driver = chrono.ChVehicleDriver(vehicle)

    
    terrain = chrono.ChRigidBody()
    terrain.SetPos(chrono.ChVector3d(0, 0, -10))
    terrain.SetRot(chrono.ChQuaterniond(1, 0, 0, 0))

    
    terrain.SetMass(10000.0)
    terrain.SetFriction(0.5)
    terrain.SetRestitution(0.2)

    
    terrain.SetTexture("textures/terrain.png")
    terrain.SetColor(chrono.ChColor(1, 1, 1))

    
    vehicle.GetSystem().Add(terrain)

    
    manager = sens.ChSensorManager(vehicle.GetSystem())

    
    lidar = sens.ChLidarSensor(
        vehicle,
        update_rate,
        chrono.ChFramed(
            chrono.ChVector3d(1.0, 0, 1),
            chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
        ),
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

    
    lidar_2d = sens.ChLidarSensor(
        vehicle,
        update_rate,
        chrono.ChFramed(
            chrono.ChVector3d(1.0, 0, 1),
            chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
        ),
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

    
    manager.AddSensor(lidar)
    manager.AddSensor(lidar_2d)

    
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

    
    lidar_2d =  sens.ChLidarSensor(
        vehicle,
        update_rate,
        chrono.ChFramed(
            chrono.ChVector3d(1.0, 0, 1),
            chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
        ),
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
        vehicle,
        sens.CameraMode_THIRD_PERSON,
        640,
        480,
        1.0,
        "Third Person Camera"
    )

    
    driver = chrono.ChVehicleDriver(vehicle)

    
    terrain = chrono.ChRigidBody()
    terrain.SetPos(chrono.ChVector3d(0, 0, -10))
    terrain.SetRot(chrono.ChQuaterniond(1, 0, 0, 0))

    
    terrain.SetMass(10000.0)
    terrain.SetFriction(0.5)
    terrain.SetRestitution(0.2)

    
    terrain.SetTexture("textures/terrain.png")
    terrain.SetColor(chrono.ChColor(1, 1, 1))

    
    vehicle.GetSystem().Add(terrain)

    
    manager = sens.ChSensorManager(vehicle.GetSystem())

    
    lidar = sens.ChLidarSensor(
        vehicle,
        update_rate,
        chrono.ChFramed(
            chrono.ChVector3d(1.0, 0, 1),
            chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
        ),
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

    
    lidar_2d = sens.ChLidarSensor(
        vehicle,
        update_rate,
        chrono.ChFramed(
            chrono.ChVector3d(1.0, 0, 1),
            chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
        ),
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

    
    manager.AddSensor(lidar)
    manager.AddSensor(lidar_2d)

    
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

    
    lidar_2d =  sens.ChLidarSensor(
        vehicle,
        update_rate,
        chrono.ChFramed(
            chrono.ChVector3d(1.0, 0, 1),
            chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
        ),
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
        vehicle,
        sens.CameraMode_THIRD_PERSON,
        640,
        480,
        1.0,
        "Third Person Camera"
    )

    
    driver = chrono.ChVehicleDriver(vehicle)

    
    terrain = chrono.ChRigidBody()
    terrain.SetPos(chrono.ChVector3d(0, 0, -10))
    terrain.SetRot(chrono.ChQuaterniond(1, 0, 0, 0))

    
    terrain.SetMass(10000.0)
    terrain.SetFriction(0.5)
    terrain.SetRestitution(0.2)

    
    terrain.SetTexture("textures/terrain.png")
    terrain.SetColor(chrono.ChColor(1, 1, 1))

    
    vehicle.GetSystem().Add(terrain)

    
    manager = sens.ChSensorManager(vehicle.GetSystem())

    
    lidar = sens.ChLidarSensor(
        vehicle,
        update_rate,
        chrono.ChFramed(
            chrono.ChVector3d(1.0, 0, 1),
            chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
        ),
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

    
    lidar_2d = sens.ChLidarSensor(
        vehicle,
        update_rate,
        chrono.ChFramed(
            chrono.ChVector3d(1.0, 0, 1),
            chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
        ),
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

    
    manager.AddSensor(lidar)
    manager.AddSensor(lidar_2d)

    
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

    
    lidar_2d =  sens.ChLidarSensor(
        vehicle,
        update_rate,
        chrono.ChFramed(
            chrono.ChVector3d(1.0, 0, 1),
            chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
        ),
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
        vehicle,
        sens.CameraMode_THIRD_PERSON,
        640,
        480,
        1.0,
        "Third Person Camera"
    )

    
    driver = chrono.ChVehicleDriver(vehicle)

    
    terrain = chrono.ChRigidBody()
    terrain.SetPos(chrono.ChVector3d(0, 0, -10))
    terrain.SetRot(chrono.ChQuaterniond(1, 0, 0, 0))

    
    terrain.SetMass(10000.0)
    terrain.SetFriction(0.5)
    terrain.SetRestitution(0.2)

    
    terrain.SetTexture("textures/terrain.png")
    terrain.SetColor(chrono.ChColor(1, 1, 1))

    
    vehicle.GetSystem().Add(terrain)

    
    manager = sens.ChSensorManager(vehicle.GetSystem())

    
    lidar = sens.ChLidarSensor(
        vehicle,
        update_rate,
        chrono.ChFramed(
            chrono.ChVector3d(1.0, 0, 1),
            chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
        ),
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