import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh

import numpy as np
import time
import math


def main():
    
    
    
    mphysicalSystem = chrono.ChSystemNSC()

    
    
    
    vehicle = veh.ARTcar()
    vehicle.SetChassisMass(1500)  
    vehicle.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1)), mphysicalSystem)  

    
    driver = veh.ChDriver()
    vehicle.SetDriver(driver)

    
    
    
    material = veh.ChMaterialSurfaceNSC()
    material.SetFriction(0.9)
    terrain = veh.RigidTerrain(mphysicalSystem, material)
    terrain.SetColor(chrono.ChColor(0.4, 0.4, 0.4))
    terrain.SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"), 200, 200)
    terrain.Initialize()

    
    
    
    manager = sens.ChSensorManager(mphysicalSystem)

    
    
    
    offset_pose = chrono.ChFrameD(
        chrono.ChVectorD(1.0, 0, 1),
        chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))
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
    

    
    camera_offset = chrono.ChFrameD(
        chrono.ChVectorD(2, 0, 1),
        chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.ChVectorD(1, 0, 0))
    )
    camera = sens.ChCameraSensor(
        vehicle.GetChassisBody(),
        camera_offset,
        90.0,  
        640, 480  
    )
    camera.SetName("Third Person Camera")
    camera.PushFilter(sens.ChFilterVisualize())
    manager.AddSensor(camera)

    
    manager.AddSensor(lidar)
    manager.AddSensor(lidar_2d)

    
    
    
    ch_time = 0.0
    render_time = 0
    t1 = time.time()

    while ch_time < end_time:
        
        driver.Update()
        vehicle.Update(chrono.GetChTimeStep())
        terrain.Update()

        
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
vis = True

if __name__ == '__main__':
    main()