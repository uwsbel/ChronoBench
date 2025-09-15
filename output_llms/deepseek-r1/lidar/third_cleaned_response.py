import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh
import numpy as np
import time
import math

def main():
    
    
    
    vehicle = veh.ARTcar()
    vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
    vehicle.SetChassisFixed(False)
    vehicle.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 1), chrono.QUNIT))
    vehicle.Initialize()
    mphysicalSystem = vehicle.GetSystem()

    
    
    
    driver = veh.ChInteractiveDriverIRR(vehicle.GetVehicle())
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.02)
    driver.Initialize()

    
    
    
    terrain = veh.RigidTerrain(mphysicalSystem)
    patch_mat = chrono.ChContactMaterialData()
    patch_mat.friction = 0.9
    patch_mat.restitution = 0.01
    patch_mat.yield_pressure = 2e5
    patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(), 100, 100)
    patch.SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"), 100, 100)
    terrain.Initialize()

    
    
    
    manager = sens.ChSensorManager(mphysicalSystem)
    manager.scene.AddPointLight(chrono.ChVector3d(0,0,100), chrono.ChColor(1,1,1), 5000)

    
    
    
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(1.0, 0, 1), 
        chrono.ChQuaterniond.FromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
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
    lidar.SetName("3D Lidar")
    lidar.SetLag(lag)
    lidar.SetCollectionWindow(collection_time)

    
    if noise_model == "CONST_NORMAL_XYZI":
        lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    lidar.PushFilter(sens.ChFilterXYZIAccess())
    if vis:
        lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "3D Lidar Depth"))
        lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "3D Point Cloud"))
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
    lidar_2d.SetName("2D Lidar")
    lidar_2d.SetLag(lag)
    lidar_2d.SetCollectionWindow(collection_time)

    
    if noise_model == "CONST_NORMAL_XYZI":
        lidar_2d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
    if vis:
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, 1, "2D Lidar Depth"))
    manager.AddSensor(lidar_2d)

    
    
    
    cam_pose = chrono.ChFramed(
        chrono.ChVector3d(-5, 0, 3),
        chrono.ChQuaterniond.FromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0))
    )
    camera = sens.ChCameraSensor(
        vehicle.GetChassisBody(),  
        30,                        
        cam_pose,                  
        1280,                      
        720,                       
        chrono.CH_PI/3             
    )
    camera.SetName("Third Person Camera")
    camera.PushFilter(sens.ChFilterVisualize(1280, 720))
    manager.AddSensor(camera)

    
    
    
    orbit_radius = 10
    orbit_rate = 0.1
    ch_time = 0.0

    while ch_time < end_time:
        
        driver.Synchronize(ch_time)
        
        
        vehicle.Synchronize(ch_time, driver.GetInputs(), terrain)
        terrain.Synchronize(ch_time)

        
        manager.Update()

        
        driver.Advance(step_size)
        vehicle.Advance(step_size)
        terrain.Advance(step_size)
        mphysicalSystem.DoStepDynamics(step_size)

        ch_time = mphysicalSystem.GetChTime()




noise_model = "NONE"
return_mode = sens.LidarReturnMode_STRONGEST_RETURN
update_rate = 5.0
horizontal_samples = 800
vertical_samples = 300
horizontal_fov = 2 * chrono.CH_PI
max_vert_angle = chrono.CH_PI / 12
min_vert_angle = -chrono.CH_PI / 6
lag = 0.0
collection_time = 1.0 / update_rate
sample_radius = 2
divergence_angle = 0.003




step_size = 1e-3
end_time = 40.0
vis = True

if __name__ == "__main__":
    main()