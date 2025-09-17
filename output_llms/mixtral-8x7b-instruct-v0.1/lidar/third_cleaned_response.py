import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicles as veh
import pychrono.fea as fea

import numpy as np
import time
import math


def main():
    
    
    
    
    my_vehicle = veh.VehicleARTIC()
    my_vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    my_vehicle.SetChassisMass(1500)

    
    my_vehicle.Initialize(mphysicalSystem)

    
    mphysicalSystem = my_vehicle.GetVehicleSystem()

    
    
    
    terrain_mat = fea.ChMaterialSurfaceNSC()
    terrain_mat.SetYoungModulus(5e7)
    terrain_mat.SetFriction(0.5)
    terrain_mat.SetRestitution(0.1)

    terrain_mesh = chrono.ChTriangleMeshConnected()
    terrain_mesh.LoadMesh("terrain.obj")

    terrain = chrono.ChBodyEasyBox(100, 10, 10, 1000, True, True, terrain_mesh, terrain_mat)
    terrain.SetPos(chrono.ChVectorD(0, -5, 0))
    mphysicalSystem.AddBody(terrain)

    
    
    
    manager = sens.ChSensorManager(mphysicalSystem)

    
    
    
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(1.0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    lidar = sens.ChLidarSensor(
        my_vehicle.GetChassis(),  
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

    
    

    manager.AddSensor(lidar)

    
    
    
    lidar_2d = sens.ChLidarSensor(
        my_vehicle.GetChassis(),  
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

    
    

    manager.AddSensor(lidar_2d)

    
    
    
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(5, 5, 5), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    camera = sens.ChCameraSensor(
        my_vehicle.GetChassis(),  
        update_rate,
        offset_pose,
        640,
        480,
        60,
        True
    )
    camera.SetName("Camera Sensor")
    camera.SetLag(lag)
    camera.SetCollectionWindow(collection_time)

    
    

    manager.AddSensor(camera)

    
    
    
    my_driver = veh.ChVehicleDriver(my_vehicle)
    my_driver.SetDefaultCommand(veh.ChVehicleDriverCommand.CMD_ACCELERATE)

    orbit_radius = 10
    orbit_rate = 0.1
    ch_time = 0.0

    render_time = 0
    t1 = time.time()

    while ch_time < end_time:
        
        my_vehicle.Synchronize(ch_time, step_size)
        my_driver.Synchronize(ch_time, step_size)
        terrain.Synchronize(ch_time, step_size)

        
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


main()