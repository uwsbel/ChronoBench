import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh
import numpy as np
import time
import math

def main():
    
    mphysicalSystem = chrono.ChSystemNSC()

    
    vehicle = veh.ArticulatedVehicle(mphysicalSystem)

    
    vehicle.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngX(0)))

    
    vehicle.SetVehicleFile("path_to_vehicle_file.json")
    vehicle.SetContactMaterial(chrono.ChMaterialSurfaceNSC())
    vehicle.SetChassisMaterial(chrono.ChMaterialSurfaceNSC())
    vehicle.SetTireMaterial(chrono.ChMaterialSurfaceNSC())
    vehicle.SetWheelMaterial(chrono.ChMaterialSurfaceNSC())
    vehicle.SetSuspensionMaterial(chrono.ChMaterialSurfaceNSC())

    
    driver = veh.ChIrrlichtDriver()
    vehicle.AddDriver(driver)

    
    terrain = chrono.ChBodyEasyBox(mphysicalSystem, 100, 100, 1, 1000, True, chrono.ChVectorD(0, -5, 0))
    terrain.GetMaterialSurface().SetFriction(0.9)
    terrain.GetMaterialSurface().SetRestitution(0.8)
    terrain.GetMaterialSurface().SetCompliance(0.0001)
    terrain.GetMaterialSurface().SetDampingF(0.3)
    terrain.GetMaterialSurface().SetDampingV(0.3)

    
    mphysicalSystem.Add(terrain)

    
    manager = sens.ChSensorManager(mphysicalSystem)

    
    offset_pose = chrono.ChFramed(
        chrono.ChVectorD(1.0, 0, 1), chrono.Q_from_AngX(0)
    )
    lidar = sens.ChLidarSensor(
        vehicle.GetChassisBody(),  
        5.0,  
        offset_pose,  
        800,  
        300,  
        2 * chrono.CH_PI,  
        chrono.CH_PI / 12,  
        -chrono.CH_PI / 6,  
        100.0,  
        sens.LidarBeamShape_RECTANGULAR,  
        2,  
        0.003,  
        0.003,  
        sens.LidarReturnMode_STRONGEST_RETURN  
    )
    lidar.SetName("Lidar Sensor")
    lidar.SetLag(0)
    lidar.SetCollectionWindow(1. / 5.0)  

    
    lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    lidar.PushFilter(sens.ChFilterVisualize(800, 300, "Raw Lidar Depth Data"))
    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
    lidar.PushFilter(sens.ChFilterXYZIAccess())

    
    manager.AddSensor(lidar)

    
    lidar_2d = sens.ChLidarSensor(
        vehicle.GetChassisBody(),  
        5.0,  
        offset_pose,  
        800,  
        1,  
        2 * chrono.CH_PI,  
        0.0,  
        0.0,  
        100.0,  
        sens.LidarBeamShape_RECTANGULAR,  
        2,  
        0.003,  
        0.003,  
        sens.LidarReturnMode_STRONGEST_RETURN  
    )
    lidar_2d.SetName("2D Lidar Sensor")
    lidar_2d.SetLag(0)
    lidar_2d.SetCollectionWindow(1. / 5.0)  

    
    lidar_2d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    lidar_2d.PushFilter(sens.ChFilterVisualize(800, 1, "Raw 2D Lidar Depth Data"))
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())

    
    manager.AddSensor(lidar_2d)

    
    camera = sens.ChCameraSensor(
        vehicle.GetChassisBody(),  
        30.0,  
        chrono.ChFramed(
            chrono.ChVectorD(0, 0, 2), chrono.Q_from_AngX(0)
        ),  
        640,  
        480,  
        1.0,  
        "Third Person Camera"  
    )
    camera.SetCollectionWindow(1. / 30.0)  

    
    manager.AddSensor(camera)

    
    step_size = 1e-3
    end_time = 40.0
    ch_time = 0.0
    t1 = time.time()

    while ch_time < end_time:
        
        vehicle.Synchronize(chrono.ChTime(ch_time))
        vehicle.Advance(step_size)

        
        driver.Synchronize(chrono.ChTime(ch_time))
        driver.Advance(step_size)

        
        terrain.Synchronize(chrono.ChTime(ch_time))
        terrain.Advance(step_size)

        
        manager.Update()

        
        mphysicalSystem.DoStepDynamics(step_size)

        
        ch_time = mphysicalSystem.GetChTime()

    print("Sim time:", end_time, "Wall time:", time.time() - t1)

main()