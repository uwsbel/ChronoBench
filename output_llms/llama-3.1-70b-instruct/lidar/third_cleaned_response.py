import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import numpy as np
import time
import math

def main():
    
    vehicle = veh.ArticulatedVehicle()
    vehicle.SetContactMaterial(chrono.ChMaterialSurfaceNSC())
    vehicle.SetChassisCollisionBox(chrono.ChBox(1, 1, 1))
    vehicle.SetChassisBodyFixed(False)
    vehicle.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngX(0)))

    
    driver = veh.ArticulatedVehicleDriver()
    driver.SetVehicle(vehicle)
    driver.Initialize()

    
    terrain = chrono.ChTerrain()
    terrain.SetMaterial(chrono.ChMaterialSurfaceNSC())
    terrain.SetTexture(chrono.GetChronoDataFile("terrain/textures/tile4.jpg"))
    terrain.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.SetPlane(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngX(0)))

    
    mphysicalSystem = chrono.ChSystemNSC()
    mphysicalSystem.SetContactMaterial(chrono.ChMaterialSurfaceNSC())
    mphysicalSystem.SetMaxPenetrationRecoverySpeed(1.0)
    mphysicalSystem.SetMinBounceSpeed(1.0)

    
    mphysicalSystem.Add(vehicle.GetPart(0))
    mphysicalSystem.Add(terrain)

    
    manager = sens.ChSensorManager(mphysicalSystem)

    
    lidar = sens.ChLidarSensor(
        vehicle.GetPart(0),  
        5.0,  
        chrono.ChFrameD(chrono.ChVectorD(1.0, 0, 1), chrono.Q_from_AngX(0)),  
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

    
    lidar_2d = sens.ChLidarSensor(
        vehicle.GetPart(0),  
        5.0,  
        chrono.ChFrameD(chrono.ChVectorD(1.0, 0, 1), chrono.Q_from_AngX(0)),  
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

    
    manager.AddSensor(lidar)
    manager.AddSensor(lidar_2d)

    
    camera = sens.ChCameraSensor(
        vehicle.GetPart(0),  
        chrono.ChFrameD(chrono.ChVectorD(0, 0, 2), chrono.Q_from_AngX(0)),  
        640,  
        480,  
        1.0,  
        0.1,  
        100.0  
    )
    camera.SetName("Third Person View Camera")
    camera.SetCollectionWindow(1. / 30.0)  

    
    manager.AddSensor(camera)

    
    step_size = 1e-3
    end_time = 40.0

    
    ch_time = 0.0
    render_time = 0
    t1 = time.time()

    while ch_time < end_time:
        
        vehicle.Synchronize(chrono.ChSystem.GetLastStep())
        driver.Synchronize(chrono.ChSystem.GetLastStep())
        terrain.Synchronize(chrono.ChSystem.GetLastStep())

        
        manager.Update()

        
        mphysicalSystem.DoStepDynamics(step_size)

        
        ch_time = mphysicalSystem.GetChTime()

    print("Sim time:", end_time, "Wall time:", time.time() - t1)

main()