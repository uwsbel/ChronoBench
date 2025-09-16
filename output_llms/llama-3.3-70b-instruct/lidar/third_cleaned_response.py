import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import numpy as np
import time
import math


def main():
    
    
    
    mphysicalSystem = chrono.ChSystemNSC()

    
    vehicle = veh.ArticulatedVehicle(mphysicalSystem)

    
    vehicle.SetVehicleFile("path/to/vehicle/data")
    vehicle.SetContactMaterial(chrono.ChMaterialSurfaceNSC())
    vehicle.SetChassisBodyCollide(True)
    vehicle.SetSuspensionCollide(True)
    vehicle.SetWheelCollide(True)
    vehicle.SetTireCollide(True)

    
    vehicle.Initialize()

    
    driver = veh.ChIrrlichtDriver()
    driver.SetVehicle(vehicle)
    driver.SetSteering(0)
    driver.SetThrottle(0)

    
    terrain = chrono.ChBodyEasyBox(mphysicalSystem, 100, 100, 1, 1000, True, True)
    terrain.SetPos(chrono.ChVector3d(0, 0, -1))
    terrain.SetMaterial(chrono.ChMaterialSurfaceNSC())
    terrain.GetVisualModel().GetShape(0).SetTexture(chrono.GetChronoDataFile("textures/grass.jpg"))
    terrain.GetVisualModel().GetShape(0).SetColor(chrono.ChColor(0.2, 0.6, 0.2))
    mphysicalSystem.Add(terrain)

    
    manager = sens.ChSensorManager(mphysicalSystem)

    
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(1.0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
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

    
    lidar_2d =  sens.ChLidarSensor(
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

    
    camera_pose = chrono.ChFramed(
        chrono.ChVector3d(0, -5, 2), chrono.QuatFromAngleAxis(chrono.CH_PI / 4, chrono.ChVector3d(0, 1, 0))
    )
    camera = sens.ChCameraSensor(
        vehicle.GetChassisBody(),              
        30.0,            
        camera_pose,            
        640,     
        480,       
        1.0         
    )
    camera.SetName("Third Person Camera")
    camera.PushFilter(sens.ChFilterVisualize(640, 480, "Third Person View"))
    manager.AddSensor(camera)

    
    
    
    orbit_radius = 10
    orbit_rate = 0.1
    ch_time = 0.0

    render_time = 0
    t1 = time.time()

    while ch_time < 40.0:
        
        lidar.SetOffsetPose(
            chrono.ChFramed(
                chrono.ChVector3d(
                    -orbit_radius * math.cos(ch_time * orbit_rate),
                    -orbit_radius * math.sin(ch_time * orbit_rate),
                    1
                ),
                chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVector3d(0, 0, 1))
            )
        )

        
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            print('XYZI buffer received from lidar. Lidar resolution: {0}x{1}'.format(xyzi_buffer.Width, xyzi_buffer.Height))
            print('Max Value: {0}'.format(np.max(xyzi_data)))

        
        manager.Update()

        
        mphysicalSystem.DoStepDynamics(1e-3)

        
        vehicle.Advance(1e-3)

        
        driver.Advance(1e-3)

        
        ch_time = mphysicalSystem.GetChTime()

    print("Sim time:", 40.0, "Wall time:", time.time() - t1)



main()