import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh

import numpy as np
import time
import math


def main():
    
    
    
    mphysicalSystem = chrono.ChSystemNSC()

    
    car = veh.ARTcar()
    car.SetContactMethod(chrono.ChContactMethod_NSC)
    car.SetChassisCollisionType(chrono.ChCollisionType_NONE)
    car.SetInitPosition(chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
    car.Initialize()

    
    mphysicalSystem.Add(car.GetChassisBody())

    
    driver = veh.ARTdriver(car)
    driver.Initialize()

    
    terrain = veh.RigidTerrain(car.GetSystem())
    patch = terrain.AddPatch(chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 100, 100)
    terrain.Initialize()

    
    
    side = 4
    box = chrono.ChBodyEasyBox(side, side, side, 1000)
    box.SetPos(chrono.ChVector3d(0, 0, 0))
    box.GetVisualModel().GetShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
    box.SetFixed(True)
    mphysicalSystem.Add(box)

    
    
    
    manager = sens.ChSensorManager(mphysicalSystem)

    
    
    
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(1.0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    lidar = sens.ChLidarSensor(
        car.GetChassisBody(),              
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

    
    lidar.PushFilter(sens.ChFilterXYZIAccess())

    
    manager.AddSensor(lidar)

    
    lidar_2d =  sens.ChLidarSensor(
        car.GetChassisBody(),              
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

    
    camera = sens.ChCameraSensor(car.GetChassisBody(), chrono.ChVector3d(0, 0, 2), chrono.ChVector3d(0, 0, 0), 640, 480)
    camera.SetName("Third Person View Camera")
    camera.SetCollectionWindow(1. / 30.0)
    manager.AddSensor(camera)

    
    
    
    orbit_radius = 10
    orbit_rate = 0.1
    ch_time = 0.0

    render_time = 0
    t1 = time.time()

    while ch_time < 40.0:
        
        car.Synchronize(1e-3)
        driver.Synchronize(1e-3)
        terrain.Synchronize(1e-3)

        
        manager.Update()

        
        mphysicalSystem.DoStepDynamics(1e-3)

        
        ch_time = mphysicalSystem.GetChTime()

    print("Sim time:", 40.0, "Wall time:", time.time() - t1)







end_time = 40.0


save = False


vis = True


out_dir = "SENSOR_OUTPUT/"






main()