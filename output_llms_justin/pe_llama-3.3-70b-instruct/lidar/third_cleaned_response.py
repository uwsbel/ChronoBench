import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh

import numpy as np
import time
import math


def main():
    
    
    
    mphysicalSystem = veh.ChSystem()

    
    car = veh.ARTcar()
    car.SetContactMethod(chrono.ChContactMethod_NSC)
    car.SetChassisCollisionType(chrono.ChCollisionType_BULLET)
    car.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
    car.Initialize()

    
    driver = veh.ChIrrGuiDriver()
    driver.SetVehicle(car)
    driver.SetSteeringMode(veh.DrivingMode_KEYBOARD)
    driver.Initialize()

    
    terrain = veh.RigidTerrain(mphysicalSystem)
    terrain.SetContactMaterial(chrono.ChMaterialSurfaceNSC())
    terrain.SetTexture(chrono.GetChronoDataFile("terrain/textures/tex_rockdiff.jpg"))
    terrain.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    
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

    
    lidar_2d = sens.ChLidarSensor(
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

    
    camera = sens.ChCameraSensor(
        car.GetChassisBody(),  
        30.0,  
        offset_pose,  
        640,  
        480,  
        60.0,  
        sens.CameraSensorMode_RGB
    )
    camera.SetName("Third Person Camera")
    camera.SetLag(0)
    camera.SetCollectionWindow(1. / 30.0)
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

        
        driver.Advance(1e-3)
        car.Advance(1e-3)

        
        mphysicalSystem.DoStepDynamics(1e-3)

        
        ch_time = mphysicalSystem.GetChTime()

    print("Sim time:", 40.0, "Wall time:", time.time() - t1)


if __name__ == '__main__':
    main()