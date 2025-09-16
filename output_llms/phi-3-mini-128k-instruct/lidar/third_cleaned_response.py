import pychrono.core as chrono
import pychrono.sensor as sens
import numpy as np
import time
import math



def initialize_vehicle_and_driver():
    vehicle = chrono.ChArticulatedBodyEasy(chrono.ChVector3d(0, 0, 0), chrono.ChQuaternion(1, 0, 0, 0))
    vehicle.SetName("ARTcar")
    vehicle.SetMass(1000)
    vehicle.SetInertiaXX(1000, 1000, 1000)
    vehicle.SetLocalInertia([1, 1, 1])
    vehicle.SetChassis(chrono.ChBodyEasyBox(1, 1, 1, 1000))
    vehicle.SetCollisionModel(chrono.ChCollisionShape_Box(chrono.ChVector3d(0, 0, 0), 10))
    vehicle.SetMotor(chrono.ChWheelMotor(1, 1, 1, 0, 0))
    vehicle.SetDriver(chrono.ChArticulatedBodyDriver())

    driver = chrono.ChArticulatedBodyDriver(vehicle)
    return vehicle, driver



def initialize_terrain():
    terrain = chrono.ChRigidBodyEasyBox(10, 1, 1, 0.1)
    terrain.SetName("Terrain")
    terrain.SetPosition(chrono.ChVector3d(0, 0, -1))
    terrain.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
    terrain.SetStatic(True)
    return terrain



def main():
    
    
    physicalSystem = chrono.ChSystemNSC()
    vehicle, driver = initialize_vehicle_and_driver()
    physicalSystem.Add(vehicle)
    physicalSystem.Add(initialize_terrain())

    
    
    manager = sens.ChSensorManager(physicalSystem)

    
    
    lidar = sens.ChLidarSensor(
        vehicle.GetBody(),                       
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
        vehicle.GetBody(),                    
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

    
    
    camera = sens.ChCameraSensor(
        vehicle.GetBody(),
        chrono.ChFramed(
            chrono.ChVector3d(0, 0, 0),
            chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
        chrono.ChMatrix33d().MakeLookAt(chrono.ChVector3d(0, 0, 1), chrono.ChVector3d(0, 0, 0))
    )
    camera.SetName("Third Person Camera")
    camera.SetFov(90, 60)
    camera.SetShutter(chrono.ChShutter(chrono.ChShutterMode_AUTO)
    camera.SetExposure(1000)
    camera.SetSensorResolution(640, 480)
    camera.SetShutterSpeed(1000)
    camera.SetFPS(30)

    
    
    manager.AddSensor(lidar)
    manager.AddSensor(lidar_2d)
    manager.AddSensor(camera)

    
    
    ch_time = 0.0
    render_time = 0
    t1 = time.time()

    while ch_time < end_time:
        
        physicalSystem.DoStepDynamics(step_size, True)
        driver.DoStepDynamics(step_size, True)
        vehicle.DoStepDynamics(step_size, True)
        terrain.DoStepDynamics(step_size, True)

        
        lidar.SetOffsetPose(
            chrono.ChFramed(
                chrono.ChVector3d(
                    -orbit_radius * math.cos(ch_time * orbit_rate),
                    -orbit_radius * math.sin(ch_time * orbit_rate),
                    1
                ),
                chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVector3d(0, 0, 1))
            )

        
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            print('XYZI buffer received from lidar. Lidar resolution: {0}x{1}'.format(xyzi_buffer.Width, xyzi_buffer.Height))
            print('Max Value: {0}'.format(np.max(xyzi_data)))

        
        manager.Update()

        
        ch_time = physicalSystem.GetChTime()

        
        if vis:
            camera.Render()

        
        time.sleep(1/30)  

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


out_dir = "SENSOR_OUTPUT/"








if __name__ == "__main__":
    main()