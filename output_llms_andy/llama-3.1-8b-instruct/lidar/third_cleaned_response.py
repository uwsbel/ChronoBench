import pychrono.core as chrono
import pychrono.sensor as sens

import numpy as np
import time
import math






m_vehicle = 1500


I_vehicle = 5000


wheel_base = 2.5


length = 4.5


width = 1.8


height = 1.5


wheel_radius = 0.3


v_vehicle = 10.0


steering_angle = 0.0


a_vehicle = 0.0


omega_vehicle = 0.0


alpha_vehicle = 0.0






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
    
    vehicle = chrono.ChVehicle()
    vehicle.SetMass(m_vehicle)
    vehicle.SetInertia(I_vehicle)
    vehicle.SetWheelBase(wheel_base)
    vehicle.SetLength(length)
    vehicle.SetWidth(width)
    vehicle.SetHeight(height)
    vehicle.SetWheelRadius(wheel_radius)
    vehicle.SetSpeed(v_vehicle)
    vehicle.SetSteeringAngle(steering_angle)
    vehicle.SetAcceleration(a_vehicle)
    vehicle.SetAngularVelocity(omega_vehicle)
    vehicle.SetAngularAcceleration(alpha_vehicle)

    
    terrain = chrono.ChTerrain()
    terrain.SetMaterial(chrono.ChMaterialSurface(chrono.ChMaterialParamTireFriction(0.5)))
    terrain.SetTexture(chrono.GetChronoDataFile("textures/terrain.png"))
    terrain.SetColor(chrono.ChColor(0.8, 0.8, 0.8))
    terrain.SetHeight(0.1)

    
    manager = sens.ChSensorManager(vehicle)

    
    lidar = sens.ChLidarSensor(
        vehicle.GetChassis(),  
        update_rate,            
        chrono.ChFramed(
            chrono.ChVector3d(1.0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
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
        vehicle.GetChassis(),  
        update_rate,            
        chrono.ChFramed(
            chrono.ChVector3d(1.0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
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
        vehicle.GetChassis(),  
        0.0, 0.0, 0.0,  
        0.0, 0.0, 0.0,  
        1.0, 1.0, 1.0  
    )
    camera.SetName("Third Person Camera")
    camera.SetResolution(640, 480)

    
    driver = chrono.ChDriver()
    driver.SetVehicle(vehicle)
    driver.SetSteeringAngle(steering_angle)
    driver.SetAcceleration(a_vehicle)
    driver.SetAngularVelocity(omega_vehicle)
    driver.SetAngularAcceleration(alpha_vehicle)

    
    vehicle_system = chrono.ChSystemNSC()
    vehicle_system.Add(vehicle)
    vehicle_system.Add(terrain)
    vehicle_system.Add(lidar)
    vehicle_system.Add(lidar_2d)
    vehicle_system.Add(camera)
    vehicle_system.Add(driver)

    
    sensor_manager = sens.ChSensorManager(vehicle_system)

    
    sensor_manager.AddSensor(lidar)
    sensor_manager.AddSensor(lidar_2d)

    
    orbit_radius = 10
    orbit_rate = 0.1
    ch_time = 0.0

    render_time = 0
    t1 = time.time()

    while ch_time < end_time:
        
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

        
        sensor_manager.Update()

        
        vehicle_system.DoStepDynamics(step_size)

        
        ch_time = vehicle_system.GetChTime()

    print("Sim time:", end_time, "Wall time:", time.time() - t1)

main()