import pychrono.core as chrono
import pychrono.sensor as sens

import numpy as np
import time
import math

def main():
    
    
    
    
    vehicle = chrono.ChVehicleART()
    vehicle.SetName("ARTcar")
    
    
    vehicle.SetInertia(chrono.ChVector3d(1000, 1000, 1000))
    vehicle.SetMass(1000)
    vehicle.SetWidth(0.5)
    vehicle.SetHeight(1.0)
    vehicle.SetWheelbase(1.2)
    vehicle.SetSuspension(0.5)
    vehicle.SetDamping(0.1, 0.1)
    vehicle.SetSteering(0.5)
    vehicle.SetEngine(100.0, 0.1)
    vehicle.SetMotor(100.0, 0.1)
    
    
    mphysicalSystem = chrono.ChSystemNSC()
    mphysicalSystem.Add(vehicle)
    
    
    
    
    manager = sens.ChSensorManager(mphysicalSystem)
    
    
    
    
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(1.0, 0, 1), 
        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    
    
    lidar = sens.ChLidarSensor(
        vehicle.GetChassis(),              
        10.0,                             
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
    lidar.SetName("3D Lidar Sensor")
    lidar.SetLag(0)
    lidar.SetCollectionWindow(1.0 / 10.0)  
    
    
    noise_model = "NONE"  
    
    
    lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01)) if noise_model == "CONST_NORMAL_XYZI" else None
    lidar.PushFilter(sens.ChFilterVisualize(800, 300, "Raw 3D Lidar Depth Data")) if vis else None
    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "3D Lidar Point Cloud"))
    lidar.PushFilter(sens.ChFilterXYZIAccess())
    
    manager.AddSensor(lidar)
    
    
    lidar_2d = sens.ChLidarSensor(
        vehicle.GetChassis(),
        10.0,
        offset_pose,
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
    lidar_2d.SetCollectionWindow(1.0 / 10.0)
    
    
    lidar_2d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01)) if noise_model == "CONST_NORMAL_XYZI" else None
    lidar_2d.PushFilter(sens.ChFilterVisualize(800, 1, "Raw 2D Lidar Depth Data")) if vis else None
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
    
    manager.AddSensor(lidar_2d)
    
    
    
    
    camera_offset = chrono.ChFramed(
        chrono.ChVector3d(0, 1.5, 1),
        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    third_person_camera = sens.ChSensorCamera(
        vehicle.GetChassis(),
        0,
        camera_offset,
        640,
        480,
        1.0,
        "Perspective",
        45.0,
        0.1,
        1000.0
    )
    third_person_camera.SetName("Third Person Camera")
    manager.AddSensor(third_person_camera)
    
    
    
    
    orbit_radius = 10
    orbit_rate = 0.1
    ch_time = 0.0
    end_time = 40.0
    
    render_time = 0
    t1 = time.time()
    
    while ch_time < end_time:
        
        vehicle.Update()
        driver = vehicle.GetDriver()
        driver.Update()
        
        
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
        
        
        mphysicalSystem.DoStepDynamics(step_size)
        
        
        ch_time = mphysicalSystem.GetChTime()
        
        
        if vis:
            render_time += 1.0 / 100.0
            if render_time >= 1.0:
                render_time = 0.0
                
                renderer = mphysicalSystem.GetRenderer()
                renderer.Render()
    
    print("Sim time:", end_time, "Wall time:", time.time() - t1)


noise_model = "NONE"  


return_mode = sens.LidarReturnMode_STRONGEST_RETURN


update_rate = 10.0


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