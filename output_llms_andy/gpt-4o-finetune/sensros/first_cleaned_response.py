import os
import math
import numpy as np
import pychrono as chrono
import pychrono.sensor as sens
from pychrono import irrlicht as chronoirr

sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'pyros'))
from pyros import r_manager, r_sensor

def main():
    SCENE_FOLDER = sens.GetChronoDataFile("sensor/scene/").sys.data_dir

    
    sys = chrono.ChSystemNSC()
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
    sys.Initialize()

    
    trimesh = chrono.ChTriangleMeshConnected().CreateFromWavefrontFile(
        sens.GetChronoDataFile("mesh/vehicle/quadcopter.obj"), False, True)
    trimesg_shape = chrono.ChVisualShapeTriangleMesh()
    trimesg_shape.SetMesh(trimesh)
    trimesg_shape.SetName("Quadcopter Mesh")
    trimesg_shape.SetMutable(False)

    
    mbody = chrono.ChBody()
    mbody.SetPos(chrono.ChVector3d(0, 0, 0))
    mbody.AddVisualShape(trimesg_shape)
    mbody.SetFixed(False)
    sys.Add(mbody)

    
    ground_body = chrono.ChBodyEasyBox(1, 1, 1, 1000, False, False)
    ground_body.SetPos(chrono.ChVector3d(0, 0, 0))
    ground_body.SetFixed(False)
    ground_body.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/grid.png"))
    sys.Add(ground_body)

    

    
    cam_offset_pose = chrono.ChFramed(chrono.ChVector3d(-5, 0, 2), chrono.QuatFromAngleAxis(.2, chrono.ChVector3d(0, 1, 0)))
    cam = sens.ChCameraSensor(
        ground_body,                               
        30,                                        
        cam_offset_pose,                           
        1280,                                      
        720,                                       
        1.408,                                     
        "rgba",                                    
        sens.SENSOR_LOG_MODE_SEQUENCE                 
    )
    cam.PushFilter(sens.ChFilterVisualize(1280, 720))  
    cam.PushFilter(r_sensor.ChFilterROSPublishImage().SetTopic("mod_cam/image_rgba"))  
    manager.AddSensor(cam)

    
    lidar_offset_pose = chrono.ChFramed(chrono.ChVector3d(-5, 0, 2), chrono.QuatFromAngleAxis(.2, chrono.ChVector3d(0, 1, 0)))
    lidar = sens.ChLidarSensor(
        ground_body,                               
        10,                                        
        lidar_offset_pose,                         
        100,                                       
        2 * chrono.CH_PI,                          
        300,                                       
        -chrono.CH_PI / 12,                        
        +chrono.CH_PI / 6,                         
        100.0,                                     
        sens.LIDAR_DISTANCE,                       
        0.01                                       
    )
    lidar.PushFilter(sens.ChFilterDIAccess())  
    lidar.PushFilter(r_sensor.ChFilterROSPublishPCD().SetTopic("mod_lidar/pointcloud"))  
    manager.AddSensor(lidar)

    
    gps_reference = chrono.ChVector3d(-89.4, 433.07, 200.0)  
    gps = sens.ChGPSSensor(
        ground_body,                               
        10,                                        
        chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))),
        gps_reference,                             
        sens.GPSACCURACY_HIGH                       
    )
    gps.PushFilter(r_sensor.ChFilterROSPublishA2B().SetTopic("mod_gps/data"))  
    manager.AddSensor(gps)

    
    acc = sens.ChAccelerometerSensor(ground_body, 10, chrono.ChFramed())
    acc.PushFilter(r_sensor.ChFilterROSPublishA2B().SetTopic("mod_acc/data"))  
    manager.AddSensor(acc)

    
    gyro = sens.ChGyroscopeSensor(ground_body, 10, chrono.ChFramed())
    gyro.PushFilter(r_sensor.ChFilterROSPublishA2B().SetTopic("mod_gyro/data"))  
    manager.AddSensor(gyro)

    
    mag = sens.ChMagnetometerSensor(ground_body, 10, chrono.ChFramed())
    mag.PushFilter(r_sensor.ChFilterROSPublishA2B().SetTopic("mod_mag/data"))  
    manager.AddSensor(mag)

    
    ros_manager = r_manager.ChROSPythonManager()
    ros_manager.AddSensor(cam)
    ros_manager.AddSensor(lidar)
    ros_manager.AddSensor(gps)
    ros_manager.AddSensor(acc)
    ros_manager.AddSensor(gyro)
    ros_manager.AddSensor(mag)

    
    if not ros_manager.Initialize():
        print("Error initializing the ROS manager.")
        return False

    
    time = 0
    time_step = 1e-3  
    time_end = 100    

    
    radius = 4
    omega = 2 * chrono.CH_PI / 20  

    while time < time_end:
        time = sys.GetChTime()  

        
        ground_body.SetPos(chrono.ChVector3d(radius * math.cos(omega * time), radius * math.sin(omega * time), 0.0))

        manager.Update()  

        
        if not ros_manager.Update(time, time_step):
            break

        sys.DoStepDynamics(time_step)  

if __name__ == "__main__":
    main()