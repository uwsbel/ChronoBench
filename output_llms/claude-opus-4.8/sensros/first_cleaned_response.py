import pychrono as ch
import pychrono.sensor as sens
import pychrono.ros as chros

import math
import numpy as np


def main():
    
    
    
    
    sys = ch.ChSystemNSC()
    sys.SetGravitationalAcceleration(ch.ChVector3d(0, 0, -9.81))

    
    
    
    
    ground = ch.ChBody()
    ground.SetFixed(False)          
    ground.SetPos(ch.ChVector3d(0, 0, 0))
    ground.EnableCollision(False)
    sys.Add(ground)

    
    mesh = ch.ChTriangleMeshConnected()
    try:
        mesh.LoadWavefrontMesh(
            ch.GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"),
            False, True)
        mesh.Transform(ch.ChVector3d(0, 0, 0),
                       ch.ChMatrix33d(1))  

        trimesh_shape = ch.ChVisualShapeTriangleMesh()
        trimesh_shape.SetMesh(mesh)
        trimesh_shape.SetName("ground_mesh")
        trimesh_shape.SetMutable(False)
        ground.AddVisualShape(trimesh_shape, ch.ChFramed())
    except Exception as e:
        print("Could not load mesh, continuing without it:", e)

    
    
    
    
    ground.SetPosDt(ch.ChVector3d(0.2, 0, 0))   
    
    ground.SetAngVelLocal(ch.ChVector3d(0, 0, 0.1))

    
    
    
    manager = sens.ChSensorManager(sys)

    
    manager.scene.AddPointLight(
        ch.ChVector3f(2, 2, 2),
        ch.ChColor(1, 1, 1),
        500.0)

    
    update_rate = 30.0          
    offset_pose = ch.ChFramed(
        ch.ChVector3d(0.0, 0.0, 1.0),
        ch.QuatFromAngleAxis(0, ch.ChVector3d(0, 0, 1)))

    
    cam_width = 640
    cam_height = 480
    cam_fov = 1.408  
    camera = sens.ChCameraSensor(
        ground,            
        update_rate,       
        offset_pose,       
        cam_width,         
        cam_height,        
        cam_fov)           
    camera.SetName("camera_sensor")
    camera.PushFilter(sens.ChFilterVisualize(cam_width, cam_height))
    camera.PushFilter(sens.ChFilterRGBA8Access())
    manager.AddSensor(camera)

    
    lidar_hsamples = 800
    lidar_vsamples = 32
    lidar = sens.ChLidarSensor(
        ground,                    
        update_rate,               
        offset_pose,               
        lidar_hsamples,            
        lidar_vsamples,            
        2 * math.pi,               
        ch.CH_PI / 12,             
        -ch.CH_PI / 6,             
        100.0)                     
    lidar.SetName("lidar_sensor")
    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0))
    manager.AddSensor(lidar)

    
    gps_reference = ch.ChVector3d(-121.75, 38.55, 0.0)  
    gps_noise = sens.ChNoiseNone()
    gps = sens.ChGPSSensor(
        ground,
        update_rate,
        offset_pose,
        gps_reference,
        gps_noise)
    gps.SetName("gps_sensor")
    gps.PushFilter(sens.ChFilterGPSAccess())
    manager.AddSensor(gps)

    
    acc_noise = sens.ChNoiseNone()
    accelerometer = sens.ChAccelerometerSensor(
        ground,
        update_rate,
        offset_pose,
        acc_noise)
    accelerometer.SetName("accelerometer_sensor")
    accelerometer.PushFilter(sens.ChFilterAccelAccess())
    manager.AddSensor(accelerometer)

    
    gyro_noise = sens.ChNoiseNone()
    gyroscope = sens.ChGyroscopeSensor(
        ground,
        update_rate,
        offset_pose,
        gyro_noise)
    gyroscope.SetName("gyroscope_sensor")
    gyroscope.PushFilter(sens.ChFilterGyroAccess())
    manager.AddSensor(gyroscope)

    
    mag_noise = sens.ChNoiseNone()
    magnetometer = sens.ChMagnetometerSensor(
        ground,
        update_rate,
        offset_pose,
        mag_noise,
        gps_reference)
    magnetometer.SetName("magnetometer_sensor")
    magnetometer.PushFilter(sens.ChFilterMagnetAccess())
    manager.AddSensor(magnetometer)

    
    
    
    ros_manager = chros.ChROSPythonManager()

    
    ros_manager.RegisterHandler(chros.ChROSClockHandler())

    
    ros_manager.RegisterHandler(
        chros.ChROSCameraHandler(
            camera.GetUpdateRate(), camera,
            "~/output/camera/data/image"))

    
    ros_manager.RegisterHandler(
        chros.ChROSLidarHandler(
            lidar, "~/output/lidar/data/pointcloud"))

    
    ros_manager.RegisterHandler(
        chros.ChROSGPSHandler(
            gps, "~/output/gps/data"))

    
    ros_manager.RegisterHandler(
        chros.ChROSAccelerometerHandler(
            accelerometer, "~/output/accelerometer/data"))

    
    ros_manager.RegisterHandler(
        chros.ChROSGyroscopeHandler(
            gyroscope, "~/output/gyroscope/data"))

    
    ros_manager.RegisterHandler(
        chros.ChROSMagnetometerHandler(
            magnetometer, "~/output/magnetometer/data"))

    
    ros_manager.Initialize()

    
    
    
    time = 0.0
    time_end = 30.0
    step_size = 1e-3

    
    realtime_timer = ch.ChRealtimeStepTimer()

    print("Starting simulation...")
    while time < time_end:
        time = sys.GetChTime()

        
        manager.Update()

        
        if not ros_manager.Update(time, step_size):
            break

        
        sys.DoStepDynamics(step_size)

        
        realtime_timer.Spin(step_size)

    print("Simulation finished.")


if __name__ == "__main__":
    main()