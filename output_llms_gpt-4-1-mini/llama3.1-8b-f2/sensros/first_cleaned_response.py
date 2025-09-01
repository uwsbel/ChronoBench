import pychrono as ch
import pychrono.sensor as sens
import pychrono.ros as chros

import rclpy.publisher
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
import numpy as np
import time


def main():
    
    system = ch.ChSystemNSC()
    system.SetGravitationalAcceleration(ch.ChVector3d(0, 0, -9.81))
    ch.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
    ch.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

    
    ground_mat = ch.ChContactMaterialNSC()
    ground = ch.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
    ground.SetPos(ch.ChVector3d(0, 0, -1))
    ground.SetFixed(False)
    ground.SetMass(0)
    ground.GetVisualShape(0).SetTexture(ch.GetChronoDataFile("textures/concrete.jpg"))
    system.Add(ground)

    
    body_mat = ch.ChContactMaterialNSC()
    body = ch.ChBodyEasyBox(1, 1, 1, 1000, True, True, body_mat)
    body.SetPos(ch.ChVector3d(0, 0, 5))
    body.SetRot(ch.QuatFromAngleAxis(.2, ch.ChVector3d(1, 0, 0)))
    body.SetMass(200)
    body.SetName("obj")
    system.Add(body)

    
    offset_pose = ch.ChFramed(ch.ChVector3d(-5, 0, 2), ch.QuatFromAngleAxis(.2, ch.ChVector3d(0, 1, 0)))
    manager = sens.ChSensorManager(system)
    manager.scene.AddPointLight(ch.ChVector3f(100, 100, 100))
    manager.scene.AddPointLight(ch.ChVector3f(-100, -100, -100))

    
    update_rate = 30
    exposure_time = 1 / update_rate
    width, height, _ = body.GetVisualShape(0).GetTexture().size
    camera = manager.AddCamera(
        ground,
        update_rate,
        offset_pose,
        width,
        height,
        exposure_time,
    )
    camera.SetName("camera")
    camera.SetLag(0)
    camera.SetCollectionWindow(0)

    
    update_rate = 5
    offset_pose = ch.ChFramed(ch.ChVector3d(0, 0, 1), ch.QuatFromAngleAxis(0, ch.ChVector3d(0, 1, 0)))
    lidar = manager.AddLidar(
        ground,
        update_rate,
        offset_pose,
        20,  
        20,  
        10,  
        5e-3,  
    )
    lidar.SetName("lidar")
    lidar.SetLag(0)
    lidar.SetCollectionWindow(0)

    
    gps = manager.AddGPS(
        ground,
        update_rate,
        offset_pose,
        100,  
        4,    
    )
    gps.SetName("gps")
    gps.SetLag(0)
    gps.SetCollectionWindow(0)

    
    noise_model_none = sens.ChNoiseNone()
    acc = manager.AddAccelerometer(ground, update_rate, offset_pose, noise_model_none)
    acc.SetName("accelerometer")
    acc.SetLag(0)
    acc.SetCollectionWindow(0)

    gyro = manager.AddGyroscope(ground, update_rate, offset_pose, noise_model_none)
    gyro.SetName("gyroscope")
    gyro.SetLag(0)
    gyro.SetCollectionWindow(0)

    
    magnetometer = manager.AddMagnetometer(ground, update_rate, offset_pose, noise_model_none)
    magnetometer.SetName("magnetometer")
    magnetometer.SetLag(0)
    magnetometer.SetCollectionWindow(0)

    
    ros_manager = chros.ChROSPythonManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler())
    ros_manager.RegisterHandler(chros.ChROSCameraHandler(camera, "~/output/camera/data/image", 100))
    ros_manager.RegisterHandler(chros.ChROSLidarHandler(lidar, "~/output/lidar/data/pointcloud", 100))
    ros_manager.RegisterHandler(chros.ChROSGPSHandler(gps, "~/output/gps/data", 100))
    acc_topic = "~/output/accelerometer/data"
    gyro_topic = "~/output/gyroscope/data"
    magnetometer_topic = "~/output/magnetometer/data"
    ros_manager.RegisterHandler(chros.ChROSIMUAccelerometerHandler(acc, acc_topic, 100))
    ros_manager.RegisterHandler(chros.ChROSIMUGyroscopeHandler(gyro, gyro_topic, 100))
    ros_manager.RegisterHandler(chros.ChROSIMUMagnetometerHandler(magnetometer, magnetometer_topic, 100))
    ros_manager.Initialize()

    
    time = 0
    time_step = 1e-3
    time_end = 100

    
    buffer = []

    
    fps = 0

    
    start_time = time.time()

    while time < time_end:
        
        manager.Update()
        ros_manager.Update()

        
        for cam in manager.GetCameras():
            if cam.GetCollectionWindow() == 0:
                img = cam.GetMostRecentImage()
                if img.HasData():
                    buffer.append(img)

        
        system.DoStepDynamics(time_step)

        
        time = system.GetChTime()

        
        time.sleep(max(0, time_step - (time - start_time) / fps))

        
        if fps == 0:
            fps = 1.0 / (time - start_time)
        else:
            fps = 1.0 / (time - start_time) * 30

    return 0


if __name__ == "__main__":
    main()