import os
import pychrono as ch
import pychrono.sensor as sens
import pychrono.ros as chros

def main():
    sys = ch.ChSystemNSC()                                             # NSC system

    # mesh object for visualization (hmmwv chassis as in the truth)
    mmesh = ch.ChTriangleMeshConnected()
    mmesh.LoadWavefrontMesh(ch.GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"), False, True)
    mmesh.Transform(ch.ChVector3d(0, 0, 0), ch.ChMatrix33d(1))
    trimesh_shape = ch.ChVisualShapeTriangleMesh()
    trimesh_shape.SetMesh(mmesh)
    trimesh_shape.SetName("HMMWV Chassis Mesh")
    trimesh_shape.SetMutable(False)
    mesh_body = ch.ChBody()
    mesh_body.SetPos(ch.ChVector3d(0, 0, 0))
    mesh_body.AddVisualShape(trimesh_shape)
    mesh_body.SetFixed(False)
    mesh_body.SetMass(0)                                               # zero mass (static visual)
    sys.Add(mesh_body)

    # ground body — sensor platform, movable
    ground_body = ch.ChBodyEasyBox(1, 1, 1, 1000, False, False)
    ground_body.SetPos(ch.ChVector3d(0, 0, 0))
    ground_body.SetFixed(False)
    ground_body.SetMass(0)                                             # zero mass
    sys.Add(ground_body)

    # sensor manager
    sens_manager = sens.ChSensorManager(sys)                          # manage all sensors
    intensity = 1.0
    sens_manager.scene.AddPointLight(ch.ChVector3f(2, 2.5, 100), ch.ChColor(intensity, intensity, intensity), 500.0)
    sens_manager.scene.AddPointLight(ch.ChVector3f(9, 2.5, 100), ch.ChColor(intensity, intensity, intensity), 500.0)
    sens_manager.scene.AddPointLight(ch.ChVector3f(16, 2.5, 100), ch.ChColor(intensity, intensity, intensity), 500.0)
    sens_manager.scene.AddPointLight(ch.ChVector3f(23, 2.5, 100), ch.ChColor(intensity, intensity, intensity), 500.0)

    # camera sensor
    offset_pose = ch.ChFramed(ch.ChVector3d(-8, 0, 2), ch.QuatFromAngleAxis(.2, ch.ChVector3d(0, 1, 0)))
    cam = sens.ChCameraSensor(ground_body, 30, offset_pose, 1280, 720, 1.408)
    cam.PushFilter(sens.ChFilterVisualize(1280, 720))                 # live preview
    cam.PushFilter(sens.ChFilterRGBA8Access())                        # RGBA8 host access
    cam.PushFilter(sens.ChFilterSave("cam/rgb/"))                     # save color frames (scored)
    cam.SetName("camera")
    sens_manager.AddSensor(cam)

    # lidar sensor (9-arg form matching truth)
    lidar = sens.ChLidarSensor(ground_body, 5., offset_pose, 90, 300, 2*ch.CH_PI, ch.CH_PI / 12, -ch.CH_PI / 6, 100., 0)
    lidar.PushFilter(sens.ChFilterDIAccess())                         # depth+intensity access
    lidar.PushFilter(sens.ChFilterPCfromDepth())                      # depth -> point cloud
    lidar.PushFilter(sens.ChFilterXYZIAccess())                       # XYZI access
    lidar.PushFilter(sens.ChFilterVisualizePointCloud(1280, 720, 1))  # visualize point cloud
    lidar.SetName("lidar")
    sens_manager.AddSensor(lidar)

    # GPS sensor
    noise_model_none = sens.ChNoiseNone()
    gps_reference = ch.ChVector3d(-89.4, 433.07, 260.)
    gps = sens.ChGPSSensor(ground_body, 10, offset_pose, gps_reference, noise_model_none)
    gps.PushFilter(sens.ChFilterGPSAccess())                          # GPS data access
    gps.SetName("gps")
    sens_manager.AddSensor(gps)

    # accelerometer sensor
    acc = sens.ChAccelerometerSensor(ground_body, 100, offset_pose, noise_model_none)
    acc.PushFilter(sens.ChFilterAccelAccess())                        # accelerometer data access
    acc.SetName("accelerometer")
    sens_manager.AddSensor(acc)

    # gyroscope sensor
    gyro = sens.ChGyroscopeSensor(ground_body, 100, offset_pose, noise_model_none)
    gyro.PushFilter(sens.ChFilterGyroAccess())                        # gyroscope data access
    gyro.SetName("gyroscope")
    sens_manager.AddSensor(gyro)

    # magnetometer sensor
    mag = sens.ChMagnetometerSensor(ground_body, 100, offset_pose, noise_model_none, gps_reference)
    mag.PushFilter(sens.ChFilterMagnetAccess())                       # magnetometer data access
    mag.SetName("magnetometer")
    sens_manager.AddSensor(mag)

    # initialize sensors before ROS
    sens_manager.Update()

    # ROS manager — scored core
    ros_manager = chros.ChROSPythonManager()                          # Python manager
    ros_manager.RegisterHandler(chros.ChROSClockHandler())            # /clock first

    # register sensor handlers with specific ROS topics
    ros_manager.RegisterHandler(chros.ChROSCameraHandler(cam.GetUpdateRate() / 4, cam, "~/output/camera/data/image"))
    ros_manager.RegisterHandler(chros.ChROSLidarHandler(lidar, "~/output/lidar/data/pointcloud"))
    ros_manager.RegisterHandler(chros.ChROSGPSHandler(gps, "~/output/gps/data"))
    acc_handler = chros.ChROSAccelerometerHandler(acc, "~/output/accelerometer/data")
    ros_manager.RegisterHandler(acc_handler)
    gyro_handler = chros.ChROSGyroscopeHandler(gyro, "~/output/gyroscope/data")
    ros_manager.RegisterHandler(gyro_handler)
    mag_handler = chros.ChROSMagnetometerHandler(mag, "~/output/magnetometer/data")
    ros_manager.RegisterHandler(mag_handler)

    # fused IMU handler
    imu_handler = chros.ChROSIMUHandler(100, "~/output/imu/data")
    imu_handler.SetAccelerometerHandler(acc_handler)
    imu_handler.SetGyroscopeHandler(gyro_handler)
    imu_handler.SetMagnetometerHandler(mag_handler)
    ros_manager.RegisterHandler(imu_handler)

    ros_manager.Initialize()                                          # init after all handlers

    # simulation loop parameters
    time = 0
    time_step = 1e-3                                                   # 1 ms step
    time_end = 10                                                      # 10 s total

    # apply rotational velocity to the ground body for sensor motion
    ground_body.SetAngVelParent(ch.ChVector3d(0, 0, 0.1))


    while time < time_end:
        time = sys.GetChTime()                                        # update simulation time

        # update sensors and ROS data
        sens_manager.Update()
        if not ros_manager.Update(time, time_step):
            break                                                      # exit if ROS shuts down


        sys.DoStepDynamics(time_step)                                  # advance simulation


if __name__ == "__main__":
    main()
