import pychrono as ch
import pychrono.sensor as sens
import pychrono.ros as chros


def main():
    # Create the Chrono simulation system.
    sys = ch.ChSystemNSC()

    # Disable gravity so the sensor carrier body can rotate in place without falling.
    sys.SetGravitationalAcceleration(ch.ChVector3d(0, 0, 0))

    # -------------------------------------------------------------------------
    # Visual mesh body
    # -------------------------------------------------------------------------
    mmesh = ch.ChTriangleMeshConnected()
    mmesh.LoadWavefrontMesh(
        ch.GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"),
        False,
        True
    )
    mmesh.Transform(ch.ChVector3d(0, 0, 0), ch.ChMatrix33d(1))

    trimesh_shape = ch.ChVisualShapeTriangleMesh()
    trimesh_shape.SetMesh(mmesh)
    trimesh_shape.SetName("HMMWV Chassis Mesh")
    trimesh_shape.SetMutable(False)

    mesh_body = ch.ChBody()
    mesh_body.SetPos(ch.ChVector3d(0, 0, 0))
    mesh_body.AddVisualShape(trimesh_shape)

    # This body is only for visualization; keep it fixed.
    mesh_body.SetFixed(True)
    sys.Add(mesh_body)

    # -------------------------------------------------------------------------
    # Sensor carrier body
    # -------------------------------------------------------------------------
    # Use a dynamic body with valid mass/inertia so angular velocity is integrated.
    ground_body = ch.ChBodyEasyBox(1, 1, 1, 1.0, False, False)
    ground_body.SetPos(ch.ChVector3d(0, 0, 0))
    ground_body.SetFixed(False)
    sys.Add(ground_body)

    # Apply rotational velocity to the sensor carrier body.
    ground_body.SetAngVelParent(ch.ChVector3d(0, 0, 0.1))

    # -------------------------------------------------------------------------
    # Sensor manager and lighting
    # -------------------------------------------------------------------------
    sens_manager = sens.ChSensorManager(sys)

    intensity = 1.0
    sens_manager.scene.AddPointLight(
        ch.ChVector3f(2, 2.5, 100),
        ch.ChColor(intensity, intensity, intensity),
        500.0
    )
    sens_manager.scene.AddPointLight(
        ch.ChVector3f(9, 2.5, 100),
        ch.ChColor(intensity, intensity, intensity),
        500.0
    )
    sens_manager.scene.AddPointLight(
        ch.ChVector3f(16, 2.5, 100),
        ch.ChColor(intensity, intensity, intensity),
        500.0
    )
    sens_manager.scene.AddPointLight(
        ch.ChVector3f(23, 2.5, 100),
        ch.ChColor(intensity, intensity, intensity),
        500.0
    )

    # Shared sensor offset pose.
    offset_pose = ch.ChFramed(
        ch.ChVector3d(-8, 0, 2),
        ch.QuatFromAngleAxis(0.2, ch.ChVector3d(0, 1, 0))
    )

    # -------------------------------------------------------------------------
    # Camera sensor
    # -------------------------------------------------------------------------
    cam = sens.ChCameraSensor(
        ground_body,
        30,
        offset_pose,
        1280,
        720,
        1.408
    )
    cam.SetName("camera")
    cam.PushFilter(sens.ChFilterVisualize(1280, 720, "Camera Visualization"))
    cam.PushFilter(sens.ChFilterRGBA8Access())
    sens_manager.AddSensor(cam)

    # -------------------------------------------------------------------------
    # 3D Lidar sensor
    # -------------------------------------------------------------------------
    lidar = sens.ChLidarSensor(
        ground_body,
        5.0,
        offset_pose,
        90,
        300,
        2 * ch.CH_PI,
        ch.CH_PI / 12,
        -ch.CH_PI / 6,
        100.0,
        0
    )
    lidar.SetName("lidar")
    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    lidar.PushFilter(sens.ChFilterXYZIAccess())
    lidar.PushFilter(
        sens.ChFilterVisualizePointCloud(
            1280,
            720,
            1,
            "3D Lidar Point Cloud Visualization"
        )
    )
    sens_manager.AddSensor(lidar)

    # -------------------------------------------------------------------------
    # 2D Lidar sensor
    # -------------------------------------------------------------------------
    lidar2d = sens.ChLidarSensor(
        ground_body,
        10.0,           # update rate [Hz]
        offset_pose,
        1080,           # horizontal samples
        1,              # one vertical channel for 2D scan
        2 * ch.CH_PI,   # horizontal field of view
        0.0,            # max vertical angle
        0.0,            # min vertical angle
        100.0,          # max range
        0               # lag
    )
    lidar2d.SetName("lidar2d")
    lidar2d.PushFilter(sens.ChFilterDIAccess())
    lidar2d.PushFilter(sens.ChFilterPCfromDepth())
    lidar2d.PushFilter(sens.ChFilterXYZIAccess())
    lidar2d.PushFilter(
        sens.ChFilterVisualizePointCloud(
            1280,
            720,
            2,
            "2D Lidar Point Cloud Visualization"
        )
    )
    sens_manager.AddSensor(lidar2d)

    # -------------------------------------------------------------------------
    # GPS sensor
    # -------------------------------------------------------------------------
    noise_model_none = sens.ChNoiseNone()
    gps_reference = ch.ChVector3d(-89.4, 433.07, 260.0)

    gps = sens.ChGPSSensor(
        ground_body,
        10,
        offset_pose,
        gps_reference,
        noise_model_none
    )
    gps.SetName("gps")
    gps.PushFilter(sens.ChFilterGPSAccess())
    sens_manager.AddSensor(gps)

    # -------------------------------------------------------------------------
    # Accelerometer sensor
    # -------------------------------------------------------------------------
    acc = sens.ChAccelerometerSensor(
        ground_body,
        100,
        offset_pose,
        noise_model_none
    )
    acc.SetName("accelerometer")
    acc.PushFilter(sens.ChFilterAccelAccess())
    sens_manager.AddSensor(acc)

    # -------------------------------------------------------------------------
    # Gyroscope sensor
    # -------------------------------------------------------------------------
    gyro = sens.ChGyroscopeSensor(
        ground_body,
        100,
        offset_pose,
        noise_model_none
    )
    gyro.SetName("gyroscope")
    gyro.PushFilter(sens.ChFilterGyroAccess())
    sens_manager.AddSensor(gyro)

    # -------------------------------------------------------------------------
    # Magnetometer sensor
    # -------------------------------------------------------------------------
    mag = sens.ChMagnetometerSensor(
        ground_body,
        100,
        offset_pose,
        noise_model_none,
        gps_reference
    )
    mag.SetName("magnetometer")
    mag.PushFilter(sens.ChFilterMagnetAccess())
    sens_manager.AddSensor(mag)

    # Initialize sensors.
    sens_manager.Update()

    # -------------------------------------------------------------------------
    # ROS manager and handlers
    # -------------------------------------------------------------------------
    ros_manager = chros.ChROSPythonManager()

    ros_manager.RegisterHandler(chros.ChROSClockHandler())

    ros_manager.RegisterHandler(
        chros.ChROSCameraHandler(
            cam.GetUpdateRate() / 4,
            cam,
            "~/output/camera/data/image"
        )
    )

    ros_manager.RegisterHandler(
        chros.ChROSLidarHandler(
            lidar.GetUpdateRate(),
            lidar,
            "~/output/lidar/data/pointcloud"
        )
    )

    # New ROS handler for the 2D lidar LaserScan output.
    ros_manager.RegisterHandler(
        chros.ChROSLidar2DHandler(
            lidar2d.GetUpdateRate(),
            lidar2d,
            "~/output/lidar2d/data/scan"
        )
    )

    ros_manager.RegisterHandler(
        chros.ChROSGPSHandler(
            gps.GetUpdateRate(),
            gps,
            "~/output/gps/data"
        )
    )

    acc_handler = chros.ChROSAccelerometerHandler(
        acc.GetUpdateRate(),
        acc,
        "~/output/accelerometer/data"
    )
    ros_manager.RegisterHandler(acc_handler)

    gyro_handler = chros.ChROSGyroscopeHandler(
        gyro.GetUpdateRate(),
        gyro,
        "~/output/gyroscope/data"
    )
    ros_manager.RegisterHandler(gyro_handler)

    mag_handler = chros.ChROSMagnetometerHandler(
        mag.GetUpdateRate(),
        mag,
        "~/output/magnetometer/data"
    )
    ros_manager.RegisterHandler(mag_handler)

    imu_handler = chros.ChROSIMUHandler(
        100,
        "~/output/imu/data"
    )
    imu_handler.SetAccelerometerHandler(acc_handler)
    imu_handler.SetGyroscopeHandler(gyro_handler)
    imu_handler.SetMagnetometerHandler(mag_handler)
    ros_manager.RegisterHandler(imu_handler)

    ros_manager.Initialize()

    # -------------------------------------------------------------------------
    # Simulation loop
    # -------------------------------------------------------------------------
    time_step = 1e-3
    time_end = 100.0

    while sys.GetChTime() < time_end:
        time = sys.GetChTime()

        sens_manager.Update()

        if not ros_manager.Update(time, time_step):
            break

        sys.DoStepDynamics(time_step)


if __name__ == "__main__":
    main()