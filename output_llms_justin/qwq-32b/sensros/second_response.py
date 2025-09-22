import pychrono as ch
import pychrono.sensor as sens
import pychrono.ros as chros

def main():
    # Create the Chrono simulation system.
    sys = ch.ChSystemNSC()

    # Add a mesh object to the simulation for visual interest.
    mmesh = ch.ChTriangleMeshConnected()
    # Load and transform a 3D mesh of a vehicle chassis.
    mmesh.LoadWavefrontMesh(ch.GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"), False, True)
    mmesh.Transform(ch.ChVectorD(0, 0, 0), ch.ChMatrix33D.Identity())  # Fixed transform parameters

    # Create a visual shape from the mesh.
    trimesh_shape = ch.ChVisualShapeTriangleMesh()
    trimesh_shape.SetMesh(mmesh)
    trimesh_shape.SetName("HMMWV Chassis Mesh")
    trimesh_shape.SetMutable(False)

    # Create a body for the mesh and add it to the simulation.
    mesh_body = ch.ChBody()
    mesh_body.SetPos(ch.ChVectorD(0, 0, 0))
    mesh_body.AddVisualShape(trimesh_shape)
    mesh_body.SetFixed(True)  # Corrected to static
    sys.Add(mesh_body)

    # Create a ground body to attach sensors.
    ground_body = ch.ChBodyEasyBox(1, 1, 1, 1000, False, False)  # Density 1000 kg/m³
    ground_body.SetPos(ch.ChVectorD(0, 0, 0))
    ground_body.SetFixed(False)  # Now dynamic due to non-zero mass
    sys.Add(ground_body)

    # Create the sensor manager.
    sens_manager = sens.ChSensorManager(sys)

    # Add point lights to the scene for better visualization.
    intensity = 1.0
    sens_manager.scene.AddPointLight(ch.ChVectorF(2, 2.5, 100), ch.ChColor(intensity, intensity, intensity), 500.0)
    sens_manager.scene.AddPointLight(ch.ChVectorF(9, 2.5, 100), ch.ChColor(intensity, intensity, intensity), 500.0)
    sens_manager.scene.AddPointLight(ch.ChVectorF(16, 2.5, 100), ch.ChColor(intensity, intensity, intensity), 500.0)
    sens_manager.scene.AddPointLight(ch.ChVectorF(23, 2.5, 100), ch.ChColor(intensity, intensity, intensity), 500.0)

    # Create and configure a camera sensor.
    offset_pose = ch.ChFrameD(
        ch.ChVectorD(-8, 0, 2),
        ch.Q_from_AngAxis(0.2, ch.ChVectorD(0, 1, 0))
    )
    cam = sens.ChCameraSensor(ground_body, offset_pose, 30, 1280, 720, 1.408)
    cam.PushFilter(sens.ChFilterVisualize(1280, 720, "camera_visualization"))
    cam.PushFilter(sens.ChFilterRGBA8Access())
    cam.SetName("camera")
    sens_manager.AddSensor(cam)

    # Create and configure a 3D Lidar sensor with corrected parameters.
    lidar = sens.ChLidarSensor(
        ground_body,
        ch.ChVectorD(0, 0, 0),
        5.0,  # Range
        90.0 * ch.CH_DEG_TO_RAD,  # Horizontal FOV (converted to radians)
        30.0 * ch.CH_DEG_TO_RAD,  # Vertical FOV (adjusted for realism)
        ch.CH_PI / 180,  # Angular resolution azimuth (1 degree)
        ch.CH_PI / 180,  # Angular resolution elevation (1 degree)
        -ch.CH_PI / 6,  # Min elevation angle
        ch.CH_PI / 6,  # Max elevation angle
        0.0,  # Noise
        offset_pose,
        False  # 3D mode
    )
    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    lidar.PushFilter(sens.ChFilterXYZIAccess())
    lidar.PushFilter(sens.ChFilterVisualizePointCloud(1280, 720, "lidar3d_visualization"))
    lidar.SetName("lidar3d")
    sens_manager.AddSensor(lidar)

    # Create and configure a 2D Lidar sensor.
    lidar2d = sens.ChLidarSensor(
        ground_body,
        ch.ChVectorD(0, 0, 0),
        50.0,  # Increased range for 2D
        180.0 * ch.CH_DEG_TO_RAD,  # Full horizontal FOV
        0.0,  # Vertical FOV (2D)
        ch.CH_PI / 180,  # Angular resolution (1 degree)
        0.0,  # Unused elevation parameters
        0.0,
        0.0,
        0.0,  # No noise
        offset_pose,
        True  # 2D mode
    )
    lidar2d.PushFilter(sens.ChFilterDIAccess())
    lidar2d.PushFilter(sens.ChFilterPCfromDepth())
    lidar2d.PushFilter(sens.ChFilterXYZIAccess())
    lidar2d.PushFilter(sens.ChFilterVisualizePointCloud(1280, 720, "lidar2d_visualization"))
    lidar2d.SetName("lidar2d")
    sens_manager.AddSensor(lidar2d)

    # Create and configure a GPS sensor.
    noise_model_none = sens.ChNoiseNone()
    gps_reference = ch.ChVectorD(-89.4, 433.07, 260.)
    gps = sens.ChGPSSensor(ground_body, 10, offset_pose, gps_reference, noise_model_none)
    gps.PushFilter(sens.ChFilterGPSAccess())
    gps.SetName("gps")
    sens_manager.AddSensor(gps)

    # Create and configure an accelerometer sensor.
    acc = sens.ChAccelerometerSensor(ground_body, 100, offset_pose, noise_model_none)
    acc.PushFilter(sens.ChFilterAccelAccess())
    acc.SetName("accelerometer")
    sens_manager.AddSensor(acc)

    # Create and configure a gyroscope sensor.
    gyro = sens.ChGyroscopeSensor(ground_body, 100, offset_pose, noise_model_none)
    gyro.PushFilter(sens.ChFilterGyroAccess())
    gyro.SetName("gyroscope")
    sens_manager.AddSensor(gyro)

    # Create and configure a magnetometer sensor.
    mag = sens.ChMagnetometerSensor(ground_body, 100, offset_pose, noise_model_none, gps_reference)
    mag.PushFilter(sens.ChFilterMagnetAccess())
    mag.SetName("magnetometer")
    sens_manager.AddSensor(mag)

    # Update the sensor manager to initialize sensors.
    sens_manager.Update()

    # Create the ROS manager and register handlers for the sensors.
    ros_manager = chros.ChROSPythonManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler())

    # Register handlers for each sensor with specified topics.
    ros_manager.RegisterHandler(chros.ChROSCameraHandler(cam.GetUpdateRate()//4, cam, "~/output/camera/data/image"))
    ros_manager.RegisterHandler(chros.ChROSLidarHandler(lidar, "~/output/lidar3d/data/pointcloud"))
    ros_manager.RegisterHandler(chros.ChROSLidarHandler(lidar2d, "~/output/lidar2d/data/scan"))  # New ROS handler for 2D Lidar
    ros_manager.RegisterHandler(chros.ChROSGPSHandler(gps, "~/output/gps/data"))
    ros_manager.RegisterHandler(chros.ChROSAccelerometerHandler(acc, "~/output/accelerometer/data"))
    ros_manager.RegisterHandler(chros.ChROSGyroscopeHandler(gyro, "~/output/gyroscope/data"))
    ros_manager.RegisterHandler(chros.ChROSMagnetometerHandler(mag, "~/output/magnetometer/data"))

    # Create and configure an IMU handler.
    imu_handler = chros.ChROSIMUHandler(100, "~/output/imu/data")
    imu_handler.SetAccelerometerHandler(acc)
    imu_handler.SetGyroscopeHandler(gyro)
    imu_handler.SetMagnetometerHandler(mag)
    ros_manager.RegisterHandler(imu_handler)

    ros_manager.Initialize()

    # Run the simulation loop.
    time_step = 1e-3
    time_end = 100

    # Apply rotational velocity to the ground body for sensor motion.
    ground_body.SetWvel_par(ch.ChVectorD(0, 0, 0.1))

    while sys.GetChTime() < time_end:
        time = sys.GetChTime()

        sens_manager.Update()
        if not ros_manager.Update(time, time_step):
            break

        sys.DoStepDynamics(time_step)

if __name__ == "__main__":
    main()