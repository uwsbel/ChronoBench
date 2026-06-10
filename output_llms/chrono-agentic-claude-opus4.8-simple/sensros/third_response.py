import os
import numpy as np
import pychrono as chrono
import pychrono.sensor as sens
import pychrono.ros as chros


def main():
    sys = chrono.ChSystemNSC()                                         # NSC system (truth: sensros uses NSC)
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))   # g = 9.81, Z-up

    # --- visual-only HMMWV chassis mesh body (for the camera to look at) ---
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())               # ensure data path for the mesh asset
    mmesh = chrono.ChTriangleMeshConnected()                           # triangle mesh container
    mmesh.LoadWavefrontMesh(chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"), False, True)
    mmesh.Transform(chrono.ChVector3d(0, 0, 0), chrono.ChMatrix33d(1))  # no extra scale
    mesh_shape = chrono.ChVisualShapeTriangleMesh()                    # visual shape from the mesh
    mesh_shape.SetMesh(mmesh)
    mesh_shape.SetMutable(False)                                       # static mesh (no per-step rebuild)
    mesh_body = chrono.ChBody()                                        # visual-only body, mass 0
    mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))
    mesh_body.SetMass(0)
    mesh_body.SetFixed(True)                                           # the looks body is fixed
    mesh_body.AddVisualShape(mesh_shape, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
    sys.Add(mesh_body)

    # --- ground body that CARRIES all the sensors; it is the one that moves ---
    ground_body = chrono.ChBodyEasyBox(1, 1, 1, 1000, False, False)    # 1x1x1, no collide / no visual
    ground_body.SetPos(chrono.ChVector3d(0, 0, 0))
    ground_body.SetFixed(True)                                         # fixed but spun via SetAngVelParent
    sys.Add(ground_body)

    # === sensor manager + scene lighting (camera needs light) ===
    sens_manager = sens.ChSensorManager(sys)                           # manager owns all sensors
    intensity = 1.0
    sens_manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    sens_manager.scene.AddPointLight(chrono.ChVector3f(9, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    sens_manager.scene.AddPointLight(chrono.ChVector3f(16, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    sens_manager.scene.AddPointLight(chrono.ChVector3f(23, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)

    # shared offset pose: behind & above the body, slightly tilted
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(-8, 0, 2),
        chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0)),
    )

    # --- camera sensor (30 Hz) ---
    cam = sens.ChCameraSensor(ground_body, 30, offset_pose, 1280, 720, 1.408)  # w, h, hfov(rad)
    cam.SetName("Camera Sensor")
    cam.SetLag(0)                                                      # truth: lag = 0
    cam.SetCollectionWindow(0)                                         # camera exposure window = 0
    cam.PushFilter(sens.ChFilterVisualize(1280, 720, "RGB Camera"))    # live preview
    cam.PushFilter(sens.ChFilterRGBA8Access())                         # host access to RGBA8 buffer
    cam.PushFilter(sens.ChFilterSave("cam/rgb/"))                      # SAVE color PNGs
    sens_manager.AddSensor(cam)

    # --- lidar sensor (5 Hz, 3D) ---
    lidar_pose = chrono.ChFramed(chrono.ChVector3d(-8, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
    lidar = sens.ChLidarSensor(
        ground_body,
        5.0,                                                          # update_rate Hz
        lidar_pose,
        800,                                                          # horizontal samples
        300,                                                          # vertical samples
        2 * chrono.CH_PI,                                             # horizontal fov (360 deg)
        chrono.CH_PI / 12,                                            # max vert angle
        -chrono.CH_PI / 6,                                            # min vert angle
        100.0,                                                        # max range
        sens.LidarBeamShape_RECTANGULAR,
        2,                                                            # sample radius
        0.003,                                                        # vert divergence
        0.003,                                                        # hori divergence
        sens.LidarReturnMode_STRONGEST_RETURN,
    )
    lidar.SetName("Lidar Sensor")
    lidar.SetLag(0)
    lidar.SetCollectionWindow(1.0 / 5.0)                              # lidar collection window = 1/rate
    lidar.PushFilter(sens.ChFilterVisualize(800, 300, "Raw Lidar Depth"))
    lidar.PushFilter(sens.ChFilterDIAccess())                         # depth+intensity access
    lidar.PushFilter(sens.ChFilterPCfromDepth())                      # depth -> XYZI point cloud
    lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
    lidar.PushFilter(sens.ChFilterXYZIAccess())                       # XYZI access
    sens_manager.AddSensor(lidar)

    # --- 2D lidar sensor (single-row scan, 5 Hz) ---
    lidar2d = sens.ChLidarSensor(
        ground_body,
        5.0,                                                          # update_rate Hz
        lidar_pose,
        800,                                                          # horizontal samples
        1,                                                            # vertical samples = 1 (2D scan)
        2 * chrono.CH_PI,                                             # horizontal fov (360 deg)
        0.0,                                                          # max vert angle = 0 (single row)
        0.0,                                                          # min vert angle = 0 (single row)
        100.0,                                                        # max range
        sens.LidarBeamShape_RECTANGULAR,
        2,                                                            # sample radius
        0.003,                                                        # vert divergence
        0.003,                                                        # hori divergence
        sens.LidarReturnMode_STRONGEST_RETURN,
    )
    lidar2d.SetName("2D Lidar Sensor")
    lidar2d.SetLag(0)
    lidar2d.SetCollectionWindow(1.0 / 5.0)                           # collection window = 1/rate
    lidar2d.PushFilter(sens.ChFilterVisualize(800, 1, "Raw 2D Lidar Depth"))   # named for visualization
    lidar2d.PushFilter(sens.ChFilterDIAccess())                      # depth+intensity access
    lidar2d.PushFilter(sens.ChFilterPCfromDepth())                   # depth -> XYZI point cloud
    lidar2d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "2D Lidar Point Cloud"))
    lidar2d.PushFilter(sens.ChFilterXYZIAccess())                    # XYZI access
    sens_manager.AddSensor(lidar2d)

    # --- GPS sensor (10 Hz) ---
    gps = sens.ChGPSSensor(
        ground_body, 10, offset_pose,
        chrono.ChVector3d(-89.400, 43.070, 260.0),                    # reference lon/lat/alt
        sens.ChNoiseNone(),
    )
    gps.SetName("GPS Sensor")
    gps.SetLag(0)
    gps.SetCollectionWindow(0)
    gps.PushFilter(sens.ChFilterGPSAccess())                          # GPS access
    sens_manager.AddSensor(gps)

    # --- accelerometer / gyroscope / magnetometer (100 Hz each) ---
    acc = sens.ChAccelerometerSensor(ground_body, 100, offset_pose, sens.ChNoiseNone())
    acc.SetName("Accelerometer Sensor")
    acc.SetLag(0)
    acc.SetCollectionWindow(0)
    acc.PushFilter(sens.ChFilterAccelAccess())                        # accel access
    sens_manager.AddSensor(acc)

    gyro = sens.ChGyroscopeSensor(ground_body, 100, offset_pose, sens.ChNoiseNone())
    gyro.SetName("Gyroscope Sensor")
    gyro.SetLag(0)
    gyro.SetCollectionWindow(0)
    gyro.PushFilter(sens.ChFilterGyroAccess())                        # gyro access
    sens_manager.AddSensor(gyro)

    mag = sens.ChMagnetometerSensor(ground_body, 100, offset_pose, sens.ChNoiseNone(),
                                    chrono.ChVector3d(-89.400, 43.070, 260.0))  # GPS reference for mag
    mag.SetName("Magnetometer Sensor")
    mag.SetLag(0)
    mag.SetCollectionWindow(0)
    mag.PushFilter(sens.ChFilterMagnetAccess())                       # magnetometer access
    sens_manager.AddSensor(mag)

    # === ROS manager + handlers (scored core) ===
    ros_manager = chros.ChROSPythonManager()                          # Python manager (hosts all handlers)
    ros_manager.RegisterHandler(chros.ChROSClockHandler())            # clock FIRST -> /clock

    cam_handler = chros.ChROSCameraHandler(cam.GetUpdateRate() / 4, cam, "~/output/camera/data/image")
    ros_manager.RegisterHandler(cam_handler)

    lidar_handler = chros.ChROSLidarHandler(lidar, "~/output/lidar/data/pointcloud")
    ros_manager.RegisterHandler(lidar_handler)

    lidar2d_handler = chros.ChROSLidarHandler(                        # 2D scan published as LASER_SCAN
        lidar2d, "~/output/lidar2d/data/scan",
        chros.ChROSLidarHandlerMessageType_LASER_SCAN,
    )
    ros_manager.RegisterHandler(lidar2d_handler)

    gps_handler = chros.ChROSGPSHandler(gps, "~/output/gps/data")
    ros_manager.RegisterHandler(gps_handler)

    acc_handler = chros.ChROSAccelerometerHandler(acc, "~/output/accelerometer/data")
    gyro_handler = chros.ChROSGyroscopeHandler(gyro, "~/output/gyroscope/data")
    mag_handler = chros.ChROSMagnetometerHandler(mag, "~/output/magnetometer/data")
    ros_manager.RegisterHandler(acc_handler)
    ros_manager.RegisterHandler(gyro_handler)
    ros_manager.RegisterHandler(mag_handler)

    imu_handler = chros.ChROSIMUHandler(100, "~/output/imu/data")     # fused IMU message
    imu_handler.SetAccelerometerHandler(acc_handler)
    imu_handler.SetGyroscopeHandler(gyro_handler)
    imu_handler.SetMagnetometerHandler(mag_handler)
    ros_manager.RegisterHandler(imu_handler)

    ros_manager.Initialize()                                          # init ONCE after all registration

    # === simulation loop (headless, time-bounded — family B) ===
    time_step = 1e-3                                                  # 1 ms step
    time_end = 100                                                    # scored end time
    time = 0

    ground_body.SetAngVelParent(chrono.ChVector3d(0, 0, 0.1))         # spin so sensors see motion


    while time < time_end:
        time = sys.GetChTime()                                        # sim clock
        sens_manager.Update()                                         # pump ALL sensors (ChFilterSave writes PNGs)
        if not ros_manager.Update(time, time_step):                  # publish to ROS; break if shut down
            break
        sys.DoStepDynamics(time_step)                                # advance physics

        rgba = cam.GetMostRecentRGBA8Buffer()                        # scored buffer access
        if rgba.HasData():
            print("Camera buffer: ", rgba.Width, "x", rgba.Height)
        xyzi = lidar.GetMostRecentXYZIBuffer()                       # scored lidar access
        if xyzi.HasData():
            print("Lidar points max: ", np.max(xyzi.GetXYZIData()))


if __name__ == "__main__":
    main()
