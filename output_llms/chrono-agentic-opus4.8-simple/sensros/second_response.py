import os
import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.ros as chros


def main():
    sys = chrono.ChSystemNSC()                                        # NSC system
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # contact/collision present

    # ground / mesh body the sensors ride on; spun so the sensors see motion
    mphysicalSystem = sys
    ground_body = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, False)     # 1 m cube carrier
    ground_body.SetPos(chrono.ChVector3d(0, 0, 0))                     # at the origin
    ground_body.SetFixed(True)                                        # held in place, only rotates
    sys.Add(ground_body)
    ground_body.SetAngVelParent(chrono.ChVector3d(0, 0, 0.3))         # spin about Z so sensors sweep

    # sensor manager + scene lighting (lights matter for the camera only)
    manager = sens.ChSensorManager(sys)                               # owns every sensor
    intensity = 1.0
    manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100),
                                chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(chrono.ChVector3f(9, 2.5, 100),
                                chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(chrono.ChVector3f(16, 2.5, 100),
                                chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(chrono.ChVector3f(23, 2.5, 100),
                                chrono.ChColor(intensity, intensity, intensity), 500.0)

    # ---- camera sensor on the body ----
    cam_offset_pose = chrono.ChFramed(chrono.ChVector3d(-8, 0, 1),
                                      chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
    cam = sens.ChCameraSensor(ground_body, 30, cam_offset_pose, 1280, 720, 1.408)   # 30 Hz RGB cam
    cam.SetName("Camera Sensor")
    cam.SetLag(0)
    cam.SetCollectionWindow(0)
    cam.PushFilter(sens.ChFilterVisualize(1280, 720, "Camera"))       # live RGB preview
    cam.PushFilter(sens.ChFilterRGBA8Access())                        # host access to RGBA8
    cam.PushFilter(sens.ChFilterSave("cam/rgb/"))                     # save color PNGs
    manager.AddSensor(cam)

    # ---- 2D Lidar sensor on the body ----
    horizontal_samples = 800                                         # horizontal beams
    vertical_samples = 1                                             # 2D lidar => single layer
    lidar_offset_pose = chrono.ChFramed(chrono.ChVector3d(-8, 0, 1),
                                        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
    lidar2d = sens.ChLidarSensor(
        ground_body,                                                # body the lidar is attached to
        5.0,                                                        # update_rate (Hz)
        lidar_offset_pose,                                          # offset pose
        horizontal_samples,                                         # h_samples
        vertical_samples,                                           # v_samples = 1 (2D)
        2 * chrono.CH_PI,                                           # horizontal_fov (full sweep)
        0.0,                                                        # max_vert_angle = 0 (2D)
        0.0,                                                        # min_vert_angle = 0 (2D)
        100.0,                                                      # max_range
        sens.LidarBeamShape_RECTANGULAR,                           # beam shape
        2,                                                          # sample_radius
        0.003,                                                      # vert divergence_angle
        0.003,                                                      # hori divergence_angle
        sens.LidarReturnMode_STRONGEST_RETURN,
    )
    lidar2d.SetName("2D Lidar Sensor")
    lidar2d.SetLag(0)
    lidar2d.SetCollectionWindow(1.0 / 5.0)                          # collection window = 1/update_rate
    # filter chain — named visualize filters for the 2D lidar
    lidar2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "2D Lidar Depth"))
    lidar2d.PushFilter(sens.ChFilterDIAccess())                     # host access to depth+intensity
    lidar2d.PushFilter(sens.ChFilterPCfromDepth())                 # depth -> XYZI point cloud
    lidar2d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "2D Lidar Point Cloud"))
    lidar2d.PushFilter(sens.ChFilterXYZIAccess())                  # host access to XYZI
    manager.AddSensor(lidar2d)

    # ---- GPS sensor ----
    gps_offset_pose = chrono.ChFramed(chrono.ChVector3d(0, 0, 0),
                                      chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
    gps = sens.ChGPSSensor(ground_body, 10, gps_offset_pose,
                           chrono.ChVector3d(-89.400, 43.070, 260.0), sens.ChNoiseNone())
    gps.SetName("GPS Sensor")
    gps.SetLag(0)
    gps.SetCollectionWindow(0)
    gps.PushFilter(sens.ChFilterGPSAccess())                        # host access to GPS
    manager.AddSensor(gps)

    # ---- IMU triplet: accelerometer + gyroscope + magnetometer ----
    imu_offset_pose = chrono.ChFramed(chrono.ChVector3d(0, 0, 0),
                                      chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
    acc = sens.ChAccelerometerSensor(ground_body, 100, imu_offset_pose, sens.ChNoiseNone())
    acc.SetName("Accelerometer")
    acc.SetLag(0)
    acc.SetCollectionWindow(0)
    acc.PushFilter(sens.ChFilterAccelAccess())                     # host access to accel
    manager.AddSensor(acc)

    gyro = sens.ChGyroscopeSensor(ground_body, 100, imu_offset_pose, sens.ChNoiseNone())
    gyro.SetName("Gyroscope")
    gyro.SetLag(0)
    gyro.SetCollectionWindow(0)
    gyro.PushFilter(sens.ChFilterGyroAccess())                     # host access to gyro
    manager.AddSensor(gyro)

    mag = sens.ChMagnetometerSensor(ground_body, 100, imu_offset_pose, sens.ChNoiseNone(),
                                    chrono.ChVector3d(-89.400, 43.070, 260.0))
    mag.SetName("Magnetometer")
    mag.SetLag(0)
    mag.SetCollectionWindow(0)
    mag.PushFilter(sens.ChFilterMagnetAccess())                    # host access to magnet
    manager.AddSensor(mag)

    # ---- ROS manager + handlers (scored core) ----
    ros_manager = chros.ChROSPythonManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler())          # /clock first

    cam_handler = chros.ChROSCameraHandler(cam.GetUpdateRate() / 4, cam, "~/output/camera/data/image")
    ros_manager.RegisterHandler(cam_handler)

    # 2D Lidar ROS handler -> publish a laser scan to ~/output/lidar2d/data/scan
    lidar2d_handler = chros.ChROSLidarHandler(lidar2d, "~/output/lidar2d/data/scan",
                                              chros.ChROSLidarHandlerMessageType_LASER_SCAN)
    ros_manager.RegisterHandler(lidar2d_handler)

    gps_handler = chros.ChROSGPSHandler(gps, "~/output/gps/data")
    ros_manager.RegisterHandler(gps_handler)

    acc_handler = chros.ChROSAccelerometerHandler(acc, "~/output/accelerometer/data")
    ros_manager.RegisterHandler(acc_handler)
    gyro_handler = chros.ChROSGyroscopeHandler(gyro, "~/output/gyroscope/data")
    ros_manager.RegisterHandler(gyro_handler)
    mag_handler = chros.ChROSMagnetometerHandler(mag, "~/output/magnetometer/data")
    ros_manager.RegisterHandler(mag_handler)

    imu_handler = chros.ChROSIMUHandler(100, "~/output/imu/data")    # fused IMU
    imu_handler.SetAccelerometerHandler(acc_handler)
    imu_handler.SetGyroscopeHandler(gyro_handler)
    imu_handler.SetMagnetometerHandler(mag_handler)
    ros_manager.RegisterHandler(imu_handler)

    ros_manager.Initialize()                                        # once, after all handlers

    # ---- run loop ----
    time = 0.0
    time_step = 2e-3                                                # integration step
    sim_end = 20.0                                                  # simulated seconds
    render_fps = 30.0
    render_every = max(1, round(1.0 / (render_fps * time_step)))   # untagged cadence constant
    while time < sim_end:
        manager.Update()                                           # pump sensors -> fill buffers
        if not ros_manager.Update(time, time_step):                # publish to ROS
            break
        sys.DoStepDynamics(time_step)                              # advance physics
        time = sys.GetChTime()


if __name__ == "__main__":
    main()
