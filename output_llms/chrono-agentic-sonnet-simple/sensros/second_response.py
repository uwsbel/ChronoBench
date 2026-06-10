import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens
import pychrono.ros as chros
import rclpy

def main():
    sys = chrono.ChSystemNSC()  # create the physics system
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  # g = 9.81 m/s^2
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # bullet collision

    # --- ground body (fixed reference) ---
    ground = chrono.ChBodyEasyBox(10, 10, 0.1, 1000, True, False)  # thin flat ground
    ground.SetPos(chrono.ChVector3d(0, 0, -0.1))                    # z = -0.1 to sit at z=0
    ground.SetFixed(True)
    ground.SetName("base_link")                                      # TF root frame
    sys.AddBody(ground)

    # --- spinning body that carries all sensors ---
    ground_body = chrono.ChBodyEasyBox(1, 1, 0.2, 1000, True, False)  # sensor platform
    ground_body.SetPos(chrono.ChVector3d(0, 0, 1.0))                   # 1 m above ground
    ground_body.SetFixed(False)
    ground_body.SetName("ground_body")
    sys.AddBody(ground_body)

    # fix the spinning body vertically but let it rotate
    link_rev = chrono.ChLinkLockLock()
    link_rev.Initialize(
        ground_body,
        ground,
        chrono.ChFramed(chrono.ChVector3d(0, 0, 1.0), chrono.QUNIT),
    )
    sys.AddLink(link_rev)

    # apply angular velocity about Z so sensors see motion
    ground_body.SetAngVelParent(chrono.ChVector3d(0, 0, 1.0))  # 1 rad/s spin

    # --- Irrlicht visualization ---
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("SensROS Demo — 2D Lidar")
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(3, -3, 3), chrono.ChVector3d(0, 0, 1))
    vis.AddTypicalLights()

    # --- sensor manager ---
    manager = sens.ChSensorManager(sys)
    intensity = 1.0
    manager.scene.AddPointLight(
        chrono.ChVector3f(2, 2.5, 100),
        chrono.ChColor(intensity, intensity, intensity),
        500.0,
    )
    manager.scene.AddPointLight(
        chrono.ChVector3f(9, 2.5, 100),
        chrono.ChColor(intensity, intensity, intensity),
        500.0,
    )

    # --- RGB camera sensor ---
    cam_offset = chrono.ChFramed(
        chrono.ChVector3d(-5, 0, 2),
        chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0)),
    )
    cam = sens.ChCameraSensor(
        ground_body,  # attach to spinning platform
        30,           # update_rate Hz
        cam_offset,
        1280, 720,
        1.408,        # horizontal FOV (rad)
    )
    cam.SetName("Camera Sensor")
    cam.SetLag(0)
    cam.SetCollectionWindow(0)
    cam.PushFilter(sens.ChFilterVisualize(1280, 720, "RGB Camera"))
    cam.PushFilter(sens.ChFilterRGBA8Access())
    cam.PushFilter(sens.ChFilterSave("cam/rgb/"))
    manager.AddSensor(cam)

    # --- 2D Lidar sensor (v_samples=1, vertical angles both 0) ---
    lidar_offset = chrono.ChFramed(
        chrono.ChVector3d(-12, 0, 1),
        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
    )
    lidar2d = sens.ChLidarSensor(
        ground_body,                         # attach to spinning platform
        5.0,                                 # update_rate Hz
        lidar_offset,                        # offset pose
        800,                                 # h_samples (horizontal resolution)
        1,                                   # v_samples = 1 → 2D lidar
        2 * chrono.CH_PI,                   # horizontal_fov (full 360 deg)
        0,                                   # max_vert_angle (0 for 2D)
        0,                                   # min_vert_angle (0 for 2D)
        100.0,                               # max_range (m)
        sens.LidarBeamShape_RECTANGULAR,     # beam shape
        2,                                   # sample_radius
        0.003,                               # vert divergence_angle
        0.003,                               # hori divergence_angle
        sens.LidarReturnMode_STRONGEST_RETURN,
    )
    lidar2d.SetName("2D Lidar Sensor")
    lidar2d.SetLag(0)
    lidar2d.SetCollectionWindow(1.0 / 5.0)  # collection window = 1/update_rate

    # lidar2d filter chain with names for visualization
    lidar2d.PushFilter(sens.ChFilterVisualize(800, 1, "Raw 2D Lidar Depth"))
    lidar2d.PushFilter(sens.ChFilterDIAccess())
    lidar2d.PushFilter(sens.ChFilterPCfromDepth())
    lidar2d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "2D Lidar Point Cloud"))
    lidar2d.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar2d)

    # --- GPS sensor ---
    gps_offset = chrono.ChFramed(
        chrono.ChVector3d(-8, 0, 1),
        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
    )
    gps = sens.ChGPSSensor(
        ground_body,
        10,           # update_rate Hz
        gps_offset,
        chrono.ChVector3d(-89.400, 43.070, 260.0),  # reference lat/lon/alt
        sens.ChNoiseNone(),
    )
    gps.SetName("GPS Sensor")
    gps.SetLag(0)
    gps.SetCollectionWindow(0)
    gps.PushFilter(sens.ChFilterGPSAccess())
    manager.AddSensor(gps)

    # --- Accelerometer ---
    acc_offset = chrono.ChFramed(
        chrono.ChVector3d(0, 0, 0),
        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
    )
    acc = sens.ChAccelerometerSensor(ground_body, 100, acc_offset, sens.ChNoiseNone())
    acc.SetName("Accelerometer Sensor")
    acc.SetLag(0)
    acc.SetCollectionWindow(0)
    acc.PushFilter(sens.ChFilterAccelAccess())
    manager.AddSensor(acc)

    # --- Gyroscope ---
    gyro_offset = chrono.ChFramed(
        chrono.ChVector3d(0, 0, 0),
        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
    )
    gyro = sens.ChGyroscopeSensor(ground_body, 100, gyro_offset, sens.ChNoiseNone())
    gyro.SetName("Gyroscope Sensor")
    gyro.SetLag(0)
    gyro.SetCollectionWindow(0)
    gyro.PushFilter(sens.ChFilterGyroAccess())
    manager.AddSensor(gyro)

    # --- Magnetometer ---
    mag_offset = chrono.ChFramed(
        chrono.ChVector3d(0, 0, 0),
        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
    )
    mag = sens.ChMagnetometerSensor(
        ground_body, 100, mag_offset,
        sens.ChNoiseNone(),
        chrono.ChVector3d(-89.400, 43.070, 260.0),  # GPS reference for mag
    )
    mag.SetName("Magnetometer Sensor")
    mag.SetLag(0)
    mag.SetCollectionWindow(0)
    mag.PushFilter(sens.ChFilterMagnetAccess())
    manager.AddSensor(mag)

    # --- ROS manager ---
    ros_manager = chros.ChROSPythonManager()

    ros_manager.RegisterHandler(chros.ChROSClockHandler())  # clock first

    ros_manager.RegisterHandler(chros.ChROSCameraHandler(30, cam, "~/output/camera/data/image"))

    # 2D lidar ROS handler — publish as LaserScan to ~/output/lidar2d/data/scan
    ros_manager.RegisterHandler(
        chros.ChROSLidarHandler(
            lidar2d,
            "~/output/lidar2d/data/scan",
            chros.ChROSLidarHandlerMessageType_LASER_SCAN,
        )
    )

    ros_manager.RegisterHandler(chros.ChROSGPSHandler(gps, "~/output/gps/data"))

    acc_handler = chros.ChROSAccelerometerHandler(acc, "~/output/accelerometer/data")
    gyro_handler = chros.ChROSGyroscopeHandler(gyro, "~/output/gyroscope/data")
    mag_handler = chros.ChROSMagnetometerHandler(mag, "~/output/magnetometer/data")
    ros_manager.RegisterHandler(acc_handler)
    ros_manager.RegisterHandler(gyro_handler)
    ros_manager.RegisterHandler(mag_handler)

    imu_handler = chros.ChROSIMUHandler(100, "~/output/imu/data")
    imu_handler.SetAccelerometerHandler(acc_handler)
    imu_handler.SetGyroscopeHandler(gyro_handler)
    imu_handler.SetMagnetometerHandler(mag_handler)
    ros_manager.RegisterHandler(imu_handler)

    ros_manager.Initialize()  # init after all handlers, before loop

    # --- simulation parameters ---
    time_step = 1e-3          # physics step size (s)
    sim_end = 100.0           # simulation duration (s)
    render_fps = 50.0
    render_every = max(1, round(1.0 / (render_fps * time_step)))  # frames per render


    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene(); vis.Render(); vis.EndScene()
        for _ in range(render_every):
            manager.Update()                                              # pump sensors
            sys.DoStepDynamics(time_step)                                 # advance physics
            time = sys.GetChTime()
            if not ros_manager.Update(time, time_step):                   # publish to ROS
                break
            if sys.GetChTime() >= sim_end:
                break


if __name__ == "__main__":
    main()
