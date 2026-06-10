import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens
import pychrono.ros as chros


def main():
    sys = chrono.ChSystemNSC()                                        # NSC rigid-body system
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # collision system

    phys_mat = chrono.ChContactMaterialNSC()                          # contact material for scene bodies

    # A mesh object so the camera/lidar see interesting geometry.
    mmesh = chrono.ChTriangleMeshConnected()
    mmesh.LoadWavefrontMesh(chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"), False, True)
    mmesh.Transform(chrono.ChVector3d(0, 0, 0), chrono.ChMatrix33d(1))   # identity transform
    trimesh_shape = chrono.ChVisualShapeTriangleMesh()
    trimesh_shape.SetMesh(mmesh)
    trimesh_shape.SetName("HMMWV Chassis Mesh")
    trimesh_shape.SetMutable(False)
    mesh_body = chrono.ChBody()
    mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))
    mesh_body.AddVisualShape(trimesh_shape)
    mesh_body.SetFixed(False)
    mesh_body.SetMass(0)
    sys.Add(mesh_body)

    # The carrier body the sensors ride on — massless, free, spun via SetAngVelParent.
    ground_body = chrono.ChBodyEasyBox(1, 1, 1, 1000, False, False)   # no visual / no collision (sensor carrier)
    ground_body.SetPos(chrono.ChVector3d(0, 0, 0))
    ground_body.SetFixed(False)                                       # free so the angular velocity takes effect
    ground_body.SetMass(0)
    ground_body.SetName("ground_body")                               # ROS / TF frame name
    sys.Add(ground_body)

    # --- sensor manager + camera-scene lighting (point lights are the canonical setup) ---
    sens_manager = sens.ChSensorManager(sys)                         # owns every sensor
    intensity = 1.0                                                  # uniform light intensity
    sens_manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    sens_manager.scene.AddPointLight(chrono.ChVector3f(9, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    sens_manager.scene.AddPointLight(chrono.ChVector3f(16, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    sens_manager.scene.AddPointLight(chrono.ChVector3f(23, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)

    offset_pose = chrono.ChFramed(                                   # shared sensor offset pose on the body
        chrono.ChVector3d(-8, 0, 2),
        chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0)),
    )

    # --- RGB camera sensor ---
    cam = sens.ChCameraSensor(ground_body, 30, offset_pose, 1280, 720, 1.408)   # 30 Hz, 1280x720, 1.408 rad hfov
    cam.SetName("camera")
    cam.PushFilter(sens.ChFilterVisualize(1280, 720, "Camera"))     # live RGB preview
    cam.PushFilter(sens.ChFilterRGBA8Access())                      # host access to the RGBA8 buffer
    cam.PushFilter(sens.ChFilterSave("cam/rgb/"))                   # save color PNG stream
    sens_manager.AddSensor(cam)                                     # push all filters BEFORE AddSensor

    # --- 3D lidar sensor (full vertical fan) ---
    lidar = sens.ChLidarSensor(
        ground_body,                                               # body the lidar is attached to
        5.0,                                                       # update_rate (Hz)
        offset_pose,                                               # offset pose
        90,                                                        # h_samples
        300,                                                       # v_samples (3D fan)
        2 * chrono.CH_PI,                                         # horizontal_fov (rad)
        chrono.CH_PI / 12,                                       # max_vert_angle
        -chrono.CH_PI / 6,                                       # min_vert_angle
        100.0,                                                   # max_range
        0,                                                       # clip_near (last positional)
    )
    lidar.SetName("lidar")
    lidar.PushFilter(sens.ChFilterDIAccess())                      # host access to depth+intensity
    lidar.PushFilter(sens.ChFilterPCfromDepth())                   # depth -> XYZI point cloud
    lidar.PushFilter(sens.ChFilterXYZIAccess())                    # host access to XYZI
    lidar.PushFilter(sens.ChFilterVisualizePointCloud(1280, 720, 1, "Lidar Point Cloud"))  # point-cloud preview
    sens_manager.AddSensor(lidar)

    # --- 2D lidar sensor (single horizontal scan plane) ---
    lidar_2d = sens.ChLidarSensor(
        ground_body,                                               # body the 2D lidar is attached to
        5.0,                                                       # update_rate (Hz)
        offset_pose,                                               # offset pose
        90,                                                        # h_samples
        1,                                                         # v_samples = 1 -> 2D planar lidar
        2 * chrono.CH_PI,                                         # horizontal_fov (rad)
        0.0,                                                      # max_vert_angle = 0 (2D)
        0.0,                                                      # min_vert_angle = 0 (2D)
        100.0,                                                   # max_range
        0,                                                       # clip_near
    )
    lidar_2d.SetName("2d_lidar")
    lidar_2d.PushFilter(sens.ChFilterDIAccess())                   # host access to depth+intensity
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())                # depth -> XYZI point cloud
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())                 # host access to XYZI
    lidar_2d.PushFilter(sens.ChFilterVisualizePointCloud(1280, 720, 1, "2D Lidar Point Cloud"))  # named point-cloud preview
    sens_manager.AddSensor(lidar_2d)

    # --- GPS sensor ---
    noise_model_none = sens.ChNoiseNone()                          # no sensor noise
    gps_reference = chrono.ChVector3d(-89.4, 433.07, 260.0)        # reference lat/lon/alt origin
    gps = sens.ChGPSSensor(ground_body, 10, offset_pose, gps_reference, noise_model_none)
    gps.SetName("gps")
    gps.PushFilter(sens.ChFilterGPSAccess())                       # host access to GPS data
    sens_manager.AddSensor(gps)

    # --- IMU sub-sensors: accelerometer, gyroscope, magnetometer ---
    acc = sens.ChAccelerometerSensor(ground_body, 100, offset_pose, noise_model_none)
    acc.SetName("accelerometer")
    acc.PushFilter(sens.ChFilterAccelAccess())                     # host access to acceleration
    sens_manager.AddSensor(acc)

    gyro = sens.ChGyroscopeSensor(ground_body, 100, offset_pose, noise_model_none)
    gyro.SetName("gyroscope")
    gyro.PushFilter(sens.ChFilterGyroAccess())                     # host access to angular rate
    sens_manager.AddSensor(gyro)

    mag = sens.ChMagnetometerSensor(ground_body, 100, offset_pose, noise_model_none, gps_reference)
    mag.SetName("magnetometer")
    mag.PushFilter(sens.ChFilterMagnetAccess())                    # host access to magnetic field
    sens_manager.AddSensor(mag)

    sens_manager.Update()                                          # prime all sensor buffers before ROS publishes

    # --- ROS manager + handlers (SCORED CORE — the data output of a ROS plan) ---
    ros_manager = chros.ChROSPythonManager()                       # Python ROS manager (hosts all handlers)
    ros_manager.RegisterHandler(chros.ChROSClockHandler())         # /clock FIRST — time-sync the ROS graph
    ros_manager.RegisterHandler(chros.ChROSCameraHandler(cam.GetUpdateRate() / 4, cam, "~/output/camera/data/image"))
    ros_manager.RegisterHandler(chros.ChROSLidarHandler(lidar, "~/output/lidar/data/pointcloud"))   # 3D -> PointCloud2
    ros_manager.RegisterHandler(chros.ChROSLidarHandler(                                            # 2D -> LaserScan
        lidar_2d, "~/output/lidar2d/data/scan",
        chros.ChROSLidarHandlerMessageType_LASER_SCAN,
    ))
    ros_manager.RegisterHandler(chros.ChROSGPSHandler(gps, "~/output/gps/data"))
    acc_handler = chros.ChROSAccelerometerHandler(acc, "~/output/accelerometer/data")
    ros_manager.RegisterHandler(acc_handler)
    gyro_handler = chros.ChROSGyroscopeHandler(gyro, "~/output/gyroscope/data")
    ros_manager.RegisterHandler(gyro_handler)
    mag_handler = chros.ChROSMagnetometerHandler(mag, "~/output/magnetometer/data")
    ros_manager.RegisterHandler(mag_handler)
    imu_handler = chros.ChROSIMUHandler(100, "~/output/imu/data")  # fused sensor_msgs/Imu publisher
    imu_handler.SetAccelerometerHandler(acc_handler)
    imu_handler.SetGyroscopeHandler(gyro_handler)
    imu_handler.SetMagnetometerHandler(mag_handler)
    ros_manager.RegisterHandler(imu_handler)
    ros_manager.Initialize()                                       # initialize ONCE, after every register

    # --- Irrlicht visualization window (built unconditionally, Initialize first) ---
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)              # Z-up
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("ROS Sensor Suite — camera + 3D/2D lidar + GPS + IMU")
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(-10, -6, 4), chrono.ChVector3d(0, 0, 0))
    vis.AddTypicalLights()

    # Spin the carrier so the sensors attached to it appear to move.
    ground_body.SetAngVelParent(chrono.ChVector3d(0, 0, 0.1))     # 0.1 rad/s yaw — gyro z reads 0.1

    time = 0                                                       # sim clock
    time_step = 1e-3                                              # integration step
    time_end = 10.0                                              # total simulated time
    render_fps = 50.0                                            # review render cadence
    render_every = max(1, round(1.0 / (render_fps * time_step)))  # untagged cadence constant
    while vis.Run() and time < time_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            time = sys.GetChTime()
            sens_manager.Update()                                # pump every sensor once per step
            if not ros_manager.Update(time, time_step):          # publish to ROS — break on shutdown
                break
            sys.DoStepDynamics(time_step)                        # advance physics
            if time >= time_end:
                break


if __name__ == "__main__":
    main()
