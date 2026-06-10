import os
import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.ros as chros
import pychrono.irrlicht as chronoirr


def main():
    sys = chrono.ChSystemNSC()                                        # rigid-body system
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))      # no gravity; body is driven kinematically
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # collision system (mesh body has geometry)

    # --- mesh body: a triangle-mesh object the sensors observe ---
    mmesh = chrono.ChTriangleMeshConnected()                          # load the bundled mesh
    mmesh.LoadWavefrontMesh(chrono.GetChronoDataFile("sensor/geometries/box.obj"), False, True)
    mmesh.Transform(chrono.ChVector3d(0, 0, 0), chrono.ChMatrix33d(2.0))   # scale up the box mesh

    trimesh_shape = chrono.ChVisualShapeTriangleMesh()               # visual shape from the mesh
    trimesh_shape.SetMesh(mmesh)
    trimesh_shape.SetName("Box Mesh")
    trimesh_shape.SetMutable(False)

    mesh_body = chrono.ChBody()                                       # the observed/rotating mesh body
    mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))                      # at the world origin
    mesh_body.SetFixed(False)                                         # free body, spun kinematically
    mesh_body.AddVisualShape(trimesh_shape, chrono.ChFramed())        # attach the mesh visual
    mesh_body.SetAngVelParent(chrono.ChVector3d(0, 0, 0.3))           # slow spin about Z so sensors see motion
    sys.Add(mesh_body)                                               # add the mesh body to the system

    # --- sensor manager + scene lighting (lights matter for the camera) ---
    manager = sens.ChSensorManager(sys)                              # owns all sensors
    intensity = 1.0                                                  # uniform light intensity
    manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 5000.0)
    manager.scene.AddPointLight(chrono.ChVector3f(9, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 5000.0)
    manager.scene.AddPointLight(chrono.ChVector3f(16, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 5000.0)
    manager.scene.AddPointLight(chrono.ChVector3f(23, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 5000.0)

    # --- RGB camera sensor riding on the mesh body ---
    cam_offset = chrono.ChFramed(                                    # camera offset pose on the body
        chrono.ChVector3d(-8, 0, 1),
        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
    )
    cam = sens.ChCameraSensor(
        mesh_body,                                                  # attach to the mesh body
        30,                                                         # update_rate (Hz) — physical rate
        cam_offset,                                                 # offset pose
        1280, 720,                                                  # width, height
        chrono.CH_PI / 3,                                          # horizontal FOV (rad)
    )
    cam.SetName("Camera Sensor")
    cam.SetLag(0)                                                   # truth: lag = 0
    cam.SetCollectionWindow(0)                                      # exposure window = 0
    cam.PushFilter(sens.ChFilterVisualize(1280, 720, "RGB Camera"))   # live RGB preview
    cam.PushFilter(sens.ChFilterRGBA8Access())                     # host access to the RGBA8 buffer
    cam.PushFilter(sens.ChFilterSave("cam/rgb/"))                  # SAVE stream: color PNGs (scored core)
    manager.AddSensor(cam)                                          # push filters before AddSensor

    # --- 2D lidar sensor (v_samples = 1, both vertical angles 0) ---
    lidar_offset = chrono.ChFramed(                                 # lidar offset pose on the body
        chrono.ChVector3d(-8, 0, 1),
        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
    )
    horizontal_samples = 800                                       # horizontal beams
    vertical_samples = 1                                           # 2D lidar -> single scan line
    lidar = sens.ChLidarSensor(
        mesh_body,                                                 # attach to the mesh body
        5.0,                                                       # update_rate (Hz)
        lidar_offset,                                              # offset pose
        horizontal_samples,                                       # h_samples
        vertical_samples,                                         # v_samples (1 -> 2D)
        2 * chrono.CH_PI,                                        # horizontal_fov (rad)
        0.0,                                                      # max_vert_angle (0 for 2D)
        0.0,                                                      # min_vert_angle (0 for 2D)
        100.0,                                                    # max_range (m)
        sens.LidarBeamShape_RECTANGULAR,                          # beam shape
        2,                                                        # sample_radius
        0.003,                                                    # vertical divergence angle
        0.003,                                                    # horizontal divergence angle
        sens.LidarReturnMode_STRONGEST_RETURN,                    # return mode
    )
    lidar.SetName("Lidar Sensor")
    lidar.SetLag(0)                                                # no lag
    lidar.SetCollectionWindow(1.0 / 5.0)                          # collection window = 1 / update_rate
    lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth"))  # depth preview
    lidar.PushFilter(sens.ChFilterDIAccess())                     # host access to depth+intensity buffer
    lidar.PushFilter(sens.ChFilterPCfromDepth())                 # convert depth -> XYZI point cloud
    lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))  # point-cloud preview
    lidar.PushFilter(sens.ChFilterXYZIAccess())                  # host access to the XYZI point cloud
    manager.AddSensor(lidar)                                      # push filters before AddSensor

    # --- GPS sensor ---
    gps_offset = chrono.ChFramed(                                 # GPS offset pose on the body
        chrono.ChVector3d(-8, 0, 1),
        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
    )
    gps = sens.ChGPSSensor(
        mesh_body,                                               # attach to the mesh body
        10,                                                      # update_rate (Hz)
        gps_offset,                                              # offset pose
        chrono.ChVector3d(-89.400, 43.070, 260.0),             # reference lat/lon/alt
        sens.ChNoiseNone(),                                      # noise model (none)
    )
    gps.SetName("GPS Sensor")
    gps.SetLag(0)                                                # no lag
    gps.SetCollectionWindow(0)                                  # instantaneous
    gps.PushFilter(sens.ChFilterGPSAccess())                   # host access to GPS data
    manager.AddSensor(gps)

    # --- accelerometer (IMU) sensor ---
    imu_offset = chrono.ChFramed(                                # IMU offset pose on the body
        chrono.ChVector3d(-8, 0, 1),
        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
    )
    acc = sens.ChAccelerometerSensor(mesh_body, 100, imu_offset, sens.ChNoiseNone())   # 100 Hz accel
    acc.SetName("Accelerometer Sensor")
    acc.SetLag(0)
    acc.SetCollectionWindow(0)
    acc.PushFilter(sens.ChFilterAccelAccess())                  # host access to accel data
    manager.AddSensor(acc)

    # --- gyroscope sensor ---
    gyro = sens.ChGyroscopeSensor(mesh_body, 100, imu_offset, sens.ChNoiseNone())      # 100 Hz gyro
    gyro.SetName("Gyroscope Sensor")
    gyro.SetLag(0)
    gyro.SetCollectionWindow(0)
    gyro.PushFilter(sens.ChFilterGyroAccess())                 # host access to angular rate
    manager.AddSensor(gyro)

    # --- magnetometer sensor ---
    mag = sens.ChMagnetometerSensor(mesh_body, 100, imu_offset, sens.ChNoiseNone(),
                                    chrono.ChVector3d(-89.400, 43.070, 260.0))          # geo reference for field
    mag.SetName("Magnetometer Sensor")
    mag.SetLag(0)
    mag.SetCollectionWindow(0)
    mag.PushFilter(sens.ChFilterMagnetAccess())                # host access to mag field
    manager.AddSensor(mag)

    # --- ROS manager + handlers (publish each sensor + the body state) ---
    mesh_body.SetName("mesh_body")                              # name the body for body/TF topics
    ros_manager = chros.ChROSPythonManager()                   # Python manager hosts all handlers
    ros_manager.RegisterHandler(chros.ChROSClockHandler())     # /clock FIRST — time-sync the ROS graph

    body_handler = chros.ChROSBodyHandler(25, mesh_body, "~/output/mesh_body/state")   # body pose/twist
    ros_manager.RegisterHandler(body_handler)

    cam_handler = chros.ChROSCameraHandler(cam.GetUpdateRate() / 4, cam, "~/output/camera/data/image")   # image
    ros_manager.RegisterHandler(cam_handler)

    lidar_handler = chros.ChROSLidarHandler(lidar, "~/output/lidar/data/pointcloud")    # point cloud
    ros_manager.RegisterHandler(lidar_handler)

    gps_handler = chros.ChROSGPSHandler(gps, "~/output/gps/data")                       # NavSatFix
    ros_manager.RegisterHandler(gps_handler)

    acc_handler = chros.ChROSAccelerometerHandler(acc, "~/output/accelerometer/data")   # accel
    ros_manager.RegisterHandler(acc_handler)

    gyro_handler = chros.ChROSGyroscopeHandler(gyro, "~/output/gyroscope/data")         # angular rate
    ros_manager.RegisterHandler(gyro_handler)

    mag_handler = chros.ChROSMagnetometerHandler(mag, "~/output/magnetometer/data")     # mag field
    ros_manager.RegisterHandler(mag_handler)

    imu_handler = chros.ChROSIMUHandler(100, "~/output/imu/data")                       # fused IMU message
    imu_handler.SetAccelerometerHandler(acc_handler)
    imu_handler.SetGyroscopeHandler(gyro_handler)
    imu_handler.SetMagnetometerHandler(mag_handler)
    ros_manager.RegisterHandler(imu_handler)

    ros_manager.Initialize()                                   # initialize ONCE, after all handlers

    # --- Irrlicht window (review render path; sensor renderer is separate) ---
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("Sensor + ROS scene")
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(-12, -6, 4), chrono.ChVector3d(0, 0, 0))
    vis.AddTypicalLights()

    time_step = 1e-3                                            # integration step
    sim_end = 10.0                                             # simulation duration (s)
    render_fps = 30.0                                         # render cadence
    render_every = max(1, round(1.0 / (render_fps * time_step)))   # untagged cadence constant
    time = 0
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            manager.Update()                                 # pump all sensors -> fill their buffers
            if not ros_manager.Update(time, time_step):      # publish sensor/body state to ROS
                break                                        # ROS layer shut down -> stop
            sys.DoStepDynamics(time_step)                    # advance physics
            time = sys.GetChTime()
            if sys.GetChTime() >= sim_end:
                break


if __name__ == "__main__":
    main()
