import os
import pychrono as ch
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr
import pychrono.ros as chros

def main():
    sys = ch.ChSystemNSC()                                           # NSC rigid-body system

    # Load and add a mesh body for visual interest
    mmesh = ch.ChTriangleMeshConnected()
    mmesh.LoadWavefrontMesh(ch.GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"), False, True)  # load HMMWV chassis mesh
    mmesh.Transform(ch.ChVector3d(0, 0, 0), ch.ChMatrix33d(1))      # identity transform

    trimesh_shape = ch.ChVisualShapeTriangleMesh()
    trimesh_shape.SetMesh(mmesh)                                     # attach mesh to visual shape
    trimesh_shape.SetName("HMMWV Chassis Mesh")
    trimesh_shape.SetMutable(False)

    mesh_body = ch.ChBody()
    mesh_body.SetPos(ch.ChVector3d(0, 0, 0))                        # place at origin
    mesh_body.AddVisualShape(trimesh_shape)
    mesh_body.SetFixed(False)                                        # movable body
    mesh_body.SetMass(0)                                             # zero mass: kinematic
    sys.Add(mesh_body)                                               # add mesh body to simulation

    # Ground body to attach sensors
    ground_body = ch.ChBodyEasyBox(1, 1, 1, 1000, False, False)
    ground_body.SetPos(ch.ChVector3d(0, 0, 0))
    ground_body.SetFixed(False)                                      # movable
    ground_body.SetMass(0)                                           # zero mass: kinematic
    sys.Add(ground_body)

    # Sensor manager
    sens_manager = sens.ChSensorManager(sys)

    intensity = 1.0
    sens_manager.scene.AddPointLight(ch.ChVector3f(2, 2.5, 100), ch.ChColor(intensity, intensity, intensity), 500.0)   # overhead lights
    sens_manager.scene.AddPointLight(ch.ChVector3f(9, 2.5, 100), ch.ChColor(intensity, intensity, intensity), 500.0)
    sens_manager.scene.AddPointLight(ch.ChVector3f(16, 2.5, 100), ch.ChColor(intensity, intensity, intensity), 500.0)
    sens_manager.scene.AddPointLight(ch.ChVector3f(23, 2.5, 100), ch.ChColor(intensity, intensity, intensity), 500.0)

    # Camera sensor
    offset_pose = ch.ChFramed(ch.ChVector3d(-8, 0, 2), ch.QuatFromAngleAxis(.2, ch.ChVector3d(0, 1, 0)))  # camera offset
    cam = sens.ChCameraSensor(ground_body, 30, offset_pose, 1280, 720, 1.408)  # 30 Hz, 1280x720, 1.408 rad FOV
    cam.PushFilter(sens.ChFilterVisualize(1280, 720))               # live preview
    cam.PushFilter(sens.ChFilterRGBA8Access())                      # host RGBA8 access
    cam.SetName("camera")
    sens_manager.AddSensor(cam)

    # 3D lidar sensor
    lidar = sens.ChLidarSensor(ground_body, 5., offset_pose, 90, 300, 2*ch.CH_PI, ch.CH_PI / 12, -ch.CH_PI / 6, 100., 0)  # 3D lidar, 5 Hz
    lidar.PushFilter(sens.ChFilterDIAccess())                       # depth + intensity access
    lidar.PushFilter(sens.ChFilterPCfromDepth())                    # depth -> XYZI point cloud
    lidar.PushFilter(sens.ChFilterXYZIAccess())                     # XYZI host access
    lidar.PushFilter(sens.ChFilterVisualizePointCloud(1280, 720, 1, "Lidar PC data"))  # point cloud preview
    lidar.SetName("lidar")
    sens_manager.AddSensor(lidar)

    # 2D lidar sensor
    offset_pose_2dlidar = ch.ChFramed(ch.ChVector3d(-8, 0, 0), ch.QuatFromAngleAxis(.2, ch.ChVector3d(0, 1, 0)))  # 2D lidar offset
    lidar2d = sens.ChLidarSensor(ground_body, 5, offset_pose_2dlidar, 480, 1, 2 * ch.CH_PI, ch.CH_PI / 12, -ch.CH_PI / 6, 100.0)  # 2D lidar (v_samples=1)
    lidar2d.PushFilter(sens.ChFilterDIAccess())                     # depth + intensity access (required)
    lidar2d.PushFilter(sens.ChFilterPCfromDepth())                  # convert depth to point cloud
    lidar2d.PushFilter(sens.ChFilterXYZIAccess())                   # XYZI host access (required)
    lidar2d.PushFilter(sens.ChFilterVisualizePointCloud(1280, 720, 1, "2D Lidar Scan Data"))  # visualize 2D scan
    sens_manager.AddSensor(lidar2d)

    # GPS sensor
    noise_model_none = sens.ChNoiseNone()
    gps_reference = ch.ChVector3d(-89.4, 433.07, 260.)              # GPS reference coordinates
    gps = sens.ChGPSSensor(ground_body, 10, offset_pose, gps_reference, noise_model_none)  # 10 Hz GPS
    gps.PushFilter(sens.ChFilterGPSAccess())                        # GPS data access
    gps.SetName("gps")
    sens_manager.AddSensor(gps)

    # Accelerometer sensor
    acc = sens.ChAccelerometerSensor(ground_body, 100, offset_pose, noise_model_none)  # 100 Hz accelerometer
    acc.PushFilter(sens.ChFilterAccelAccess())                      # acceleration data access
    acc.SetName("accelerometer")
    sens_manager.AddSensor(acc)

    # Gyroscope sensor
    gyro = sens.ChGyroscopeSensor(ground_body, 100, offset_pose, noise_model_none)  # 100 Hz gyroscope
    gyro.PushFilter(sens.ChFilterGyroAccess())                      # gyroscope data access
    gyro.SetName("gyroscope")
    sens_manager.AddSensor(gyro)

    # Magnetometer sensor
    mag = sens.ChMagnetometerSensor(ground_body, 100, offset_pose, noise_model_none, gps_reference)  # 100 Hz magnetometer
    mag.PushFilter(sens.ChFilterMagnetAccess())                     # magnetometer data access
    mag.SetName("magnetometer")
    sens_manager.AddSensor(mag)

    sens_manager.Update()                                           # initialize sensor manager

    # ROS manager setup
    ros_manager = chros.ChROSPythonManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler())          # clock handler first

    ros_manager.RegisterHandler(chros.ChROSCameraHandler(cam.GetUpdateRate() / 4, cam, "~/output/camera/data/image"))  # camera at 1/4 rate
    ros_manager.RegisterHandler(chros.ChROSLidarHandler(lidar, "~/output/lidar/data/pointcloud"))  # 3D lidar pointcloud
    ros_manager.RegisterHandler(chros.ChROSLidarHandler(lidar2d, "~/output/lidar2d/data/scan", chros.ChROSLidarHandlerMessageType_LASER_SCAN))  # 2D lidar laser scan
    ros_manager.RegisterHandler(chros.ChROSGPSHandler(gps, "~/output/gps/data"))  # GPS

    acc_handler = chros.ChROSAccelerometerHandler(acc, "~/output/accelerometer/data")  # accelerometer handler
    ros_manager.RegisterHandler(acc_handler)
    gyro_handler = chros.ChROSGyroscopeHandler(gyro, "~/output/gyroscope/data")  # gyroscope handler
    ros_manager.RegisterHandler(gyro_handler)
    mag_handler = chros.ChROSMagnetometerHandler(mag, "~/output/magnetometer/data")  # magnetometer handler
    ros_manager.RegisterHandler(mag_handler)

    imu_handler = chros.ChROSIMUHandler(100, "~/output/imu/data")   # fused IMU handler at 100 Hz
    imu_handler.SetAccelerometerHandler(acc_handler)
    imu_handler.SetGyroscopeHandler(gyro_handler)
    imu_handler.SetMagnetometerHandler(mag_handler)
    ros_manager.RegisterHandler(imu_handler)

    ros_manager.Initialize()                                        # initialize after all handlers registered

    # Irrlicht visualization (built unconditionally)
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetCameraVertical(ch.CameraVerticalDir_Z)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("sensros")
    vis.Initialize()                                                # Initialize FIRST
    vis.AddLogo(ch.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(ch.ChVector3d(-8, -8, 4), ch.ChVector3d(0, 0, 0))  # camera view
    vis.AddTypicalLights()

    time = 0
    time_step = 1e-3                                                # simulation time step
    time_end = 100                                                  # 100 s simulation

    render_fps = 50.0
    render_every = max(1, round(1.0 / (render_fps * time_step)))   # render cadence

    ground_body.SetAngVelParent(ch.ChVector3d(0, 0, 0.1))          # spin ground body so sensors see motion


    while vis.Run() and time < time_end:
        vis.BeginScene(); vis.Render(); vis.EndScene()
        for _ in range(render_every):
            time = sys.GetChTime()                                  # update simulation time
            sens_manager.Update()                                   # pump all sensors
            if not ros_manager.Update(time, time_step):             # publish to ROS; exit if ROS shuts down
                break
            sys.DoStepDynamics(time_step)                           # advance physics
            if time >= time_end:
                break


if __name__ == "__main__":
    main()
