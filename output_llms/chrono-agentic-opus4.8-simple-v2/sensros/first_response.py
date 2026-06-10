import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens
import pychrono.ros as chros


def main():
    sys = chrono.ChSystemNSC()                                       # NSC system
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))     # no gravity: kinematic spin
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET) # bullet collision

    # --- ground body that carries all the sensors; spun so the sensors see motion ---
    mmesh = chrono.ChTriangleMeshConnected()                         # mesh for visualization
    mmesh.LoadWavefrontMesh(chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"), False, True)
    mmesh.Transform(chrono.ChVector3d(0, 0, 0), chrono.ChMatrix33d(1.0))  # identity transform

    trimesh_shape = chrono.ChVisualShapeTriangleMesh()              # visual mesh shape
    trimesh_shape.SetMesh(mmesh)                                    # attach loaded mesh
    trimesh_shape.SetName("ground_mesh")                           # name the asset
    trimesh_shape.SetMutable(False)                                # static geometry

    ground_body = chrono.ChBodyEasyBox(1, 1, 1, 1000, False, False) # ground carrier body
    ground_body.SetPos(chrono.ChVector3d(0, 0, 0))                  # at the origin
    ground_body.AddVisualShape(trimesh_shape, chrono.ChFramed())    # show the mesh on it
    ground_body.SetFixed(False)                                    # free body so it can spin
    sys.Add(ground_body)                                          # add to the system

    ground_body.SetAngVelParent(chrono.ChVector3d(0, 0, 1))        # spin about Z so sensors move

    # --- sensor manager + scene lighting (camera-only point/area lights) ---
    manager = sens.ChSensorManager(sys)                            # owns every sensor
    intensity = 1.0                                               # uniform light intensity
    manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100),
                                chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(chrono.ChVector3f(9, 2.5, 100),
                                chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddAreaLight(chrono.ChVector3f(0, 0, 4),
                               chrono.ChColor(intensity, intensity, intensity), 500.0,
                               chrono.ChVector3f(1, 0, 0), chrono.ChVector3f(0, -1, 0))

    gps_reference = chrono.ChVector3d(-89.400, 43.070, 260.0)      # lat/lon/alt origin

    # --- camera sensor on the ground body ---
    offset_pose = chrono.ChFramed(chrono.ChVector3d(-5, 0, 2),
                                  chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0)))
    cam = sens.ChCameraSensor(ground_body, 30, offset_pose, 1280, 720, 1.408)  # RGB camera
    cam.SetName("Camera Sensor")                                  # sensor name
    cam.SetLag(0)                                                 # no lag
    cam.SetCollectionWindow(0)                                    # instantaneous exposure
    cam.PushFilter(sens.ChFilterVisualize(1280, 720, "RGB Camera"))  # live preview
    cam.PushFilter(sens.ChFilterRGBA8Access())                    # host RGBA access
    cam.PushFilter(sens.ChFilterSave("cam/rgb/"))                 # save color PNGs
    manager.AddSensor(cam)                                        # register the camera

    # --- lidar sensor on the ground body ---
    lidar_pose = chrono.ChFramed(chrono.ChVector3d(0, 0, 1),
                                 chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
    lidar = sens.ChLidarSensor(
        ground_body, 5.0, lidar_pose, 800, 300,
        2 * chrono.CH_PI, chrono.CH_PI / 12, -chrono.CH_PI / 6, 100.0,
        sens.LidarBeamShape_RECTANGULAR, 2, 0.003, 0.003,
        sens.LidarReturnMode_STRONGEST_RETURN)                    # 3D lidar
    lidar.SetName("Lidar Sensor")                                 # sensor name
    lidar.SetLag(0)                                               # no lag
    lidar.SetCollectionWindow(1.0 / 5.0)                          # window = 1/update_rate
    lidar.PushFilter(sens.ChFilterVisualize(800, 300, "Raw Lidar Depth"))  # depth preview
    lidar.PushFilter(sens.ChFilterDIAccess())                     # depth+intensity access
    lidar.PushFilter(sens.ChFilterPCfromDepth())                 # depth -> point cloud
    lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
    lidar.PushFilter(sens.ChFilterXYZIAccess())                  # XYZI access
    manager.AddSensor(lidar)                                      # register the lidar

    # --- GPS sensor on the ground body ---
    gps_pose = chrono.ChFramed(chrono.ChVector3d(0, 0, 1),
                               chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
    gps = sens.ChGPSSensor(ground_body, 10, gps_pose, gps_reference, sens.ChNoiseNone())
    gps.SetName("GPS Sensor")                                     # sensor name
    gps.SetLag(0)                                                 # no lag
    gps.SetCollectionWindow(0)                                    # instantaneous read
    gps.PushFilter(sens.ChFilterGPSAccess())                     # host GPS access
    manager.AddSensor(gps)                                        # register the GPS

    # --- accelerometer sensor on the ground body ---
    imu_pose = chrono.ChFramed(chrono.ChVector3d(0, 0, 1),
                               chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
    acc = sens.ChAccelerometerSensor(ground_body, 100, imu_pose, sens.ChNoiseNone())
    acc.SetName("Accelerometer Sensor")                          # sensor name
    acc.SetLag(0)                                                 # no lag
    acc.SetCollectionWindow(0)                                    # instantaneous read
    acc.PushFilter(sens.ChFilterAccelAccess())                   # host accel access
    manager.AddSensor(acc)                                        # register the accelerometer

    # --- gyroscope sensor on the ground body ---
    gyro = sens.ChGyroscopeSensor(ground_body, 100, imu_pose, sens.ChNoiseNone())
    gyro.SetName("Gyroscope Sensor")                             # sensor name
    gyro.SetLag(0)                                                # no lag
    gyro.SetCollectionWindow(0)                                   # instantaneous read
    gyro.PushFilter(sens.ChFilterGyroAccess())                   # host gyro access
    manager.AddSensor(gyro)                                       # register the gyroscope

    # --- magnetometer sensor on the ground body (needs the GPS reference) ---
    mag = sens.ChMagnetometerSensor(ground_body, 100, imu_pose, sens.ChNoiseNone(), gps_reference)
    mag.SetName("Magnetometer Sensor")                          # sensor name
    mag.SetLag(0)                                                 # no lag
    mag.SetCollectionWindow(0)                                    # instantaneous read
    mag.PushFilter(sens.ChFilterMagnetAccess())                 # host magnetometer access
    manager.AddSensor(mag)                                        # register the magnetometer

    # --- ROS manager: publish each sensor + the body pose to its own topic ---
    ros_manager = chros.ChROSPythonManager()                     # python-capable ROS manager
    ros_manager.RegisterHandler(chros.ChROSClockHandler())       # /clock first

    body_handler = chros.ChROSBodyHandler(25, ground_body, "~/output/ground/state")
    ros_manager.RegisterHandler(body_handler)                    # publish body pose/twist

    cam_handler = chros.ChROSCameraHandler(cam.GetUpdateRate() / 4, cam, "~/output/camera/data")
    ros_manager.RegisterHandler(cam_handler)                     # publish camera image

    lidar_handler = chros.ChROSLidarHandler(lidar, "~/output/lidar/data",
                                            chros.ChROSLidarHandlerMessageType_LASER_SCAN)
    ros_manager.RegisterHandler(lidar_handler)                   # publish lidar laser scan

    gps_handler = chros.ChROSGPSHandler(gps, "~/output/gps/data")
    ros_manager.RegisterHandler(gps_handler)                     # publish NavSatFix

    acc_handler = chros.ChROSAccelerometerHandler(acc, "~/output/accelerometer/data")
    ros_manager.RegisterHandler(acc_handler)                     # publish accel

    gyro_handler = chros.ChROSGyroscopeHandler(gyro, "~/output/gyroscope/data")
    ros_manager.RegisterHandler(gyro_handler)                    # publish angular rate

    mag_handler = chros.ChROSMagnetometerHandler(mag, "~/output/magnetometer/data")
    ros_manager.RegisterHandler(mag_handler)                     # publish magnetic field

    imu_handler = chros.ChROSIMUHandler(100, "~/output/imu/data")  # fused IMU message
    imu_handler.SetAccelerometerHandler(acc_handler)             # feed accel
    imu_handler.SetGyroscopeHandler(gyro_handler)                # feed gyro
    imu_handler.SetMagnetometerHandler(mag_handler)              # feed mag
    ros_manager.RegisterHandler(imu_handler)                     # publish sensor_msgs/Imu

    ros_manager.Initialize()                                      # initialize ONCE after registration

    # --- Irrlicht visualization window (review renderer) ---
    vis = chronoirr.ChVisualSystemIrrlicht()                     # irrlicht system
    vis.AttachSystem(sys)                                         # bind the physical system
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)            # Z-up camera
    vis.SetWindowSize(1280, 720)                                 # window resolution
    vis.SetWindowTitle("Sensor + ROS Demo")                     # window title
    vis.Initialize()                                             # create the device first
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # logo
    vis.AddSkyBox()                                              # sky box
    vis.AddCamera(chrono.ChVector3d(8, -8, 4), chrono.ChVector3d(0, 0, 0))  # view
    vis.AddTypicalLights()                                       # standard lights

    time_step = 1e-3                                             # integration step
    sim_end = 10.0                                               # run length (s)
    render_fps = 50.0                                            # review frame rate
    render_every = max(1, round(1.0 / (render_fps * time_step)))  # untagged cadence constant

    realtime_timer = chrono.ChRealtimeStepTimer()               # keep wall-clock ~ sim time
    time = 0                                                    # sim time accumulator
    while vis.Run() and time < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            time = sys.GetChTime()                              # current sim time
            manager.Update()                                    # pump every sensor once per step
            if not ros_manager.Update(time, time_step):         # publish to ROS; break on shutdown
                break
            sys.DoStepDynamics(time_step)                       # advance the physics
            realtime_timer.Spin(time_step)                      # real-time pacing
            if sys.GetChTime() >= sim_end:
                break


if __name__ == "__main__":
    main()
