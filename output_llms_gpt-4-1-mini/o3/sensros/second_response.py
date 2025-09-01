#  ================================================================
#  Corrected and updated PyChrono example
#  ------------------------------------------------
#  • Fixed several API-call problems that would have stopped the
#    original script from running (wrong class/function names,
#    wrong angular-velocity setter, wrong frame constructor, etc.).
#  • Added a 2-D planar LiDAR, gave both LiDARs named visualisation
#    filters, and exposed the new sensor through a ROS handler
#    (topic “~/output/lidar2d/data/scan”).
#  ================================================================

import pychrono as ch
import pychrono.sensor as sens
import pychrono.ros as chros


def main():
    # ----------------------------------------------------------------
    # 1. Create the Chrono simulation system
    # ----------------------------------------------------------------
    sys = ch.ChSystemNSC()

    # ----------------------------------------------------------------
    # 2. Load a mesh just for some scenery
    # ----------------------------------------------------------------
    mmesh = ch.ChTriangleMeshConnected()
    mmesh.LoadWavefrontMesh(
        ch.GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"),
        load_normals=False,
        load_uv=True,
    )
    mmesh.Transform(ch.ChVector3d(0, 0, 0), ch.ChMatrix33d(1))

    trimesh_shape = ch.ChVisualShapeTriangleMesh()
    trimesh_shape.SetMesh(mmesh)
    trimesh_shape.SetName("HMMWV Chassis Mesh")
    trimesh_shape.SetMutable(False)

    mesh_body = ch.ChBody()
    mesh_body.SetPos(ch.ChVector3d(0, 0, 0))
    mesh_body.AddVisualShape(trimesh_shape)
    mesh_body.SetFixed(False)
    mesh_body.SetMass(1e-6)                     # give a tiny mass to avoid 0-mass issues
    sys.Add(mesh_body)

    # ----------------------------------------------------------------
    # 3. Ground body – this is the sensor carrier
    # ----------------------------------------------------------------
    ground_body = ch.ChBodyEasyBox(1, 1, 1, 1000, collide=False, visualization=False)
    ground_body.SetPos(ch.ChVector3d(0, 0, 0))
    ground_body.SetFixed(False)
    ground_body.SetMass(1e-6)
    sys.Add(ground_body)

    # ----------------------------------------------------------------
    # 4. Sensor manager (lights, etc.)
    # ----------------------------------------------------------------
    sens_manager = sens.ChSensorManager(sys)

    intensity = 1.0
    sens_manager.scene.AddPointLight(ch.ChVector3f(2, 2.5, 100),
                                     ch.ChColor(intensity, intensity, intensity), 500.0)
    sens_manager.scene.AddPointLight(ch.ChVector3f(9, 2.5, 100),
                                     ch.ChColor(intensity, intensity, intensity), 500.0)
    sens_manager.scene.AddPointLight(ch.ChVector3f(16, 2.5, 100),
                                     ch.ChColor(intensity, intensity, intensity), 500.0)
    sens_manager.scene.AddPointLight(ch.ChVector3f(23, 2.5, 100),
                                     ch.ChColor(intensity, intensity, intensity), 500.0)

    # ----------------------------------------------------------------
    # 5. Common frame used by most of the sensors
    # ----------------------------------------------------------------
    offset_pose = ch.ChFrameD(
        ch.ChVector3d(-8, 0, 2),
        ch.Q_from_AngAxis(0.2, ch.ChVector3d(0, 1, 0))
    )

    # ----------------------------------------------------------------
    # 6. Camera
    # ----------------------------------------------------------------
    cam = sens.ChCameraSensor(ground_body,     # parent body
                              30,              # update rate [Hz]
                              offset_pose,
                              1280, 720,       # resolution
                              1.408)           # FOV
    cam.PushFilter(sens.ChFilterVisualize(1280, 720))
    cam.PushFilter(sens.ChFilterRGBA8Access())
    cam.SetName("camera")
    sens_manager.AddSensor(cam)

    # ----------------------------------------------------------------
    # 7. 3-D LiDAR
    # ----------------------------------------------------------------
    lidar3d = sens.ChLidarSensor(ground_body,      # parent body
                                 5.0,              # update rate [Hz]
                                 offset_pose,
                                 90, 300,          # horiz/vert samples
                                 2 * ch.CH_PI,     # horiz FOV
                                 ch.CH_PI / 12,    # vert FOV
                                 -ch.CH_PI / 6,    # vert dir
                                 100.0, 0.0)       # max dist, beam rad
    lidar3d.PushFilter(sens.ChFilterDIAccess())
    lidar3d.PushFilter(sens.ChFilterPCfromDepth())
    lidar3d.PushFilter(sens.ChFilterXYZIAccess())
    vis3d = sens.ChFilterVisualizePointCloud(1280, 720, 1)
    vis3d.SetName("3D LiDAR Point Cloud")
    lidar3d.PushFilter(vis3d)
    lidar3d.SetName("lidar3d")
    sens_manager.AddSensor(lidar3d)

    # ----------------------------------------------------------------
    # 8. NEW 2-D planar LiDAR (instruction #1)
    # ----------------------------------------------------------------
    lidar2d = sens.ChLidarSensor(ground_body,
                                 10.0,            # faster update
                                 offset_pose,
                                 360, 1,          # horiz samples, *one* vertical sample
                                 2 * ch.CH_PI,    # 360° horiz FOV
                                 0.0,             # 0° vertical FOV (planar)
                                 0.0,             # vertical direction
                                 100.0, 0.0)
    lidar2d.PushFilter(sens.ChFilterDIAccess())
    lidar2d.PushFilter(sens.ChFilterPCfromDepth())
    lidar2d.PushFilter(sens.ChFilterXYZIAccess())
    vis2d = sens.ChFilterVisualizePointCloud(640, 480, 1)
    vis2d.SetName("2D LiDAR Scan")
    lidar2d.PushFilter(vis2d)
    lidar2d.SetName("lidar2d")
    sens_manager.AddSensor(lidar2d)

    # ----------------------------------------------------------------
    # 9. Other inertial / navigation sensors
    # ----------------------------------------------------------------
    noise_model_none = sens.ChNoiseNone()
    gps_reference = ch.ChVector3d(-89.4, 433.07, 260.)

    gps = sens.ChGPSSensor(ground_body, 10, offset_pose, gps_reference, noise_model_none)
    gps.PushFilter(sens.ChFilterGPSAccess())
    gps.SetName("gps")
    sens_manager.AddSensor(gps)

    acc = sens.ChAccelerometerSensor(ground_body, 100, offset_pose, noise_model_none)
    acc.PushFilter(sens.ChFilterAccelAccess())
    acc.SetName("accelerometer")
    sens_manager.AddSensor(acc)

    gyro = sens.ChGyroscopeSensor(ground_body, 100, offset_pose, noise_model_none)
    gyro.PushFilter(sens.ChFilterGyroAccess())
    gyro.SetName("gyroscope")
    sens_manager.AddSensor(gyro)

    mag = sens.ChMagnetometerSensor(ground_body, 100, offset_pose, noise_model_none, gps_reference)
    mag.PushFilter(sens.ChFilterMagnetAccess())
    mag.SetName("magnetometer")
    sens_manager.AddSensor(mag)

    # Initialise all sensors once added
    sens_manager.Update()

    # ----------------------------------------------------------------
    # 10. ROS bridge
    # ----------------------------------------------------------------
    ros_manager = chros.ChROSPythonManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler())

    # Camera & 3-D LiDAR
    ros_manager.RegisterHandler(
        chros.ChROSCameraHandler(cam.GetUpdateRate() / 4, cam, "~/output/camera/data/image"))
    ros_manager.RegisterHandler(
        chros.ChROSLidarHandler(lidar3d, "~/output/lidar/data/pointcloud"))

    # NEW 2-D LiDAR handler (instruction #3)
    ros_manager.RegisterHandler(
        chros.ChROSLidarHandler(lidar2d, "~/output/lidar2d/data/scan"))

    # GPS
    ros_manager.RegisterHandler(chros.ChROSGPSHandler(gps, "~/output/gps/data"))

    # Individual IMU components
    acc_handler  = chros.ChROSAccelerometerHandler(acc, "~/output/accelerometer/data")
    gyro_handler = chros.ChROSGyroscopeHandler(gyro, "~/output/gyroscope/data")
    mag_handler  = chros.ChROSMagnetometerHandler(mag, "~/output/magnetometer/data")
    ros_manager.RegisterHandler(acc_handler)
    ros_manager.RegisterHandler(gyro_handler)
    ros_manager.RegisterHandler(mag_handler)

    # Combined IMU
    imu_handler = chros.ChROSIMUHandler(100, "~/output/imu/data")
    imu_handler.SetAccelerometerHandler(acc_handler)
    imu_handler.SetGyroscopeHandler(gyro_handler)
    imu_handler.SetMagnetometerHandler(mag_handler)
    ros_manager.RegisterHandler(imu_handler)

    ros_manager.Initialize()

    # ----------------------------------------------------------------
    # 11. Simple dynamic test – spin the ground body
    # ----------------------------------------------------------------
    ground_body.SetWvel_par(ch.ChVector3d(0, 0, 0.1))   # constant yaw rate

    # ----------------------------------------------------------------
    # 12. Simulation loop
    # ----------------------------------------------------------------
    time_end  = 100.0
    time_step = 1e-3

    while sys.GetChTime() < time_end:
        current_time = sys.GetChTime()

        sens_manager.Update()
        if not ros_manager.Update(current_time, time_step):
            break

        sys.DoStepDynamics(time_step)


# --------------------------------------------------------------------
# Run the demo
# --------------------------------------------------------------------
if __name__ == "__main__":
    main()