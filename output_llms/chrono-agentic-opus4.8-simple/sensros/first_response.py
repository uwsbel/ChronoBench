import os
import pychrono as ch
import pychrono.sensor as sens
import pychrono.ros as chros
import pychrono.irrlicht as chronoirr


def main():
    sys = ch.ChSystemNSC()                                            # NSC system
    sys.SetCollisionSystemType(ch.ChCollisionSystem.Type_BULLET)      # collision system (contact-capable scene)

    mmesh = ch.ChTriangleMeshConnected()                              # mesh object for visualization
    mmesh.LoadWavefrontMesh(ch.GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"), False, True)
    mmesh.Transform(ch.ChVector3d(0, 0, 0), ch.ChMatrix33d(1))        # identity placement

    trimesh_shape = ch.ChVisualShapeTriangleMesh()                    # visual shape from the mesh
    trimesh_shape.SetMesh(mmesh)
    trimesh_shape.SetName("HMMWV Chassis Mesh")
    trimesh_shape.SetMutable(False)

    mesh_body = ch.ChBody()                                           # body carrying the visual mesh
    mesh_body.SetPos(ch.ChVector3d(0, 0, 0))
    mesh_body.AddVisualShape(trimesh_shape)
    mesh_body.SetFixed(False)
    mesh_body.SetMass(0)
    sys.Add(mesh_body)

    ground_body = ch.ChBodyEasyBox(1, 1, 1, 1000, False, False)      # body the sensors attach to
    ground_body.SetPos(ch.ChVector3d(0, 0, 0))
    ground_body.SetFixed(False)
    ground_body.SetMass(0)
    sys.Add(ground_body)

    sens_manager = sens.ChSensorManager(sys)                         # sensor manager oversees all sensors

    intensity = 1.0                                                   # point lights for the camera sensor
    sens_manager.scene.AddPointLight(ch.ChVector3f(2, 2.5, 100), ch.ChColor(intensity, intensity, intensity), 500.0)
    sens_manager.scene.AddPointLight(ch.ChVector3f(9, 2.5, 100), ch.ChColor(intensity, intensity, intensity), 500.0)
    sens_manager.scene.AddPointLight(ch.ChVector3f(16, 2.5, 100), ch.ChColor(intensity, intensity, intensity), 500.0)
    sens_manager.scene.AddPointLight(ch.ChVector3f(23, 2.5, 100), ch.ChColor(intensity, intensity, intensity), 500.0)

    offset_pose = ch.ChFramed(ch.ChVector3d(-8, 0, 2),               # sensor offset pose on the ground body
                              ch.QuatFromAngleAxis(.2, ch.ChVector3d(0, 1, 0)))

    cam = sens.ChCameraSensor(ground_body, 30, offset_pose, 1280, 720, 1.408)   # RGB camera, 30 Hz
    cam.PushFilter(sens.ChFilterVisualize(1280, 720))                # live RGB preview
    cam.PushFilter(sens.ChFilterRGBA8Access())                       # host access to RGBA8 buffer
    cam.SetName("camera")
    sens_manager.AddSensor(cam)

    lidar = sens.ChLidarSensor(ground_body, 5., offset_pose, 90, 300,        # lidar, 5 Hz
                               2 * ch.CH_PI, ch.CH_PI / 12, -ch.CH_PI / 6, 100., 0)
    lidar.PushFilter(sens.ChFilterDIAccess())                        # depth+intensity access
    lidar.PushFilter(sens.ChFilterPCfromDepth())                    # depth -> point cloud
    lidar.PushFilter(sens.ChFilterXYZIAccess())                     # host access to XYZI
    lidar.PushFilter(sens.ChFilterVisualizePointCloud(1280, 720, 1))   # live point-cloud preview
    lidar.SetName("lidar")
    sens_manager.AddSensor(lidar)

    noise_model_none = sens.ChNoiseNone()                           # no sensor noise
    gps_reference = ch.ChVector3d(-89.4, 433.07, 260.)             # reference lat/lon/alt
    gps = sens.ChGPSSensor(ground_body, 10, offset_pose, gps_reference, noise_model_none)   # GPS, 10 Hz
    gps.PushFilter(sens.ChFilterGPSAccess())                        # host access to GPS data
    gps.SetName("gps")
    sens_manager.AddSensor(gps)

    acc = sens.ChAccelerometerSensor(ground_body, 100, offset_pose, noise_model_none)   # accelerometer, 100 Hz
    acc.PushFilter(sens.ChFilterAccelAccess())                      # host access to accel data
    acc.SetName("accelerometer")
    sens_manager.AddSensor(acc)

    gyro = sens.ChGyroscopeSensor(ground_body, 100, offset_pose, noise_model_none)       # gyroscope, 100 Hz
    gyro.PushFilter(sens.ChFilterGyroAccess())                     # host access to gyro data
    gyro.SetName("gyroscope")
    sens_manager.AddSensor(gyro)

    mag = sens.ChMagnetometerSensor(ground_body, 100, offset_pose, noise_model_none, gps_reference)  # magnetometer, 100 Hz
    mag.PushFilter(sens.ChFilterMagnetAccess())                    # host access to magnetic field
    mag.SetName("magnetometer")
    sens_manager.AddSensor(mag)

    sens_manager.Update()                                           # prime the sensors once

    ros_manager = chros.ChROSPythonManager()                        # ROS bridge owns the handlers
    ros_manager.RegisterHandler(chros.ChROSClockHandler())          # /clock first
    ros_manager.RegisterHandler(chros.ChROSCameraHandler(cam.GetUpdateRate() / 4, cam, "~/output/camera/data/image"))
    ros_manager.RegisterHandler(chros.ChROSLidarHandler(lidar, "~/output/lidar/data/pointcloud"))
    ros_manager.RegisterHandler(chros.ChROSGPSHandler(gps, "~/output/gps/data"))
    acc_handler = chros.ChROSAccelerometerHandler(acc, "~/output/accelerometer/data")
    ros_manager.RegisterHandler(acc_handler)
    gyro_handler = chros.ChROSGyroscopeHandler(gyro, "~/output/gyroscope/data")
    ros_manager.RegisterHandler(gyro_handler)
    mag_handler = chros.ChROSMagnetometerHandler(mag, "~/output/magnetometer/data")
    ros_manager.RegisterHandler(mag_handler)
    imu_handler = chros.ChROSIMUHandler(100, "~/output/imu/data")    # fused IMU message
    imu_handler.SetAccelerometerHandler(acc_handler)
    imu_handler.SetGyroscopeHandler(gyro_handler)
    imu_handler.SetMagnetometerHandler(mag_handler)
    ros_manager.RegisterHandler(imu_handler)
    ros_manager.Initialize()                                         # initialize once, after all handlers

    vis = chronoirr.ChVisualSystemIrrlicht()                        # Irrlicht review window
    vis.AttachSystem(sys)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("Sensors + ROS on a moving ground body")
    vis.Initialize()                                                # Initialize FIRST, then add scene elements
    vis.AddLogo()
    vis.AddSkyBox()
    vis.AddCamera(ch.ChVector3d(-8, -8, 4), ch.ChVector3d(0, 0, 0))
    vis.AddTypicalLights()

    time_step = 1e-3                                                 # integration step
    time_end = 100                                                   # simulation end time
    render_fps = 30                                                  # review render cadence
    render_every = max(1, round(1.0 / (render_fps * time_step)))    # physics steps per rendered frame

    ground_body.SetAngVelParent(ch.ChVector3d(0, 0, 0.1))           # spin the body so the sensors see motion


    while vis.Run() and sys.GetChTime() < time_end:
        vis.BeginScene(); vis.Render(); vis.EndScene()
        for _ in range(render_every):
            time = sys.GetChTime()
            sens_manager.Update()                                  # pump all sensors once per step
            if not ros_manager.Update(time, time_step):            # publish to ROS, break on shutdown
                break
            sys.DoStepDynamics(time_step)                          # advance physics
            if sys.GetChTime() >= time_end:
                break


if __name__ == "__main__":
    main()
