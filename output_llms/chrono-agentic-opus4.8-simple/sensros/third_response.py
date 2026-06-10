import os
import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.ros as chros
import pychrono.irrlicht as chronoirr


def main():
    sys = chrono.ChSystemNSC()                                        # NSC system for the sensed scene
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))      # no gravity: body held by velocity only
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # collision system (mesh has collision geometry)

    mmesh = chrono.ChTriangleMeshConnected()                          # triangle mesh container
    mmesh.LoadWavefrontMesh(chrono.GetChronoDataFile("sensor/geometries/cube.obj"), False, True)  # load the cube .obj
    mmesh.Transform(chrono.ChVector3d(0, 0, 0), chrono.ChMatrix33d(2.0))  # scale the mesh up by 2

    trimesh_shape = chrono.ChVisualShapeTriangleMesh()               # visual shape wrapping the mesh
    trimesh_shape.SetMesh(mmesh)                                      # attach the loaded mesh
    trimesh_shape.SetName("mesh_body")                               # name the visual shape
    trimesh_shape.SetMutable(False)                                   # static geometry, no per-step rebuild

    mesh_body = chrono.ChBody()                                       # the body the sensors observe
    mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))                      # at the world origin
    mesh_body.AddVisualShape(trimesh_shape)                           # add the mesh visual
    mesh_body.SetMass(1.0)                                           # finite mass so the body is dynamic (not fixed)
    mesh_body.SetName("mesh_body")                                   # name it (becomes a ROS frame id)
    sys.Add(mesh_body)                                                # add the mesh body to the system

    mesh_body.SetAngVelParent(chrono.ChVector3d(0, 0, 0.3))          # spin slowly about Z so sensors see motion

    manager = sens.ChSensorManager(sys)                              # sensor manager owns all sensors
    intensity = 1.0                                                   # point-light intensity
    manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100),
                                chrono.ChColor(intensity, intensity, intensity), 500.0)   # light for the camera
    manager.scene.AddPointLight(chrono.ChVector3f(9, 2.5, 100),
                                chrono.ChColor(intensity, intensity, intensity), 500.0)   # second point light

    offset_pose = chrono.ChFramed(chrono.ChVector3d(-8, 0, 1),
                                  chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))  # camera offset pose

    cam = sens.ChCameraSensor(mesh_body, 30, offset_pose, 1280, 720, 1.408)   # RGB camera on the mesh body
    cam.SetName("Camera Sensor")                                     # name the camera
    cam.SetLag(0)                                                     # no lag
    cam.SetCollectionWindow(0)                                        # instantaneous exposure
    cam.PushFilter(sens.ChFilterVisualize(1280, 720, "RGB Camera"))  # live RGB preview
    cam.PushFilter(sens.ChFilterRGBA8Access())                       # host access to the RGBA8 buffer
    cam.PushFilter(sens.ChFilterSave("cam/rgb/"))                    # save RGB PNGs (scored sensor output)
    manager.AddSensor(cam)                                           # register the camera

    lidar_pose = chrono.ChFramed(chrono.ChVector3d(-8, 0, 1),
                                 chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))  # 2D lidar offset pose
    horizontal_samples = 800                                         # horizontal beams
    vertical_samples = 1                                             # 2D lidar: a single scan line
    lidar = sens.ChLidarSensor(
        mesh_body,                                                   # attach the lidar to the mesh body
        5.0,                                                         # update rate (Hz)
        lidar_pose,                                                  # offset pose
        horizontal_samples,                                          # horizontal samples
        vertical_samples,                                            # vertical samples (1 -> 2D lidar)
        2 * chrono.CH_PI,                                            # 360 deg horizontal FOV
        0.0,                                                         # max vertical angle (0 for 2D)
        0.0,                                                         # min vertical angle (0 for 2D)
        100.0,                                                       # max range
        sens.LidarBeamShape_RECTANGULAR,                            # beam shape
        2,                                                          # sample radius
        0.003,                                                      # vertical divergence angle
        0.003,                                                      # horizontal divergence angle
        sens.LidarReturnMode_STRONGEST_RETURN,                      # return mode
    )
    lidar.SetName("Lidar Sensor")                                   # name the lidar
    lidar.SetLag(0)                                                  # no lag
    lidar.SetCollectionWindow(1.0 / 5.0)                            # collection window = 1 / update_rate
    lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth"))  # depth preview
    lidar.PushFilter(sens.ChFilterDIAccess())                       # host access to depth+intensity
    lidar.PushFilter(sens.ChFilterPCfromDepth())                   # convert depth -> XYZI point cloud
    lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))  # point-cloud preview
    lidar.PushFilter(sens.ChFilterXYZIAccess())                    # host access to the XYZI point cloud
    manager.AddSensor(lidar)                                        # register the 2D lidar

    gps_pose = chrono.ChFramed(chrono.ChVector3d(0, 0, 0),
                               chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))  # GPS/IMU offset pose
    gps = sens.ChGPSSensor(mesh_body, 10, gps_pose,
                           chrono.ChVector3d(-89.400, 43.070, 260.0), sens.ChNoiseNone())  # GPS sensor
    gps.SetName("GPS Sensor")                                       # name the GPS
    gps.SetLag(0)                                                   # no lag
    gps.SetCollectionWindow(0)                                      # instantaneous
    gps.PushFilter(sens.ChFilterGPSAccess())                        # host access to GPS data
    manager.AddSensor(gps)                                          # register the GPS

    acc = sens.ChAccelerometerSensor(mesh_body, 100, gps_pose, sens.ChNoiseNone())   # accelerometer
    acc.SetName("Accelerometer Sensor")                            # name it
    acc.SetLag(0)                                                   # no lag
    acc.SetCollectionWindow(0)                                      # instantaneous
    acc.PushFilter(sens.ChFilterAccelAccess())                      # host access to accel data
    manager.AddSensor(acc)                                          # register the accelerometer

    gyro = sens.ChGyroscopeSensor(mesh_body, 100, gps_pose, sens.ChNoiseNone())      # gyroscope
    gyro.SetName("Gyroscope Sensor")                               # name it
    gyro.SetLag(0)                                                  # no lag
    gyro.SetCollectionWindow(0)                                     # instantaneous
    gyro.PushFilter(sens.ChFilterGyroAccess())                     # host access to gyro data
    manager.AddSensor(gyro)                                        # register the gyroscope

    mag = sens.ChMagnetometerSensor(mesh_body, 100, gps_pose, sens.ChNoiseNone(),
                                    chrono.ChVector3d(-89.400, 43.070, 260.0))        # magnetometer
    mag.SetName("Magnetometer Sensor")                            # name it
    mag.SetLag(0)                                                  # no lag
    mag.SetCollectionWindow(0)                                     # instantaneous
    mag.PushFilter(sens.ChFilterMagnetAccess())                   # host access to mag data
    manager.AddSensor(mag)                                         # register the magnetometer

    ros_manager = chros.ChROSPythonManager()                      # the Python ROS manager
    ros_manager.RegisterHandler(chros.ChROSClockHandler())        # /clock first, time-syncs the graph
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, mesh_body, "~/output/mesh_body/state"))  # body pose/twist
    ros_manager.RegisterHandler(chros.ChROSCameraHandler(30, cam, "~/output/camera/data/image"))    # camera image
    ros_manager.RegisterHandler(chros.ChROSLidarHandler(lidar, "~/output/lidar/data/pointcloud"))   # lidar point cloud
    ros_manager.RegisterHandler(chros.ChROSGPSHandler(gps, "~/output/gps/data"))     # GPS NavSatFix

    acc_handler = chros.ChROSAccelerometerHandler(acc, "~/output/accelerometer/data")  # accel handler
    gyro_handler = chros.ChROSGyroscopeHandler(gyro, "~/output/gyroscope/data")         # gyro handler
    mag_handler = chros.ChROSMagnetometerHandler(mag, "~/output/magnetometer/data")     # mag handler
    ros_manager.RegisterHandler(acc_handler)                       # register accel
    ros_manager.RegisterHandler(gyro_handler)                     # register gyro
    ros_manager.RegisterHandler(mag_handler)                      # register mag

    imu_handler = chros.ChROSIMUHandler(100, "~/output/imu/data")  # fused IMU publisher
    imu_handler.SetAccelerometerHandler(acc_handler)              # feed accel
    imu_handler.SetGyroscopeHandler(gyro_handler)                # feed gyro
    imu_handler.SetMagnetometerHandler(mag_handler)              # feed mag
    ros_manager.RegisterHandler(imu_handler)                     # register the fused IMU

    ros_manager.Initialize()                                      # initialize ROS once, after all handlers

    vis = chronoirr.ChVisualSystemIrrlicht()                     # Irrlicht review window
    vis.AttachSystem(sys)                                         # attach the system
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)            # Z-up camera
    vis.SetWindowSize(1280, 720)                                 # window size
    vis.SetWindowTitle("Sensor + ROS scene")                    # window title
    vis.Initialize()                                             # initialize FIRST
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # logo after Initialize
    vis.AddSkyBox()                                              # sky box
    vis.AddCamera(chrono.ChVector3d(-12, 0, 4), chrono.ChVector3d(0, 0, 0))  # interactive camera
    vis.AddTypicalLights()                                       # standard lights

    time_step = 1e-3                                             # physics step
    sim_end = 10.0                                               # end time
    render_fps = 50.0                                            # review render cadence
    render_every = max(1, round(1.0 / (render_fps * time_step)))   # untagged cadence constant

    time = 0.0                                                   # sim time tracker
    while vis.Run() and time < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            manager.Update()                                    # pump sensors every physics step
            sys.DoStepDynamics(time_step)                       # advance physics
            time = sys.GetChTime()                              # current sim time
            if not ros_manager.Update(time, time_step):        # publish to ROS; stop if the graph shut down
                break
            if time >= sim_end:
                break


if __name__ == "__main__":
    main()
