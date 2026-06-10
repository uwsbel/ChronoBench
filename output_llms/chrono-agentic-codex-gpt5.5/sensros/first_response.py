"""Moving Chrono sensor platform with ROS2 publishers.

This PyChrono NSC simulation creates a named moving ground_body carrying camera,
lidar, GPS, accelerometer, gyroscope, and magnetometer sensors.  A sensor manager
updates the sensor streams, while a ChROSPythonManager publishes the body, TF,
individual sensor topics, and a fused IMU topic during a real-time loop.
"""

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens
import pychrono.ros as chros


# === Constants ===
TIME_STEP = 0.005
SIM_END = 8.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once
BODY_MASS = 60.0
BODY_INERTIA = chrono.ChVector3d(8.0, 8.0, 8.0)
BODY_SIZE = chrono.ChVector3d(1.2, 0.35, 0.8)
BODY_START = chrono.ChVector3d(0.0, 0.0, 1.0)
BODY_LIN_VEL = chrono.ChVector3d(0.45, 0.0, 0.0)
BODY_ANG_VEL = chrono.ChVector3d(0.0, 0.0, 0.65)
GPS_REFERENCE = chrono.ChVector3d(-89.400, 43.070, 260.0)
CAMERA_RATE = 30.0
LIDAR_RATE = 5.0
IMU_RATE = 100.0
GPS_RATE = 10.0
ROS_BODY_RATE = 50.0


# === System & gravity ===
system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))
system.SetSolverType(chrono.ChSolver.Type_PSOR)
system.GetSolver().AsIterative().SetMaxIterations(60)


# === Bodies ===
world = chrono.ChBody()
world.SetName("base_link")
world.SetFixed(True)
system.AddBody(world)

ground_body = chrono.ChBody()
ground_body.SetName("ground_body")
ground_body.SetMass(BODY_MASS)
ground_body.SetInertiaXX(BODY_INERTIA)
ground_body.SetPos(BODY_START)
ground_body.SetPosDt(BODY_LIN_VEL)
ground_body.SetAngVelParent(BODY_ANG_VEL)
ground_body.EnableCollision(False)

box_visual = chrono.ChVisualShapeBox(BODY_SIZE)
box_visual.SetColor(chrono.ChColor(0.15, 0.45, 0.85))
ground_body.AddVisualShape(box_visual)

mesh_visual = chrono.ChVisualShapeModelFile()
mesh_visual.SetFilename(chrono.GetChronoDataFile("sensor/geometries/box.obj"))
ground_body.AddVisualShape(
    mesh_visual,
    chrono.ChFramed(
        chrono.ChVector3d(0.0, 0.0, 0.48),
        chrono.QuatFromAngleAxis(0.0, chrono.ChVector3d(0, 0, 1)),
    ),
)
system.AddBody(ground_body)

ground = ground_body  # cache: sensor and ROS handlers reuse the moving body


# === Sensor manager & sensors ===
manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 12), chrono.ChColor(1, 1, 1), 500.0)
manager.scene.AddPointLight(chrono.ChVector3f(-4, -3, 8), chrono.ChColor(0.7, 0.7, 0.8), 300.0)

sensor_mount = chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, 0.42), chrono.QUNIT)
camera_pose = chrono.ChFramed(
    chrono.ChVector3d(-1.4, 0.0, 0.55),
    chrono.QuatFromAngleAxis(0.12, chrono.ChVector3d(0, 1, 0)),
)
lidar_pose = chrono.ChFramed(chrono.ChVector3d(0.35, 0.0, 0.58), chrono.QUNIT)

camera = sens.ChCameraSensor(ground, CAMERA_RATE, camera_pose, 640, 480, 1.408)
camera.SetName("camera")
camera.SetLag(0)
camera.SetCollectionWindow(0)
camera.PushFilter(sens.ChFilterVisualize(640, 480, "ROS Camera"))
camera.PushFilter(sens.ChFilterRGBA8Access())
camera.PushFilter(sens.ChFilterSave("cam/rgb/"))
manager.AddSensor(camera)

lidar = sens.ChLidarSensor(
    ground,
    LIDAR_RATE,
    lidar_pose,
    320,
    32,
    2.0 * chrono.CH_PI,
    chrono.CH_PI / 12.0,
    -chrono.CH_PI / 12.0,
    60.0,
    sens.LidarBeamShape_RECTANGULAR,
    1,
    0.003,
    0.003,
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar.SetName("lidar")
lidar.SetLag(0)
lidar.SetCollectionWindow(1.0 / LIDAR_RATE)
lidar.PushFilter(sens.ChFilterVisualize(320, 32, "Raw Lidar Depth"))
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
lidar.PushFilter(sens.ChFilterXYZIAccess())
manager.AddSensor(lidar)

gps = sens.ChGPSSensor(ground, GPS_RATE, sensor_mount, GPS_REFERENCE, sens.ChNoiseNone())
gps.SetName("gps")
gps.SetLag(0)
gps.SetCollectionWindow(0)
gps.PushFilter(sens.ChFilterGPSAccess())
manager.AddSensor(gps)

accelerometer = sens.ChAccelerometerSensor(ground, IMU_RATE, sensor_mount, sens.ChNoiseNone())
accelerometer.SetName("accelerometer")
accelerometer.SetLag(0)
accelerometer.SetCollectionWindow(0)
accelerometer.PushFilter(sens.ChFilterAccelAccess())
manager.AddSensor(accelerometer)

gyroscope = sens.ChGyroscopeSensor(ground, IMU_RATE, sensor_mount, sens.ChNoiseNone())
gyroscope.SetName("gyroscope")
gyroscope.SetLag(0)
gyroscope.SetCollectionWindow(0)
gyroscope.PushFilter(sens.ChFilterGyroAccess())
manager.AddSensor(gyroscope)

magnetometer = sens.ChMagnetometerSensor(ground, IMU_RATE, sensor_mount, sens.ChNoiseNone(), GPS_REFERENCE)
magnetometer.SetName("magnetometer")
magnetometer.SetLag(0)
magnetometer.SetCollectionWindow(0)
magnetometer.PushFilter(sens.ChFilterMagnetAccess())
manager.AddSensor(magnetometer)


# === ROS publishers ===
ros_manager = chros.ChROSPythonManager()
ros_manager.RegisterHandler(chros.ChROSClockHandler())
ros_manager.RegisterHandler(chros.ChROSBodyHandler(ROS_BODY_RATE, ground, "~/output/ground_body"))

tf_handler = chros.ChROSTFHandler(ROS_BODY_RATE)
tf_handler.AddTransform(world, world.GetName(), ground, ground.GetName())
tf_handler.AddSensor(camera, ground.GetName(), "camera")
tf_handler.AddSensor(lidar, ground.GetName(), "lidar")
tf_handler.AddSensor(gps, ground.GetName(), "gps")
tf_handler.AddSensor(accelerometer, ground.GetName(), "accelerometer")
tf_handler.AddSensor(gyroscope, ground.GetName(), "gyroscope")
tf_handler.AddSensor(magnetometer, ground.GetName(), "magnetometer")
ros_manager.RegisterHandler(tf_handler)

ros_manager.RegisterHandler(chros.ChROSCameraHandler(CAMERA_RATE, camera, "~/output/camera/image"))
ros_manager.RegisterHandler(
    chros.ChROSLidarHandler(
        LIDAR_RATE,
        lidar,
        "~/output/lidar/point_cloud",
        chros.ChROSLidarHandlerMessageType_POINT_CLOUD2,
    )
)
ros_manager.RegisterHandler(chros.ChROSGPSHandler(GPS_RATE, gps, "~/output/gps/data"))
accel_handler = chros.ChROSAccelerometerHandler(IMU_RATE, accelerometer, "~/output/accelerometer/data")
gyro_handler = chros.ChROSGyroscopeHandler(IMU_RATE, gyroscope, "~/output/gyroscope/data")
mag_handler = chros.ChROSMagnetometerHandler(IMU_RATE, magnetometer, "~/output/magnetometer/data")
ros_manager.RegisterHandler(accel_handler)
ros_manager.RegisterHandler(gyro_handler)
ros_manager.RegisterHandler(mag_handler)
imu_handler = chros.ChROSIMUHandler(IMU_RATE, "~/output/imu/data", "imu")
imu_handler.SetAccelerometerHandler(accel_handler)
imu_handler.SetGyroscopeHandler(gyro_handler)
imu_handler.SetMagnetometerHandler(mag_handler)
ros_manager.RegisterHandler(imu_handler)
ros_manager.Initialize()


# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("PyChrono Sensor ROS Platform")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(4, -5, 3), chrono.ChVector3d(0, 0, 1))
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 20, 20, chrono.ChCoordsysd(), chrono.ChColor(0.35, 0.35, 0.35))


# === Main loop ===
timer = chrono.ChRealtimeStepTimer()

try:
    while vis.Run() and system.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(RENDER_EVERY):
            sim_time = system.GetChTime()  # cache: one time read per physics step
            manager.Update()
            if not ros_manager.Update(sim_time, TIME_STEP):
                vis.GetDevice().closeDevice()
                break
            system.DoStepDynamics(TIME_STEP)
            timer.Spin(TIME_STEP)
            if system.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError, OSError) as exc:
    print(f"Simulation failed: {exc}")
    raise
finally:
    pass
