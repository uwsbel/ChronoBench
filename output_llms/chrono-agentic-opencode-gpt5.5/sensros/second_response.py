"""Sensor ROS bridge demo with a rotating NSC body carrying Chrono sensors.

The scene uses a single free rigid body in a zero-gravity ChSystemNSC, plus fixed
obstacle boxes that give camera and lidar sensors visible geometry. A camera,
3D lidar, GPS, accelerometer, gyroscope, magnetometer, and an added 2D lidar are
attached to the rotating body and published through ChROSPythonManager handlers;
the 2D lidar publishes laser scans on ~/output/lidar2d/data/scan.
"""

import traceback

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens
import pychrono.ros as chros


# === Constants === compact physical and recording parameters for the sensor scene
TIME_STEP = 0.005
SIM_END = 1.2
RENDER_FPS = 12.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once
BODY_SIZE = chrono.ChVector3d(1.0, 0.35, 0.25)
BODY_DENSITY = 1000.0
OBSTACLE_DENSITY = 1000.0
SENSOR_RATE_CAMERA = 30.0
SENSOR_RATE_LIDAR = 5.0
SENSOR_RATE_IMU = 10.0
GPS_REFERENCE = chrono.ChVector3d(-89.400, 43.070, 260.0)


def configure_sensor(sensor, name, lag, collection_window):
    """Set common sensor timing fields once so handler setup stays compact."""
    sensor.SetName(name)
    sensor.SetLag(lag)
    sensor.SetCollectionWindow(collection_window)
    return sensor


# === System & Bodies === zero-gravity NSC system with a spinning sensor carrier
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

contact_mat = chrono.ChContactMaterialNSC()
contact_mat.SetFriction(0.8)
contact_mat.SetRestitution(0.0)

ground_body = chrono.ChBodyEasyBox(
    BODY_SIZE.x, BODY_SIZE.y, BODY_SIZE.z, BODY_DENSITY, True, True, contact_mat
)
ground_body.SetName("ground_body")
ground_body.SetPos(chrono.ChVector3d(0, 0, 1.0))
ground_body.SetAngVelParent(chrono.ChVector3d(0, 0, 0.35))
sys.AddBody(ground_body)

obstacles = []
for name, pos, size in [
    ("obstacle_front", chrono.ChVector3d(4.0, 0.0, 1.0), chrono.ChVector3d(0.35, 1.8, 1.0)),
    ("obstacle_left", chrono.ChVector3d(0.0, 3.0, 1.0), chrono.ChVector3d(1.8, 0.35, 1.0)),
    ("obstacle_right", chrono.ChVector3d(0.0, -3.0, 1.0), chrono.ChVector3d(1.8, 0.35, 1.0)),
]:
    obstacle = chrono.ChBodyEasyBox(size.x, size.y, size.z, OBSTACLE_DENSITY, True, True, contact_mat)
    obstacle.SetName(name)
    obstacle.SetPos(pos)
    obstacle.SetFixed(True)
    sys.AddBody(obstacle)
    obstacles.append(obstacle)

ground_body_cached = ground_body  # cache: parent body reused by all sensors and logs


# === Sensor Manager === OptiX sensors attached to the rotating body for ROS output
manager = sens.ChSensorManager(sys)
manager.scene.AddPointLight(
    chrono.ChVector3f(2, 2.5, 8), chrono.ChColor(1.0, 1.0, 1.0), 50.0
)
manager.scene.AddAreaLight(
    chrono.ChVector3f(0, 0, 4),
    chrono.ChColor(0.8, 0.8, 0.8),
    30.0,
    chrono.ChVector3f(2, 0, 0),
    chrono.ChVector3f(0, 2, 0),
)

camera = sens.ChCameraSensor(
    ground_body_cached,
    SENSOR_RATE_CAMERA,
    chrono.ChFramed(
        chrono.ChVector3d(-2.0, 0.0, 0.6),
        chrono.QuatFromAngleAxis(0.10, chrono.ChVector3d(0, 1, 0)),
    ),
    320,
    240,
    1.408,
)
configure_sensor(camera, "Camera Sensor", 0, 0)
camera.PushFilter(sens.ChFilterVisualize(320, 240, "Camera Sensor"))
camera.PushFilter(sens.ChFilterRGBA8Access())
camera.PushFilter(sens.ChFilterSave("cam/rgb/"))
manager.AddSensor(camera)

lidar = sens.ChLidarSensor(
    ground_body_cached,
    SENSOR_RATE_LIDAR,
    chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, 0.45), chrono.QUNIT),
    180,
    32,
    2 * chrono.CH_PI,
    chrono.CH_PI / 12,
    -chrono.CH_PI / 6,
    100.0,
    sens.LidarBeamShape_RECTANGULAR,
    2,
    0.003,
    0.003,
    sens.LidarReturnMode_STRONGEST_RETURN,
)
configure_sensor(lidar, "Lidar Sensor", 0, 1.0 / SENSOR_RATE_LIDAR)
lidar.PushFilter(sens.ChFilterVisualize(180, 32, "Raw Lidar Depth"))
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterVisualizePointCloud(320, 240, 1.0, "Lidar Point Cloud"))
lidar.PushFilter(sens.ChFilterXYZIAccess())
manager.AddSensor(lidar)

lidar2d = sens.ChLidarSensor(
    ground_body_cached,
    SENSOR_RATE_LIDAR,
    chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, 0.35), chrono.QUNIT),
    180,
    1,
    2 * chrono.CH_PI,
    0.0,
    0.0,
    100.0,
    sens.LidarBeamShape_RECTANGULAR,
    1,
    0.003,
    0.003,
    sens.LidarReturnMode_STRONGEST_RETURN,
)
configure_sensor(lidar2d, "2D Lidar Sensor", 0, 1.0 / SENSOR_RATE_LIDAR)
lidar2d.PushFilter(sens.ChFilterDIAccess())
lidar2d.PushFilter(sens.ChFilterPCfromDepth())
lidar2d.PushFilter(sens.ChFilterVisualizePointCloud(320, 240, 1.0, "2D Lidar Point Cloud"))
lidar2d.PushFilter(sens.ChFilterXYZIAccess())
manager.AddSensor(lidar2d)

imu_pose = chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, 0.25), chrono.QUNIT)
noise_none = sens.ChNoiseNone()
gps = sens.ChGPSSensor(ground_body_cached, SENSOR_RATE_IMU, imu_pose, GPS_REFERENCE, noise_none)
configure_sensor(gps, "GPS Sensor", 0, 0)
gps.PushFilter(sens.ChFilterGPSAccess())
manager.AddSensor(gps)

accelerometer = sens.ChAccelerometerSensor(ground_body_cached, SENSOR_RATE_IMU, imu_pose, noise_none)
configure_sensor(accelerometer, "Accelerometer Sensor", 0, 0)
accelerometer.PushFilter(sens.ChFilterAccelAccess())
manager.AddSensor(accelerometer)

gyroscope = sens.ChGyroscopeSensor(ground_body_cached, SENSOR_RATE_IMU, imu_pose, noise_none)
configure_sensor(gyroscope, "Gyroscope Sensor", 0, 0)
gyroscope.PushFilter(sens.ChFilterGyroAccess())
manager.AddSensor(gyroscope)

magnetometer = sens.ChMagnetometerSensor(
    ground_body_cached, SENSOR_RATE_IMU, imu_pose, noise_none, GPS_REFERENCE
)
configure_sensor(magnetometer, "Magnetometer Sensor", 0, 0)
magnetometer.PushFilter(sens.ChFilterMagnetAccess())
manager.AddSensor(magnetometer)


# === ROS Bridge === register clock first, then sensor handlers and fused IMU
ros_manager = chros.ChROSPythonManager()
ros_manager.RegisterHandler(chros.ChROSClockHandler())
ros_manager.RegisterHandler(chros.ChROSCameraHandler(SENSOR_RATE_CAMERA, camera, "~/output/camera/data/image"))
ros_manager.RegisterHandler(chros.ChROSLidarHandler(SENSOR_RATE_LIDAR, lidar, "~/output/lidar/data/pointcloud"))
ros_manager.RegisterHandler(
    chros.ChROSLidarHandler(
        SENSOR_RATE_LIDAR,
        lidar2d,
        "~/output/lidar2d/data/scan",
        chros.ChROSLidarHandlerMessageType_LASER_SCAN,
    )
)
ros_manager.RegisterHandler(chros.ChROSGPSHandler(SENSOR_RATE_IMU, gps, "~/output/gps/data"))
accel_handler = chros.ChROSAccelerometerHandler(
    SENSOR_RATE_IMU, accelerometer, "~/output/accelerometer/data"
)
gyro_handler = chros.ChROSGyroscopeHandler(SENSOR_RATE_IMU, gyroscope, "~/output/gyroscope/data")
mag_handler = chros.ChROSMagnetometerHandler(
    SENSOR_RATE_IMU, magnetometer, "~/output/magnetometer/data"
)
ros_manager.RegisterHandler(accel_handler)
ros_manager.RegisterHandler(gyro_handler)
ros_manager.RegisterHandler(mag_handler)
imu_handler = chros.ChROSIMUHandler(SENSOR_RATE_IMU, "~/output/imu/data")
imu_handler.SetAccelerometerHandler(accel_handler)
imu_handler.SetGyroscopeHandler(gyro_handler)
imu_handler.SetMagnetometerHandler(mag_handler)
ros_manager.RegisterHandler(imu_handler)
ros_manager.Initialize()


# === Visualization === Irrlicht review window distinct from the ROS sensor streams
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("PyChrono sensros 2D lidar ROS bridge")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(5.0, -6.0, 3.2), chrono.ChVector3d(0.0, 0.0, 1.0))
vis.AddTypicalLights()
vis.AddGrid(
    0.5,
    0.5,
    24,
    24,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    chrono.ChColor(0.4, 0.4, 0.4),
)


# === Main Loop === render, pump sensors, publish ROS, and advance dynamics
try:

    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(RENDER_EVERY):
            sim_time = sys.GetChTime()
            body_pos = ground_body_cached.GetPos()  # cache: reused for logging fields
            body_rot = ground_body_cached.GetRot()  # cache: reused for logging fields
            body_angvel = ground_body_cached.GetAngVelParent()  # cache: reused for logging fields

            manager.Update()

            if not ros_manager.Update(sim_time, TIME_STEP):
                raise RuntimeError("ROS manager stopped during sensor publishing")
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break

except (OSError, IOError) as exc:  # system I/O failure while running visualization or ROS
    traceback.print_exc()
    raise
except (RuntimeError, ValueError) as exc:  # ROS shutdown, sensor setup, or invalid state failure
    traceback.print_exc()
    raise
finally:
    pass
