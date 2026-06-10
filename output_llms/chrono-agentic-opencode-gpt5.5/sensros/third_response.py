"""Sensor/ROS demonstration with an NSC system, a rotating mesh body, and onboard sensors.

The simulation creates a single mesh body carrying RGB camera, 2D lidar, GPS,
accelerometer, gyroscope, and magnetometer sensors. ROS2 handlers publish the
sensor streams, body state, TF, clock, and fused IMU data while Irrlicht displays
the rotating body and the sensor manager updates every physics step.
"""

import traceback

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens
import pychrono.ros as chros


# === Constants === named parameters keep the sensor scene reproducible
time_step = 1.0e-3
sim_end = 2.0
render_fps = 30.0
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once
camera_rate = 30.0
lidar_rate = 5.0
imu_rate = 10.0
body_rate = 25.0
mesh_density = 1000.0
horizontal_samples = 800
vertical_samples = 1


# === System & Body === zero gravity lets the mesh carrier spin in place for sensor motion
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

contact_mat = chrono.ChContactMaterialNSC()
contact_mat.SetFriction(0.6)
contact_mat.SetRestitution(0.0)

base_link = chrono.ChBody()
base_link.SetName("base_link")
base_link.SetFixed(True)
sys.Add(base_link)

mesh_file = chrono.GetChronoDataFile("models/red_teapot.obj")
mesh_body = chrono.ChBodyEasyMesh(mesh_file, mesh_density, True, True, True, contact_mat)
mesh_body.SetName("ground_body")
mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))
mesh_body.SetAngVelParent(chrono.ChVector3d(0, 0, 1.5))
sys.Add(mesh_body)
sensor_body = mesh_body  # cache: all sensors and ROS handlers reuse the same carrier body


# === Sensor Manager === required sensor filters are scored-core outputs and ROS data sources
manager = sens.ChSensorManager(sys)
manager.scene.AddPointLight(
    chrono.ChVector3f(2, 2.5, 10),
    chrono.ChColor(1.0, 1.0, 1.0),
    100.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(-3, -3, 8),
    chrono.ChColor(0.8, 0.8, 0.8),
    100.0,
)

camera_offset = chrono.ChFramed(
    chrono.ChVector3d(-5, 0, 2),
    chrono.QuatFromAngleAxis(0.25, chrono.ChVector3d(0, 1, 0)),
)
camera = sens.ChCameraSensor(sensor_body, camera_rate, camera_offset, 1280, 720, 1.408)
camera.SetName("Camera Sensor")
camera.SetLag(0)
camera.SetCollectionWindow(0)
camera.PushFilter(sens.ChFilterVisualize(640, 360, "RGB Camera"))
camera.PushFilter(sens.ChFilterRGBA8Access())
camera.PushFilter(sens.ChFilterSave("cam/rgb/"))
manager.AddSensor(camera)

lidar_offset = chrono.ChFramed(
    chrono.ChVector3d(0, 0, 0.5),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
lidar = sens.ChLidarSensor(
    sensor_body,
    lidar_rate,
    lidar_offset,
    horizontal_samples,
    vertical_samples,
    2 * chrono.CH_PI,
    0.0,
    0.0,
    100.0,
    sens.LidarBeamShape_RECTANGULAR,
    2,
    0.003,
    0.003,
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar.SetName("2D Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(1.0 / lidar_rate)
lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth"))
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
lidar.PushFilter(sens.ChFilterXYZIAccess())
manager.AddSensor(lidar)

sensor_offset = chrono.ChFramed(chrono.VNULL, chrono.QUNIT)
gps_reference = chrono.ChVector3d(-89.400, 43.070, 260.0)
noise_none = sens.ChNoiseNone()

gps = sens.ChGPSSensor(sensor_body, imu_rate, sensor_offset, gps_reference, noise_none)
gps.SetName("GPS Sensor")
gps.SetLag(0)
gps.SetCollectionWindow(0)
gps.PushFilter(sens.ChFilterGPSAccess())
manager.AddSensor(gps)

accelerometer = sens.ChAccelerometerSensor(sensor_body, imu_rate, sensor_offset, noise_none)
accelerometer.SetName("Accelerometer Sensor")
accelerometer.SetLag(0)
accelerometer.SetCollectionWindow(0)
accelerometer.PushFilter(sens.ChFilterAccelAccess())
manager.AddSensor(accelerometer)

gyroscope = sens.ChGyroscopeSensor(sensor_body, imu_rate, sensor_offset, noise_none)
gyroscope.SetName("Gyroscope Sensor")
gyroscope.SetLag(0)
gyroscope.SetCollectionWindow(0)
gyroscope.PushFilter(sens.ChFilterGyroAccess())
manager.AddSensor(gyroscope)

magnetometer = sens.ChMagnetometerSensor(sensor_body, imu_rate, sensor_offset, noise_none, gps_reference)
magnetometer.SetName("Magnetometer Sensor")
magnetometer.SetLag(0)
magnetometer.SetCollectionWindow(0)
magnetometer.PushFilter(sens.ChFilterMagnetAccess())
manager.AddSensor(magnetometer)


# === ROS Bridge === handlers publish the body, TF, sensors, and fused IMU streams
ros_manager = chros.ChROSPythonManager()
ros_manager.RegisterHandler(chros.ChROSClockHandler())
ros_manager.RegisterHandler(chros.ChROSBodyHandler(body_rate, sensor_body, "~/output/body"))

tf_handler = chros.ChROSTFHandler(body_rate)
tf_handler.AddTransform(base_link, base_link.GetName(), sensor_body, sensor_body.GetName())
ros_manager.RegisterHandler(tf_handler)

ros_manager.RegisterHandler(chros.ChROSCameraHandler(camera_rate, camera, "~/output/camera/image"))
ros_manager.RegisterHandler(
    chros.ChROSLidarHandler(lidar_rate, lidar, "~/output/lidar/scan", chros.ChROSLidarHandlerMessageType_LASER_SCAN)
)
ros_manager.RegisterHandler(chros.ChROSGPSHandler(imu_rate, gps, "~/output/gps/fix"))

accel_handler = chros.ChROSAccelerometerHandler(imu_rate, accelerometer, "~/output/accelerometer/data")
gyro_handler = chros.ChROSGyroscopeHandler(imu_rate, gyroscope, "~/output/gyroscope/data")
mag_handler = chros.ChROSMagnetometerHandler(imu_rate, magnetometer, "~/output/magnetometer/data")
ros_manager.RegisterHandler(accel_handler)
ros_manager.RegisterHandler(gyro_handler)
ros_manager.RegisterHandler(mag_handler)

imu_handler = chros.ChROSIMUHandler(imu_rate, "~/output/imu/data", "ground_body_imu")
imu_handler.SetAccelerometerHandler(accel_handler)
imu_handler.SetGyroscopeHandler(gyro_handler)
imu_handler.SetMagnetometerHandler(mag_handler)
ros_manager.RegisterHandler(imu_handler)
ros_manager.Initialize()


# === Visualization === Irrlicht review window is separate from the OptiX sensors
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Chrono Sensor ROS")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(4, -6, 3), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AddGrid(
    0.5,
    0.5,
    20,
    20,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, -0.6), chrono.QUNIT),
    chrono.ChColor(0.4, 0.4, 0.4),
)


# === Main Loop === sensors update before ROS publishes and dynamics advances
frame = 0
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            time = sys.GetChTime()  # cache: reused by ROS, logging, and stop checks this step
            pos = sensor_body.GetPos()  # cache: body pose reused for review logging
            rot = sensor_body.GetRot()  # cache: orientation reused for review logging
            manager.Update()
            if not ros_manager.Update(time, time_step):
                break
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
        else:
            continue
        break
except (RuntimeError, ValueError) as exc:  # solver divergence / invalid state
    traceback.print_exc()
    raise
except (OSError, IOError) as exc:  # disk, renderer, or ROS resource error
    traceback.print_exc()
    raise
finally:
    pass
