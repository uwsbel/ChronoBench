"""
sensros — PyChrono sensor + ROS2 bridge demo.

Models a ground body that slowly rotates in place, carrying six sensors:
  - RGB camera (ChCameraSensor)
  - 3D lidar (ChLidarSensor)
  - GPS (ChGPSSensor)
  - Accelerometer (ChAccelerometerSensor)
  - Gyroscope (ChGyroscopeSensor)
  - Magnetometer (ChMagnetometerSensor)

A ChROSPythonManager publishes every sensor's data to ROS2 topics, plus a
fused IMU topic built from the acc/gyro/mag handlers, a body-pose topic, and
the simulation clock.

System: ChSystemNSC (no contact — pure kinematic rigid body).
Expected behavior: body rotates, all sensors update at their physical rates,
ROS topics appear on the graph, simulation runs for sim_end seconds.
"""

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens
import pychrono.ros as chros

# === Named constants ===
TIME_STEP   = 1e-3        # physics time step [s]
SIM_END     = 30.0        # simulation duration [s]
RENDER_FPS  = 50.0        # Irrlicht render rate [Hz]
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# Sensor update rates (physical Hz — never 1/time_step)
CAM_HZ      = 30
LIDAR_HZ    = 5
GPS_HZ      = 10
IMU_HZ      = 100

# Body spin rate
ANG_VEL_Z   = 0.5  # rad/s about Z axis

# GPS reference origin (lat/lon/alt)
GPS_REF = chrono.ChVector3d(-89.400, 43.070, 260.0)

# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
# No collision system needed — pure MBS, no contact shapes.

# === Bodies ===
# Ground / moving body that carries all sensors.
ground_body = chrono.ChBody()
ground_body.SetName("ground_body")
ground_body.SetFixed(False)
ground_body.SetMass(10.0)
ground_body.SetInertiaXX(chrono.ChVector3d(1.0, 1.0, 1.0))
ground_body.SetPos(chrono.ChVector3d(0, 0, 0))
# Give the body a slow spin so sensors see non-trivial motion.
ground_body.SetAngVelParent(chrono.ChVector3d(0, 0, ANG_VEL_Z))

# Visual shape: a box to represent the sensor platform
box_shape = chrono.ChVisualShapeBox(0.4, 0.4, 0.1)
box_shape.SetColor(chrono.ChColor(0.4, 0.4, 0.8))
ground_body.AddVisualShape(box_shape)
sys.AddBody(ground_body)

# Add a mesh / visual reference object so the camera has something to see.
floor_body = chrono.ChBody()
floor_body.SetName("floor")
floor_body.SetFixed(True)
floor_body.SetMass(1.0)
floor_body.SetInertiaXX(chrono.ChVector3d(1.0, 1.0, 1.0))
floor_body.SetPos(chrono.ChVector3d(0, 0, -0.5))
floor_shape = chrono.ChVisualShapeBox(10.0, 10.0, 0.05)
floor_shape.SetColor(chrono.ChColor(0.6, 0.6, 0.6))
floor_body.AddVisualShape(floor_shape)
sys.AddBody(floor_body)

# === Sensor manager ===
manager = sens.ChSensorManager(sys)
intensity = 1.0
manager.scene.AddPointLight(
    chrono.ChVector3f(2.0, 2.5, 100.0),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(-2.0, -2.5, 100.0),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)

# === Camera sensor ===
cam_offset = chrono.ChFramed(
    chrono.ChVector3d(-5.0, 0.0, 2.0),
    chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0)),
)
cam = sens.ChCameraSensor(
    ground_body,     # attach to the rotating body
    CAM_HZ,
    cam_offset,
    1280, 720,
    1.408,
)
cam.SetName("Camera Sensor")
cam.SetLag(0)
cam.SetCollectionWindow(0)
cam.PushFilter(sens.ChFilterVisualize(1280, 720, "RGB Camera"))
cam.PushFilter(sens.ChFilterRGBA8Access())
cam.PushFilter(sens.ChFilterSave("cam/rgb/"))
manager.AddSensor(cam)

# === Lidar sensor ===
lidar_offset = chrono.ChFramed(
    chrono.ChVector3d(-12.0, 0.0, 1.0),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
lidar = sens.ChLidarSensor(
    ground_body,
    LIDAR_HZ,
    lidar_offset,
    800,                              # h_samples
    300,                              # v_samples
    2 * chrono.CH_PI,                 # horizontal_fov
    chrono.CH_PI / 12,                # max_vert_angle
    -chrono.CH_PI / 6,               # min_vert_angle
    100.0,                            # max_range
    sens.LidarBeamShape_RECTANGULAR,
    2,                                # sample_radius
    0.003,                            # vert divergence_angle
    0.003,                            # hori divergence_angle
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(1.0 / LIDAR_HZ)
lidar.PushFilter(sens.ChFilterVisualize(800, 300, "Raw Lidar Depth"))
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
lidar.PushFilter(sens.ChFilterXYZIAccess())
manager.AddSensor(lidar)

# === GPS sensor ===
gps_offset = chrono.ChFramed(
    chrono.ChVector3d(-8.0, 0.0, 1.0),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
gps = sens.ChGPSSensor(
    ground_body,
    GPS_HZ,
    gps_offset,
    GPS_REF,
    sens.ChNoiseNone(),
)
gps.SetName("GPS Sensor")
gps.SetLag(0)
gps.SetCollectionWindow(0)
gps.PushFilter(sens.ChFilterGPSAccess())
manager.AddSensor(gps)

# === Accelerometer sensor ===
acc_offset = chrono.ChFramed(
    chrono.ChVector3d(0.0, 0.0, 0.0),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
acc = sens.ChAccelerometerSensor(ground_body, IMU_HZ, acc_offset, sens.ChNoiseNone())
acc.SetName("Accelerometer Sensor")
acc.SetLag(0)
acc.SetCollectionWindow(0)
acc.PushFilter(sens.ChFilterAccelAccess())
manager.AddSensor(acc)

# === Gyroscope sensor ===
gyro_offset = chrono.ChFramed(
    chrono.ChVector3d(0.0, 0.0, 0.0),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
gyro = sens.ChGyroscopeSensor(ground_body, IMU_HZ, gyro_offset, sens.ChNoiseNone())
gyro.SetName("Gyroscope Sensor")
gyro.SetLag(0)
gyro.SetCollectionWindow(0)
gyro.PushFilter(sens.ChFilterGyroAccess())
manager.AddSensor(gyro)

# === Magnetometer sensor ===
mag_offset = chrono.ChFramed(
    chrono.ChVector3d(0.0, 0.0, 0.0),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
mag = sens.ChMagnetometerSensor(ground_body, IMU_HZ, mag_offset, sens.ChNoiseNone(), GPS_REF)
mag.SetName("Magnetometer Sensor")
mag.SetLag(0)
mag.SetCollectionWindow(0)
mag.PushFilter(sens.ChFilterMagnetAccess())
manager.AddSensor(mag)

# === ROS manager ===
ros_manager = chros.ChROSPythonManager()

# Clock handler FIRST
ros_manager.RegisterHandler(chros.ChROSClockHandler())

# Body pose handler
ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, ground_body, "~/output/body/state"))

# Camera handler
ros_manager.RegisterHandler(chros.ChROSCameraHandler(CAM_HZ, cam, "~/output/camera/image"))

# Lidar handler
ros_manager.RegisterHandler(chros.ChROSLidarHandler(lidar, "~/output/lidar/point_cloud"))

# GPS handler
ros_manager.RegisterHandler(chros.ChROSGPSHandler(gps, "~/output/gps/fix"))

# IMU individual handlers
acc_handler = chros.ChROSAccelerometerHandler(acc, "~/output/accelerometer/data")
gyro_handler = chros.ChROSGyroscopeHandler(gyro, "~/output/gyroscope/data")
mag_handler  = chros.ChROSMagnetometerHandler(mag, "~/output/magnetometer/data")
ros_manager.RegisterHandler(acc_handler)
ros_manager.RegisterHandler(gyro_handler)
ros_manager.RegisterHandler(mag_handler)

# Fused IMU handler
imu_handler = chros.ChROSIMUHandler(IMU_HZ, "~/output/imu/data")
imu_handler.SetAccelerometerHandler(acc_handler)
imu_handler.SetGyroscopeHandler(gyro_handler)
imu_handler.SetMagnetometerHandler(mag_handler)
ros_manager.RegisterHandler(imu_handler)

# Initialize ROS manager ONCE, after all registration
ros_manager.Initialize()

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("sensros — Sensor + ROS2 Demo")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(8.0, -8.0, 5.0), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AddGrid(
    0.5, 0.5, 40, 40,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, -0.5), chrono.QUNIT),
    chrono.ChColor(0.4, 0.4, 0.4),
)


# === Main loop ===
frame = 0
while vis.Run() and sys.GetChTime() < SIM_END:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        manager.Update()                              # pump sensors — every physics step
        if not ros_manager.Update(sys.GetChTime(), TIME_STEP):
            break
        sys.DoStepDynamics(TIME_STEP)
        if sys.GetChTime() >= SIM_END:
            break
