"""
sensros — sensor + ROS2 bridge demo (PyChrono 9.0.x, Irrlicht).

Models a spinning ground body that carries a full sensor suite:
  - RGB camera (OptiX)
  - 3D Lidar sensor
  - 2D Lidar sensor (added in this revision)
  - GPS
  - Accelerometer, Gyroscope, Magnetometer (fused IMU)

All sensors publish over ROS2 via ChROSPythonManager. The body rotates at a
constant angular velocity so sensors see continuous motion. System: ChSystemNSC,
Z-up, no contact (pure-kinematic spinning body → no collision system needed).
"""

import os
import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens
import pychrono.ros as chros


# === Named constants ===
TIME_STEP   = 1e-3          # physics step (s)
SIM_END     = 5.0           # simulation duration (s)
RENDER_FPS  = 50.0          # Irrlicht render cadence
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# Spinning body geometry
BODY_MASS   = 1.0           # kg
BODY_SIZE   = chrono.ChVector3d(1.0, 0.1, 0.1)  # box half-extents (x,y,z) — full extents

# Sensor update rates (physical Hz)
CAM_RATE    = 30            # camera Hz
LIDAR_RATE  = 5             # 3-D lidar Hz
LIDAR2D_RATE = 5            # 2-D lidar Hz
GPS_RATE    = 10            # GPS Hz
IMU_RATE    = 100           # IMU (accel/gyro/mag) Hz

# GPS reference origin
GPS_REF     = chrono.ChVector3d(-89.400, 43.070, 260.0)

# Angular velocity of the spinning body (rad/s about Z)
ANG_VEL_Z   = 0.5

# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))  # no gravity: sensor showcase

# === Bodies ===
# Fixed "world" anchor body
world_body = chrono.ChBody()
world_body.SetFixed(True)
world_body.SetName("world")
sys.Add(world_body)

# Spinning sensor-carrier body
ground_body = chrono.ChBodyEasyBox(
    BODY_SIZE.x, BODY_SIZE.y, BODY_SIZE.z,
    BODY_MASS / (BODY_SIZE.x * BODY_SIZE.y * BODY_SIZE.z),  # density
    False, False,   # collide=False, visualize=False (we add shape manually)
)
ground_body.SetFixed(False)
ground_body.SetName("ground_body")
ground_body.SetPos(chrono.ChVector3d(0, 0, 1))
ground_body.SetAngVelParent(chrono.ChVector3d(0, 0, ANG_VEL_Z))
sys.Add(ground_body)

# Visual shape for the spinning body
vis_box = chrono.ChVisualShapeBox(BODY_SIZE.x, BODY_SIZE.y, BODY_SIZE.z)
vis_box.SetColor(chrono.ChColor(0.5, 0.7, 0.9))
ground_body.AddVisualShape(vis_box)

# === Sensor manager & lighting ===
manager = sens.ChSensorManager(sys)
intensity = 1.0
manager.scene.AddPointLight(
    chrono.ChVector3f(2, 2.5, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(9, 2.5, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)

# === Sensors ===

# --- Camera sensor ---
cam_offset = chrono.ChFramed(
    chrono.ChVector3d(-12, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
cam = sens.ChCameraSensor(
    ground_body,
    CAM_RATE,
    cam_offset,
    1280, 720,
    1.408,
)
cam.SetName("Camera Sensor")
cam.SetLag(0)
cam.SetCollectionWindow(0)
cam.PushFilter(sens.ChFilterVisualize(1280, 720, "RGB Camera"))
cam.PushFilter(sens.ChFilterRGBA8Access())
cam.PushFilter(sens.ChFilterSave("cam/camera/"))
manager.AddSensor(cam)

# --- 3D Lidar sensor ---
lidar_offset = chrono.ChFramed(
    chrono.ChVector3d(-12, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
lidar = sens.ChLidarSensor(
    ground_body,
    LIDAR_RATE,
    lidar_offset,
    800,                               # h_samples
    300,                               # v_samples
    2 * chrono.CH_PI,                  # horizontal FOV
    chrono.CH_PI / 12,                 # max_vert_angle
    -chrono.CH_PI / 6,                 # min_vert_angle
    100.0,                             # max_range
    sens.LidarBeamShape_RECTANGULAR,
    2,                                 # sample_radius
    0.003,                             # vert divergence
    0.003,                             # hori divergence
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(1.0 / LIDAR_RATE)
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterXYZIAccess())
manager.AddSensor(lidar)

# --- 2D Lidar sensor (single horizontal scan plane) ---
lidar2d_offset = chrono.ChFramed(
    chrono.ChVector3d(-8, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
lidar2d = sens.ChLidarSensor(
    ground_body,
    LIDAR2D_RATE,
    lidar2d_offset,
    800,                               # h_samples
    1,                                 # v_samples = 1 → 2D scan
    2 * chrono.CH_PI,                  # horizontal FOV
    0.0,                               # max_vert_angle = 0 (2D)
    0.0,                               # min_vert_angle = 0 (2D)
    100.0,                             # max_range
    sens.LidarBeamShape_RECTANGULAR,
    2,                                 # sample_radius
    0.003,                             # vert divergence
    0.003,                             # hori divergence
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar2d.SetName("2D Lidar Sensor")
lidar2d.SetLag(0)
lidar2d.SetCollectionWindow(1.0 / LIDAR2D_RATE)
lidar2d.PushFilter(sens.ChFilterDIAccess())
lidar2d.PushFilter(sens.ChFilterPCfromDepth())
lidar2d.PushFilter(sens.ChFilterXYZIAccess())
manager.AddSensor(lidar2d)

# --- GPS sensor ---
gps_offset = chrono.ChFramed(
    chrono.ChVector3d(-8, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
gps = sens.ChGPSSensor(
    ground_body,
    GPS_RATE,
    gps_offset,
    GPS_REF,
    sens.ChNoiseNone(),
)
gps.SetName("GPS Sensor")
gps.SetLag(0)
gps.SetCollectionWindow(0)
gps.PushFilter(sens.ChFilterGPSAccess())
manager.AddSensor(gps)

# --- Accelerometer ---
acc_offset = chrono.ChFramed(
    chrono.ChVector3d(-8, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
acc = sens.ChAccelerometerSensor(
    ground_body,
    IMU_RATE,
    acc_offset,
    sens.ChNoiseNone(),
)
acc.SetName("Accelerometer Sensor")
acc.SetLag(0)
acc.SetCollectionWindow(0)
acc.PushFilter(sens.ChFilterAccelAccess())
manager.AddSensor(acc)

# --- Gyroscope ---
gyro_offset = chrono.ChFramed(
    chrono.ChVector3d(-8, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
gyro = sens.ChGyroscopeSensor(
    ground_body,
    IMU_RATE,
    gyro_offset,
    sens.ChNoiseNone(),
)
gyro.SetName("Gyroscope Sensor")
gyro.SetLag(0)
gyro.SetCollectionWindow(0)
gyro.PushFilter(sens.ChFilterGyroAccess())
manager.AddSensor(gyro)

# --- Magnetometer ---
mag_offset = chrono.ChFramed(
    chrono.ChVector3d(-8, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
mag = sens.ChMagnetometerSensor(
    ground_body,
    IMU_RATE,
    mag_offset,
    sens.ChNoiseNone(),
    GPS_REF,
)
mag.SetName("Magnetometer Sensor")
mag.SetLag(0)
mag.SetCollectionWindow(0)
mag.PushFilter(sens.ChFilterMagnetAccess())
manager.AddSensor(mag)

# === ROS manager ===
ros_manager = chros.ChROSPythonManager()

# 1. Clock handler first
ros_manager.RegisterHandler(chros.ChROSClockHandler())

# 2. Camera handler
ros_manager.RegisterHandler(chros.ChROSCameraHandler(CAM_RATE, cam, "~/output/camera/data/image"))

# 3. 3D Lidar handler (point cloud)
ros_manager.RegisterHandler(chros.ChROSLidarHandler(
    lidar,
    "~/output/lidar/data/pointcloud",
    chros.ChROSLidarHandlerMessageType_POINT_CLOUD2,
))

# 4. 2D Lidar handler (laser scan)
ros_manager.RegisterHandler(chros.ChROSLidarHandler(
    lidar2d,
    "~/output/lidar2d/data/scan",
    chros.ChROSLidarHandlerMessageType_LASER_SCAN,
))

# 5. GPS handler
ros_manager.RegisterHandler(chros.ChROSGPSHandler(GPS_RATE, gps, "~/output/gps/data"))

# 6. Accelerometer, Gyroscope, Magnetometer handlers (also registered for fused IMU)
acc_handler  = chros.ChROSAccelerometerHandler(IMU_RATE, acc, "~/output/accelerometer/data")
gyro_handler = chros.ChROSGyroscopeHandler(IMU_RATE, gyro, "~/output/gyroscope/data")
mag_handler  = chros.ChROSMagnetometerHandler(IMU_RATE, mag, "~/output/magnetometer/data")
ros_manager.RegisterHandler(acc_handler)
ros_manager.RegisterHandler(gyro_handler)
ros_manager.RegisterHandler(mag_handler)

# 7. Fused IMU handler
imu_handler = chros.ChROSIMUHandler(IMU_RATE, "~/output/imu/data")
imu_handler.SetAccelerometerHandler(acc_handler)
imu_handler.SetGyroscopeHandler(gyro_handler)
imu_handler.SetMagnetometerHandler(mag_handler)
ros_manager.RegisterHandler(imu_handler)

# Initialize ROS manager ONCE, after all handlers registered
ros_manager.Initialize()

# === Visualization — Irrlicht ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("sensros — Sensor + ROS2 Bridge Demo")
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, -15, 3), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()

# === Review-only setup ===

# CSV logging setup

frame = 0  # Irrlicht frame counter

# === Main loop ===
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            manager.Update()                    # pump all sensors
            if not ros_manager.Update(sys.GetChTime(), TIME_STEP):
                break
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:      # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
