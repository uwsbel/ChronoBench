"""
sensros — Sensor + ROS2 Integration Demo (PyChrono 9.0.x, Irrlicht).

Models a moving ground body (spinning box) carrying six sensors:
  - ChCameraSensor      → ChROSCameraHandler
  - ChLidarSensor       → ChROSLidarHandler
  - ChGPSSensor         → ChROSGPSHandler
  - ChAccelerometerSensor → ChROSAccelerometerHandler (also fed into ChROSIMUHandler)
  - ChGyroscopeSensor   → ChROSGyroscopeHandler       (also fed into ChROSIMUHandler)
  - ChMagnetometerSensor → ChROSMagnetometerHandler   (also fed into ChROSIMUHandler)

The ground body spins about Z so all sensors observe real motion.
A ChROSPythonManager publishes all sensor streams plus a /clock topic.
System: ChSystemNSC (no gravity), kinematic angular motion.
"""

# === Imports ===
import os
import math
import csv
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens
import pychrono.ros as chros

# === Simulation constants ===
TIME_STEP   = 1e-3          # physics time step [s]
SIM_END     = 20.0          # simulation duration [s]
RENDER_FPS  = 50.0          # Irrlicht render rate [fps]
SPIN_OMEGA  = 0.5           # body angular velocity [rad/s] — Z-axis spin

# Sensor parameters (physical rates in Hz — never 1/dt)
CAM_RATE    = 30            # camera update rate [Hz]
LIDAR_RATE  = 5             # lidar update rate [Hz]
GPS_RATE    = 10            # GPS update rate [Hz]
IMU_RATE    = 100           # IMU (accel/gyro/mag) update rate [Hz]

# GPS/magnetometer reference origin (lat, lon, alt)
GPS_REF = chrono.ChVector3d(-89.400, 43.070, 260.0)

# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))  # kinematic — no gravity

# Collision system required: bodies have collision shapes for sensor rendering
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Precomputed render cadence constant (precomputed once before the loop)
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))

# === Bodies ===
mat_nsc = chrono.ChContactMaterialNSC()
mat_nsc.SetFriction(0.5)
mat_nsc.SetRestitution(0.0)

# Sensor platform body — spun about Z at constant rate
ground_body = chrono.ChBody()
ground_body.SetName("ground")
ground_body.SetFixed(False)
ground_body.SetMass(10.0)
ground_body.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))
ground_body.SetPos(chrono.ChVector3d(0, 0, 0))
ground_body.SetAngVelParent(chrono.ChVector3d(0, 0, SPIN_OMEGA))  # constant Z-spin

# Box visual + collision so OptiX sensor camera can render it
box_shape = chrono.ChVisualShapeBox(1.0, 1.0, 0.2)
box_shape.SetColor(chrono.ChColor(0.5, 0.7, 0.9))
ground_body.AddVisualShape(box_shape)
col_box = chrono.ChCollisionShapeBox(mat_nsc, 1.0, 1.0, 0.2)
ground_body.AddCollisionShape(col_box)
ground_body.EnableCollision(True)
sys.AddBody(ground_body)

# Fixed floor for visual context
floor_body = chrono.ChBody()
floor_body.SetFixed(True)
floor_body.SetName("floor")
floor_body.SetPos(chrono.ChVector3d(0, 0, -0.5))
floor_shape = chrono.ChVisualShapeBox(10.0, 10.0, 0.1)
floor_shape.SetColor(chrono.ChColor(0.4, 0.4, 0.4))
floor_body.AddVisualShape(floor_shape)
col_floor = chrono.ChCollisionShapeBox(mat_nsc, 10.0, 10.0, 0.1)
floor_body.AddCollisionShape(col_floor)
floor_body.EnableCollision(True)
sys.AddBody(floor_body)

# === Sensor Manager ===
sens_manager = sens.ChSensorManager(sys)
# Point lights for camera sensor rendering (ChVector3f required — not ChVector3d)
intensity = 1.0
sens_manager.scene.AddPointLight(
    chrono.ChVector3f(2, 2.5, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)
sens_manager.scene.AddPointLight(
    chrono.ChVector3f(-2, -2.5, 10),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)

# === Camera sensor ===
cam_offset = chrono.ChFramed(
    chrono.ChVector3d(-5, 0, 2),
    chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0)),
)
cam = sens.ChCameraSensor(ground_body, CAM_RATE, cam_offset, 1280, 720, 1.408)
cam.SetName("Camera Sensor")
cam.SetLag(0)
cam.SetCollectionWindow(0)
cam.PushFilter(sens.ChFilterVisualize(1280, 720, "RGB Camera"))
cam.PushFilter(sens.ChFilterRGBA8Access())
cam.PushFilter(sens.ChFilterSave("cam/rgb/"))
sens_manager.AddSensor(cam)

# === Lidar sensor — data-only filter chain (no point-cloud visualization window) ===
lidar_offset = chrono.ChFramed(
    chrono.ChVector3d(0, 0, 0.5),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
lidar = sens.ChLidarSensor(
    ground_body,
    LIDAR_RATE,
    lidar_offset,
    800,                              # h_samples
    16,                               # v_samples
    2 * chrono.CH_PI,                # horizontal_fov
    chrono.CH_PI / 12,               # max_vert_angle
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
lidar.SetCollectionWindow(1.0 / LIDAR_RATE)  # lidar collection window = 1/update_rate
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterXYZIAccess())
sens_manager.AddSensor(lidar)

# === GPS sensor (4-arg constructor: body, rate, offset, ref_origin, noise_model) ===
gps_offset = chrono.ChFramed(
    chrono.ChVector3d(0, 0, 0.1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
gps = sens.ChGPSSensor(ground_body, GPS_RATE, gps_offset, GPS_REF, sens.ChNoiseNone())
gps.SetName("GPS Sensor")
gps.SetLag(0)
gps.SetCollectionWindow(0)
gps.PushFilter(sens.ChFilterGPSAccess())
sens_manager.AddSensor(gps)

# === IMU sensors (accelerometer, gyroscope, magnetometer) — shared offset pose ===
imu_offset = chrono.ChFramed(
    chrono.ChVector3d(0, 0, 0),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)

acc = sens.ChAccelerometerSensor(ground_body, IMU_RATE, imu_offset, sens.ChNoiseNone())
acc.SetName("IMU Accelerometer")
acc.SetLag(0)
acc.SetCollectionWindow(0)
acc.PushFilter(sens.ChFilterAccelAccess())
sens_manager.AddSensor(acc)

gyro = sens.ChGyroscopeSensor(ground_body, IMU_RATE, imu_offset, sens.ChNoiseNone())
gyro.SetName("IMU Gyroscope")
gyro.SetLag(0)
gyro.SetCollectionWindow(0)
gyro.PushFilter(sens.ChFilterGyroAccess())
sens_manager.AddSensor(gyro)

mag = sens.ChMagnetometerSensor(ground_body, IMU_RATE, imu_offset, sens.ChNoiseNone(), GPS_REF)
mag.SetName("IMU Magnetometer")
mag.SetLag(0)
mag.SetCollectionWindow(0)
mag.PushFilter(sens.ChFilterMagnetAccess())
sens_manager.AddSensor(mag)

# === ROS Manager — ChROSPythonManager required for Python-subclass handlers ===
ros_manager = chros.ChROSPythonManager()

# 1. Clock handler first — publishes /clock so ROS graph is time-synced
ros_manager.RegisterHandler(chros.ChROSClockHandler())

# 2. Camera handler
cam_handler = chros.ChROSCameraHandler(CAM_RATE, cam, "~/output/camera/image")
ros_manager.RegisterHandler(cam_handler)

# 3. Lidar handler
lidar_handler = chros.ChROSLidarHandler(lidar, "~/output/lidar/point_cloud")
ros_manager.RegisterHandler(lidar_handler)

# 4. GPS handler
gps_handler = chros.ChROSGPSHandler(gps, "~/output/gps/data")
ros_manager.RegisterHandler(gps_handler)

# 5. Accelerometer handler (standalone + fed into fused IMU)
acc_handler = chros.ChROSAccelerometerHandler(acc, "~/output/accelerometer/data")
ros_manager.RegisterHandler(acc_handler)

# 6. Gyroscope handler (standalone + fed into fused IMU)
gyro_handler = chros.ChROSGyroscopeHandler(gyro, "~/output/gyroscope/data")
ros_manager.RegisterHandler(gyro_handler)

# 7. Magnetometer handler (standalone + fed into fused IMU)
mag_handler = chros.ChROSMagnetometerHandler(mag, "~/output/magnetometer/data")
ros_manager.RegisterHandler(mag_handler)

# 8. Fused IMU handler — publishes sensor_msgs/Imu combining the three sensors
imu_handler = chros.ChROSIMUHandler(IMU_RATE, "~/output/imu/data")
imu_handler.SetAccelerometerHandler(acc_handler)
imu_handler.SetGyroscopeHandler(gyro_handler)
imu_handler.SetMagnetometerHandler(mag_handler)
ros_manager.RegisterHandler(imu_handler)

# Initialize ROS manager AFTER all handlers, BEFORE the loop
ros_manager.Initialize()

# === Visualization — full Irrlicht block (Initialize FIRST, then scene elements) ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("SensROS — Sensor + ROS2 Demo")
vis.Initialize()                                      # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(8, -8, 4), chrono.ChVector3d(0, 0, 0))  # AFTER Initialize
vis.AddTypicalLights()
vis.AddGrid(
    1.0, 1.0, 20, 20,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, -0.45), chrono.QUNIT),
    chrono.ChColor(0.4, 0.4, 0.4),
)

# === Review-only setup ===

# === Main loop ===
frame = 0
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            time = sys.GetChTime()
            # Sensor scene update order: pump sensors → ROS publish → advance physics
            sens_manager.Update()
            if not ros_manager.Update(time, TIME_STEP):
                break
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:  # solver divergence / bad physics state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
