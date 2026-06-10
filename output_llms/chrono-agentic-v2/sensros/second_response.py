"""
sensros Turn 2 — Sensor + ROS2 bridge simulation (ChSystemNSC).

Models a rotating box body that carries a suite of sensors:
  - RGB camera (Chase view)
  - 3D Lidar sensor
  - 2D Lidar sensor (added in Turn 2, publishing to ~/output/lidar2d/data/scan)
  - GPS sensor
  - Accelerometer, Gyroscope, Magnetometer (fused into IMU)

The body spins at a constant angular velocity so all sensors observe motion.
Each sensor is registered with a matching ChROS handler, with the 2D Lidar
sensor using ChROSLidarHandler (LASER_SCAN type) on the ~/output/lidar2d/data/scan
topic.
System: ChSystemNSC, Y-up gravity disabled (sensor demo; body is kinematically spun).
"""

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens
import pychrono.ros as chros
import os                                         # review-only (used in REC)

# === Named constants ===
TIME_STEP   = 1e-3          # physics step size (s)
SIM_END     = 10.0          # simulation end time (s)
RENDER_FPS  = 50.0          # Irrlicht render rate (Hz)
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# Sensor update rates (physical Hz)
CAM_RATE    = 30
LIDAR_RATE  = 5
LIDAR2D_RATE = 5
GPS_RATE    = 10
IMU_RATE    = 100

# Body spin rate
ANG_VEL = 0.5   # rad/s about Z

# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))   # no gravity (sensor demo)

# === Bodies ===
# Ground body (fixed, named base_link for TF root)
ground = chrono.ChBody()
ground.SetFixed(True)
ground.SetName("base_link")
ground_shape = chrono.ChVisualShapeBox(4, 0.1, 4)
ground.AddVisualShape(ground_shape, chrono.ChFramed(chrono.ChVector3d(0, -0.05, 0), chrono.QUNIT))
sys.Add(ground)

# Rotating sensor-carrier box
box = chrono.ChBody()
box.SetName("sensor_box")
box.SetPos(chrono.ChVector3d(0, 1, 0))
box.SetAngVelParent(chrono.ChVector3d(0, ANG_VEL, 0))   # spin about Y
box_shape = chrono.ChVisualShapeBox(0.5, 0.5, 0.5)
box_shape.SetColor(chrono.ChColor(0.4, 0.7, 0.3))
box.AddVisualShape(box_shape)
# Collision shape required for OptiX sensor rendering
mat = chrono.ChContactMaterialNSC()
box.EnableCollision(False)    # kinematic body; no contact needed
sys.Add(box)

# === Sensor Manager ===
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

# === Camera Sensor ===
cam_offset = chrono.ChFramed(
    chrono.ChVector3d(-5, 0, 2),
    chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0)),
)
cam = sens.ChCameraSensor(box, CAM_RATE, cam_offset, 1280, 720, 1.408)
cam.SetName("Camera Sensor")
cam.SetLag(0)
cam.SetCollectionWindow(0)
cam.PushFilter(sens.ChFilterVisualize(1280, 720, "RGB Camera"))
cam.PushFilter(sens.ChFilterRGBA8Access())
cam.PushFilter(sens.ChFilterSave("cam/rgb/"))
manager.AddSensor(cam)

# === 3D Lidar Sensor ===
lidar_offset = chrono.ChFramed(
    chrono.ChVector3d(-12, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
h_samples_3d = 800
v_samples_3d = 300
lidar = sens.ChLidarSensor(
    box, LIDAR_RATE, lidar_offset,
    h_samples_3d, v_samples_3d,
    2 * chrono.CH_PI,
    chrono.CH_PI / 12,
    -chrono.CH_PI / 6,
    100.0,
    sens.LidarBeamShape_RECTANGULAR,
    2, 0.003, 0.003,
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(1.0 / LIDAR_RATE)
lidar.PushFilter(sens.ChFilterVisualize(h_samples_3d, v_samples_3d, "Raw Lidar Depth"))
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
lidar.PushFilter(sens.ChFilterXYZIAccess())
manager.AddSensor(lidar)

# === 2D Lidar Sensor (Turn 2: added for laser-scan publishing via ROS) ===
lidar2d_offset = chrono.ChFramed(
    chrono.ChVector3d(0, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
h_samples_2d = 800
v_samples_2d = 1           # 2D lidar: single ring
lidar2d = sens.ChLidarSensor(
    box, LIDAR2D_RATE, lidar2d_offset,
    h_samples_2d, v_samples_2d,
    2 * chrono.CH_PI,
    0,     # max_vert_angle = 0 for 2D
    0,     # min_vert_angle = 0 for 2D
    100.0,
    sens.LidarBeamShape_RECTANGULAR,
    2, 0.003, 0.003,
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar2d.SetName("2D Lidar Sensor")
lidar2d.SetLag(0)
lidar2d.SetCollectionWindow(1.0 / LIDAR2D_RATE)
lidar2d.PushFilter(sens.ChFilterVisualize(h_samples_2d, v_samples_2d, "2D Lidar Depth"))
lidar2d.PushFilter(sens.ChFilterDIAccess())
lidar2d.PushFilter(sens.ChFilterXYZIAccess())
manager.AddSensor(lidar2d)

# === GPS Sensor ===
gps_offset = chrono.ChFramed(
    chrono.ChVector3d(-8, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
gps = sens.ChGPSSensor(
    box, GPS_RATE, gps_offset,
    chrono.ChVector3d(-89.400, 43.070, 260.0),
    sens.ChNoiseNone(),
)
gps.SetName("GPS Sensor")
gps.SetLag(0)
gps.SetCollectionWindow(0)
gps.PushFilter(sens.ChFilterGPSAccess())
manager.AddSensor(gps)

# === IMU Sensors (Accelerometer + Gyroscope + Magnetometer) ===
imu_offset = chrono.ChFramed(
    chrono.ChVector3d(0, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)

acc = sens.ChAccelerometerSensor(box, IMU_RATE, imu_offset, sens.ChNoiseNone())
acc.SetName("Accelerometer")
acc.SetLag(0)
acc.SetCollectionWindow(0)
acc.PushFilter(sens.ChFilterAccelAccess())
manager.AddSensor(acc)

gyro = sens.ChGyroscopeSensor(box, IMU_RATE, imu_offset, sens.ChNoiseNone())
gyro.SetName("Gyroscope")
gyro.SetLag(0)
gyro.SetCollectionWindow(0)
gyro.PushFilter(sens.ChFilterGyroAccess())
manager.AddSensor(gyro)

mag = sens.ChMagnetometerSensor(
    box, IMU_RATE, imu_offset,
    sens.ChNoiseNone(),
    chrono.ChVector3d(-89.400, 43.070, 260.0),
)
mag.SetName("Magnetometer")
mag.SetLag(0)
mag.SetCollectionWindow(0)
mag.PushFilter(sens.ChFilterMagnetAccess())
manager.AddSensor(mag)

# === ROS Manager ===
ros_manager = chros.ChROSPythonManager()

# 1. Clock handler first
ros_manager.RegisterHandler(chros.ChROSClockHandler())

# 2. Camera handler
ros_manager.RegisterHandler(chros.ChROSCameraHandler(CAM_RATE, cam, "~/output/camera/data/image"))

# 3. 3D Lidar handler (point cloud)
ros_manager.RegisterHandler(chros.ChROSLidarHandler(lidar, "~/output/lidar3d/data/pc"))

# 4. 2D Lidar handler (laser scan) — Turn 2 addition
ros_manager.RegisterHandler(
    chros.ChROSLidarHandler(
        lidar2d,
        "~/output/lidar2d/data/scan",
        chros.ChROSLidarHandlerMessageType_LASER_SCAN,
    )
)

# 5. GPS handler
ros_manager.RegisterHandler(chros.ChROSGPSHandler(gps, "~/output/gps/data"))

# 6. IMU sub-handlers
acc_handler  = chros.ChROSAccelerometerHandler(acc,  "~/output/accelerometer/data")
gyro_handler = chros.ChROSGyroscopeHandler(gyro, "~/output/gyroscope/data")
mag_handler  = chros.ChROSMagnetometerHandler(mag,  "~/output/magnetometer/data")
ros_manager.RegisterHandler(acc_handler)
ros_manager.RegisterHandler(gyro_handler)
ros_manager.RegisterHandler(mag_handler)

# 7. Fused IMU handler
imu_handler = chros.ChROSIMUHandler(IMU_RATE, "~/output/imu/data")
imu_handler.SetAccelerometerHandler(acc_handler)
imu_handler.SetGyroscopeHandler(gyro_handler)
imu_handler.SetMagnetometerHandler(mag_handler)
ros_manager.RegisterHandler(imu_handler)

# 8. Body handler
ros_manager.RegisterHandler(chros.ChROSBodyHandler(30, box, "~/output/box/state"))

# 9. TF handler
tf_handler = chros.ChROSTFHandler(30)
tf_handler.AddTransform(ground, ground.GetName(), box, box.GetName())
ros_manager.RegisterHandler(tf_handler)

# Initialize ONCE, after all registration
ros_manager.Initialize()

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("sensros — 2D Lidar + ROS")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, -8), chrono.ChVector3d(0, 1, 0))
vis.AddTypicalLights()

# === Main loop ===

frame = 0
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            manager.Update()          # pump all sensors every physics step
            sys.DoStepDynamics(TIME_STEP)
            if not ros_manager.Update(sys.GetChTime(), TIME_STEP):
                break
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback; traceback.print_exc()
    raise
