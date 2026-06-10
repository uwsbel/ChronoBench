"""
Sensros: Multi-sensor simulation with ROS publishing.

A moving ground body carries camera, lidar, GPS, accelerometer, gyroscope,
and magnetometer sensors, all published over ROS2 via ChROSPythonManager.

System type: ChSystemNSC (no contact; pure kinematics + sensors)
Main bodies: ground_body (moving, carries all sensors), mesh_body (visual prop)
Sensors: camera, lidar, GPS, accelerometer, gyroscope, magnetometer
ROS handlers: Clock + Body + TF + Camera + Lidar + GPS + Accelerometer + Gyroscope
            + Magnetometer + fused IMU
Expected behavior: all six sensors update in real time; ROS publishes each topic.
"""

import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens
import pychrono.ros as chros

# === Named constants ===
TIME_STEP = 1e-3
SIM_END = 10.0
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))

# Sensor update rates (physical Hz, not 1/dt)
CAM_RATE = 30.0
LIDAR_RATE = 5.0
GPS_RATE = 10.0
IMU_RATE = 100.0

# ROS publish rates
BODY_RATE = 30.0
TF_RATE = 30.0

# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# === Ground body (moving — carries all sensors) ===
ground_body = chrono.ChBody()
ground_body.SetName("ground_body")
ground_body.SetFixed(False)
ground_body.SetMass(1.0)
ground_body.SetInertiaXX(chrono.ChVector3d(0.01, 0.01, 0.01))
ground_body.SetPos(chrono.ChVector3d(0, 0, 0))
ground_body.SetRot(chrono.QUNIT)
# Visual shape for the ground body
ground_vis = chrono.ChVisualShapeBox(1.0, 1.0, 0.05)
ground_body.AddVisualShape(ground_vis)
# Give it an initial spin so sensors see motion
ground_body.SetAngVelParent(chrono.ChVector3d(0, 0, 0.5))
sys.AddBody(ground_body)

# === Mesh body for visualization (visual-only prop on the ground) ===
mesh_body = chrono.ChBody()
mesh_body.SetName("mesh_body")
mesh_body.SetFixed(True)
mesh_body.SetPos(chrono.ChVector3d(0, 0, 0.1))
sys.AddBody(mesh_body)

# Attach a box visual shape for the mesh body
box_vis = chrono.ChVisualShapeBox(0.6, 0.4, 0.3)
mesh_body.AddVisualShape(box_vis)

# === Sensor manager ===
manager = sens.ChSensorManager(sys)

# Scene lighting for camera (point lights — canonical setup)
intensity = 1.0
manager.scene.AddPointLight(
    chrono.ChVector3f(2, 2.5, 10),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(-4, -3, 10),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)

# Sensor offset poses relative to ground_body
SENSOR_OFFSET = chrono.ChFramed(
    chrono.ChVector3d(0, 0, 0.5),
    chrono.QUNIT,
)

# --- Camera sensor ---
cam = sens.ChCameraSensor(ground_body, CAM_RATE, SENSOR_OFFSET, 1280, 720, 1.408)
cam.SetName("Camera Sensor")
cam.SetLag(0)
cam.SetCollectionWindow(0)
cam.PushFilter(sens.ChFilterVisualize(1280, 720, "RGB Camera"))
cam.PushFilter(sens.ChFilterRGBA8Access())
cam.PushFilter(sens.ChFilterSave("cam/rgb/"))
manager.AddSensor(cam)

# --- Lidar sensor ---
lidar = sens.ChLidarSensor(
    ground_body,
    LIDAR_RATE,
    SENSOR_OFFSET,
    800,   # h_samples
    1,     # v_samples (2D lidar)
    2 * chrono.CH_PI,        # horizontal_fov
    0,                           # max_vert_angle
    0,                           # min_vert_angle
    100.0,                       # max_range
    sens.LidarBeamShape_RECTANGULAR,
    2,                           # sample_radius
    0.003,                       # vert divergence_angle
    0.003,                       # hori divergence_angle
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(1.0 / LIDAR_RATE)
lidar.PushFilter(sens.ChFilterVisualize(800, 1, "Lidar Depth"))
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterXYZIAccess())
manager.AddSensor(lidar)

# --- Accelerometer ---
acc = sens.ChAccelerometerSensor(
    ground_body, IMU_RATE, SENSOR_OFFSET, sens.ChNoiseNone()
)
acc.SetName("Accelerometer Sensor")
acc.SetLag(0)
acc.SetCollectionWindow(0)
acc.PushFilter(sens.ChFilterAccelAccess())
manager.AddSensor(acc)

# --- Gyroscope ---
gyro = sens.ChGyroscopeSensor(
    ground_body, IMU_RATE, SENSOR_OFFSET, sens.ChNoiseNone()
)
gyro.SetName("Gyroscope Sensor")
gyro.SetLag(0)
gyro.SetCollectionWindow(0)
gyro.PushFilter(sens.ChFilterGyroAccess())
manager.AddSensor(gyro)

# --- Magnetometer ---
mag = sens.ChMagnetometerSensor(
    ground_body, IMU_RATE, SENSOR_OFFSET, sens.ChNoiseNone(),
    chrono.ChVector3d(45, 0, 45),  # gps_reference: local magnetic field in NED [uT]
)
mag.SetName("Magnetometer Sensor")
mag.SetLag(0)
mag.SetCollectionWindow(0)
mag.PushFilter(sens.ChFilterMagnetAccess())
manager.AddSensor(mag)

# --- GPS ---
gps = sens.ChGPSSensor(
    ground_body,
    GPS_RATE,
    SENSOR_OFFSET,
    chrono.ChVector3d(-89.400, 43.070, 260.0),  # reference lat/lon/alt
    sens.ChNoiseNone(),
)
gps.SetName("GPS Sensor")
gps.SetLag(0)
gps.SetCollectionWindow(0)
gps.PushFilter(sens.ChFilterGPSAccess())
manager.AddSensor(gps)

# === ROS manager ===
ros_manager = chros.ChROSPythonManager()

# Clock handler first — publishes /clock
ros_manager.RegisterHandler(chros.ChROSClockHandler())

# Body handler for ground body pose/twist
ros_manager.RegisterHandler(
    chros.ChROSBodyHandler(BODY_RATE, ground_body, "~/output/body/data")
)

# TF handler
tf_handler = chros.ChROSTFHandler(TF_RATE)
tf_handler.AddTransform(
    ground_body, ground_body.GetName(),
    mesh_body, mesh_body.GetName(),
)
ros_manager.RegisterHandler(tf_handler)

# Camera handler
ros_manager.RegisterHandler(
    chros.ChROSCameraHandler(CAM_RATE, cam, "~/output/camera/image")
)

# Lidar handler
ros_manager.RegisterHandler(
    chros.ChROSLidarHandler(lidar, "~/output/lidar/scan")
)

# GPS handler
ros_manager.RegisterHandler(
    chros.ChROSGPSHandler(gps, "~/output/gps/navsat")
)

# Accelerometer handler
acc_handler = chros.ChROSAccelerometerHandler(
    acc, "~/output/accelerometer/data"
)
ros_manager.RegisterHandler(acc_handler)

# Gyroscope handler
gyro_handler = chros.ChROSGyroscopeHandler(
    gyro, "~/output/gyroscope/data"
)
ros_manager.RegisterHandler(gyro_handler)

# Magnetometer handler
mag_handler = chros.ChROSMagnetometerHandler(
    mag, "~/output/magnetometer/data"
)
ros_manager.RegisterHandler(mag_handler)

# Fused IMU handler
imu_handler = chros.ChROSIMUHandler(IMU_RATE, "~/output/imu/data")
imu_handler.SetAccelerometerHandler(acc_handler)
imu_handler.SetGyroscopeHandler(gyro_handler)
imu_handler.SetMagnetometerHandler(mag_handler)
ros_manager.RegisterHandler(imu_handler)

# Initialize ROS — once, after all registration
ros_manager.Initialize()

# === Visualization (full Irrlicht block) ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Sensros: Multi-Sensor Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(
    chrono.ChVector3d(3, -3, 3),
    chrono.ChVector3d(0, 0, 0),
)
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 40, 40, chrono.ChCoordsysd(), chrono.ChColor(0.4, 0.4, 0.4))

# === Review-only recording scaffolding ===
REC = bool(os.environ.get("SIMBENCH_RECORD"))
os.makedirs("frames", exist_ok=True)

frame = 0

# === Main loop ===
while vis.Run() and sys.GetChTime() < SIM_END:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    if REC:
        vis.WriteImageToFile(f"frames/img_{frame:06d}.png")
    frame += 1

    for _ in range(RENDER_EVERY):
        # 1. Update sensors
        manager.Update()

        # 2. Publish ROS topics
        time_now = sys.GetChTime()
        if not ros_manager.Update(time_now, TIME_STEP):
            break

        # 3. Advance physics
        sys.DoStepDynamics(TIME_STEP)

        if sys.GetChTime() >= SIM_END:
            break

# === Review-only post-processing ===
