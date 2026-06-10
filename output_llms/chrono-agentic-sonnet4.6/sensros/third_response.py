"""
sensros — Sensor + ROS2 bridge simulation.

A spinning rigid body (ground_body) carries a full sensor suite:
camera, 2D lidar, GPS, accelerometer, gyroscope, and magnetometer.
All sensors publish to ROS2 via ChROSPythonManager; a fused IMU handler
combines the three inertial sensors. The body rotates at 0.2 rad/s about Z
so every sensor sees dynamic motion. A mesh body (floor tile) is added to
the physics system to give the sensors a visible scene object.

System: ChSystemNSC, Z-up, collision via Bullet (mesh body has collision shape).
Expected behavior: body spins smoothly; ROS topics publish sensor data; ROS
manager loop exits cleanly when the ROS layer shuts down.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens
import pychrono.ros as chros


# === Simulation constants ===
TIME_STEP   = 1e-3          # physics step size (s)
SIM_END     = 20.0          # simulation end time (s)
RENDER_FPS  = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# Sensor update rates (physical Hz)
CAM_RATE    = 30
LIDAR_RATE  = 5
GPS_RATE    = 10
IMU_RATE    = 100

# Body spin rate about Z axis (rad/s)
SPIN_RATE = 0.2

# Horizontal lidar parameters
H_SAMPLES = 800
V_SAMPLES = 1   # 2D lidar: single horizontal ring

# GPS reference origin (lat, lon, alt)
GPS_ORIGIN = chrono.ChVector3d(-89.400, 43.070, 260.0)

# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Contact material (for mesh body) ===
mat = chrono.ChContactMaterialNSC()
mat.SetFriction(0.5)
mat.SetRestitution(0.0)

# === Bodies ===
# -- Fixed hub body (anchor for the spin motor) --
hub_body = chrono.ChBody()
hub_body.SetName("hub_body")
hub_body.SetFixed(True)
hub_body.SetPos(chrono.ChVector3d(0, 0, 1))
sys.Add(hub_body)

# -- Spinning sensor carrier body driven by a rotation speed motor --
ground_body = chrono.ChBody()
ground_body.SetName("ground_body")
ground_body.SetFixed(False)
ground_body.SetPos(chrono.ChVector3d(0, 0, 1))
ground_body.SetMass(1.0)
ground_body.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))

vis_box = chrono.ChVisualShapeBox(0.5, 0.5, 0.1)
ground_body.AddVisualShape(vis_box)

coll_box = chrono.ChCollisionShapeBox(mat, 0.5, 0.5, 0.1)
ground_body.AddCollisionShape(coll_box)
ground_body.EnableCollision(False)   # no collision with floor — just spin freely
sys.Add(ground_body)

# Motor: spin ground_body about Z at SPIN_RATE rad/s
spin_motor = chrono.ChLinkMotorRotationSpeed()
spin_motor.Initialize(
    ground_body, hub_body,
    chrono.ChFramed(chrono.ChVector3d(0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 0, 1))),
)
spin_func = chrono.ChFunctionConst(SPIN_RATE)   # constant spin rate (rad/s)
spin_motor.SetSpeedFunction(spin_func)
sys.AddLink(spin_motor)

# -- Mesh body (floor tile) added to the simulation system --
mesh_body = chrono.ChBodyEasyBox(5.0, 5.0, 0.1, 100.0, True, True, mat)
mesh_body.SetFixed(True)
mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))
mesh_body.SetName("mesh_body")
sys.Add(mesh_body)  # add mesh body to simulation system

# === Sensor manager ===
manager = sens.ChSensorManager(sys)

# Point lights for the camera sensor
intensity = 1.0
manager.scene.AddPointLight(
    chrono.ChVector3f(2.0, 2.5, 100.0),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(-2.0, 2.5, 100.0),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)

# === Camera sensor ===
cam_offset = chrono.ChFramed(
    chrono.ChVector3d(-5, 0, 2),
    chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0)),
)
cam = sens.ChCameraSensor(
    ground_body,        # body camera rides on
    CAM_RATE,           # physical update rate (Hz)
    cam_offset,         # offset from the body
    1280, 720,          # width, height
    1.408,              # horizontal FOV (rad)
)
cam.SetName("Camera Sensor")
cam.SetLag(0)
cam.SetCollectionWindow(0)
cam.PushFilter(sens.ChFilterVisualize(1280, 720, "RGB Camera"))
cam.PushFilter(sens.ChFilterRGBA8Access())
cam.PushFilter(sens.ChFilterSave("cam/camera/"))
manager.AddSensor(cam)

# === 2D Lidar sensor ===
lidar_offset = chrono.ChFramed(
    chrono.ChVector3d(-12, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
lidar = sens.ChLidarSensor(
    ground_body,                            # body lidar rides on
    LIDAR_RATE,                             # physical update rate (Hz)
    lidar_offset,                           # offset from the body
    H_SAMPLES,                              # horizontal samples
    V_SAMPLES,                              # vertical samples (1 = 2D)
    2 * chrono.CH_PI,                       # horizontal FOV
    0,                                      # max vertical angle (0 for 2D)
    0,                                      # min vertical angle (0 for 2D)
    100.0,                                  # max range (m)
    sens.LidarBeamShape_RECTANGULAR,        # beam shape
    2,                                      # sample_radius
    0.003,                                  # vertical divergence angle
    0.003,                                  # horizontal divergence angle
    sens.LidarReturnMode_STRONGEST_RETURN,  # return mode
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(1.0 / LIDAR_RATE)   # lidar collection window = 1 / update_rate

# 2D lidar filter chain — ChFilterDIAccess, ChFilterPCfromDepth, ChFilterXYZIAccess
lidar.PushFilter(sens.ChFilterVisualize(H_SAMPLES, V_SAMPLES, "Raw Lidar Depth"))
lidar.PushFilter(sens.ChFilterDIAccess())           # host access to depth+intensity
lidar.PushFilter(sens.ChFilterPCfromDepth())        # depth → XYZI point cloud
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
lidar.PushFilter(sens.ChFilterXYZIAccess())         # host access to XYZI
manager.AddSensor(lidar)

# === GPS sensor ===
gps_offset = chrono.ChFramed(
    chrono.ChVector3d(-8, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
gps = sens.ChGPSSensor(
    ground_body,
    GPS_RATE,
    gps_offset,
    GPS_ORIGIN,
    sens.ChNoiseNone(),
)
gps.SetName("GPS Sensor")
gps.SetLag(0)
gps.SetCollectionWindow(0)
gps.PushFilter(sens.ChFilterGPSAccess())
manager.AddSensor(gps)

# === Accelerometer sensor ===
acc_offset = chrono.ChFramed(
    chrono.ChVector3d(0, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
acc = sens.ChAccelerometerSensor(ground_body, IMU_RATE, acc_offset, sens.ChNoiseNone())
acc.SetName("Accelerometer Sensor")
acc.SetLag(0)
acc.SetCollectionWindow(0)
acc.PushFilter(sens.ChFilterAccelAccess())
manager.AddSensor(acc)

# === Gyroscope sensor ===
gyro_offset = chrono.ChFramed(
    chrono.ChVector3d(0, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
gyro = sens.ChGyroscopeSensor(ground_body, IMU_RATE, gyro_offset, sens.ChNoiseNone())
gyro.SetName("Gyroscope Sensor")
gyro.SetLag(0)
gyro.SetCollectionWindow(0)
gyro.PushFilter(sens.ChFilterGyroAccess())
manager.AddSensor(gyro)

# === Magnetometer sensor ===
mag_offset = chrono.ChFramed(
    chrono.ChVector3d(0, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
mag = sens.ChMagnetometerSensor(
    ground_body, IMU_RATE, mag_offset,
    sens.ChNoiseNone(),
    GPS_ORIGIN,
)
mag.SetName("Magnetometer Sensor")
mag.SetLag(0)
mag.SetCollectionWindow(0)
mag.PushFilter(sens.ChFilterMagnetAccess())
manager.AddSensor(mag)

# === ROS Manager ===
ros_manager = chros.ChROSPythonManager()

# 1. Clock handler FIRST (publishes /clock)
ros_manager.RegisterHandler(chros.ChROSClockHandler())

# 2. Body handler for ground_body pose/twist
ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, ground_body, "~/output/ground_body/state"))

# 3. Camera handler
ros_manager.RegisterHandler(chros.ChROSCameraHandler(CAM_RATE, cam, "~/output/camera/image_color"))

# 4. Lidar handler
ros_manager.RegisterHandler(chros.ChROSLidarHandler(lidar, "~/output/lidar/point_cloud"))

# 5. GPS handler
ros_manager.RegisterHandler(chros.ChROSGPSHandler(gps, "~/output/gps/data"))

# 6. IMU sub-handlers (registered standalone and fed into fused handler)
acc_handler  = chros.ChROSAccelerometerHandler(acc, "~/output/accelerometer/data")
gyro_handler = chros.ChROSGyroscopeHandler(gyro, "~/output/gyroscope/data")
mag_handler  = chros.ChROSMagnetometerHandler(mag, "~/output/magnetometer/data")
ros_manager.RegisterHandler(acc_handler)
ros_manager.RegisterHandler(gyro_handler)
ros_manager.RegisterHandler(mag_handler)

# 7. Fused IMU handler combining accel + gyro + mag
imu_handler = chros.ChROSIMUHandler(100, "~/output/imu/data")
imu_handler.SetAccelerometerHandler(acc_handler)
imu_handler.SetGyroscopeHandler(gyro_handler)
imu_handler.SetMagnetometerHandler(mag_handler)
ros_manager.RegisterHandler(imu_handler)

# Initialize ONCE, after ALL registration, before the loop
ros_manager.Initialize()

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("sensros — Sensor + ROS2 Bridge")
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.Initialize()                         # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(-8, 0, 4), chrono.ChVector3d(0, 0, 1))
vis.AddTypicalLights()

# === Review-only setup ===

# === Main loop ===
frame = 0

try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            manager.Update()             # pump sensors every physics step
            sys.DoStepDynamics(TIME_STEP)
            # Check ROS manager update; exit loop if the ROS layer shuts down
            if not ros_manager.Update(sys.GetChTime(), TIME_STEP):
                break
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback; traceback.print_exc()
    raise
finally:
    pass
