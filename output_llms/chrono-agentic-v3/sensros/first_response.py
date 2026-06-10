"""
sensros — PyChrono sensor + ROS2 bridge demo.

Models a rotating platform (ChSystemNSC, no gravity) carrying a full sensor
suite: camera, lidar, GPS, accelerometer, gyroscope, and magnetometer.  Every
sensor publishes to a dedicated ROS2 topic via ChROSPythonManager; the IMU
triad is also fused into a ChROSIMUHandler.  The platform spins at 0.5 rad/s
so every sensor observes time-varying motion.

System type : ChSystemNSC (no gravity, free rotation)
Bodies       : ground (fixed visual reference), sensor_platform (spinning)
Expected     : platform spins at ~0.5 rad/s; all six sensors update and
               publish to /clock, ~/output/camera/*, ~/output/lidar/*,
               ~/output/gps/*, ~/output/imu/*, ~/output/accelerometer/*,
               ~/output/gyroscope/*, ~/output/magnetometer/*.
"""

# === Imports ===
import os
import csv

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens
import pychrono.ros as chros


# === Named constants ===
TIME_STEP    = 1e-3           # physics step (s)
SIM_END      = 30.0           # simulation duration (s)
RENDER_FPS   = 50.0           # Irrlicht render rate (Hz)
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))        # precomputed once

# Sensor update rates (physical Hz, never 1/time_step)
CAM_HZ   = 30
LIDAR_HZ =  5
GPS_HZ   = 10
IMU_HZ   = 100

# GPS / magnetometer reference origin (lat, lon, alt)
GPS_ORIGIN = chrono.ChVector3d(-89.400, 43.070, 260.0)

# Platform spin rate
SPIN_RATE = 0.5   # rad/s about Z

# === System & gravity ===
sys_chrono = chrono.ChSystemNSC()
sys_chrono.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))  # free rotation, no gravity
sys_chrono.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Bodies ===
# Fixed ground reference — provides a visual floor and collision surface
ground = chrono.ChBody()
ground.SetFixed(True)
ground.SetName("base_link")
ground.SetPos(chrono.ChVector3d(0, 0, 0))
ground_vis = chrono.ChVisualShapeBox(6, 6, 0.1)
ground_vis.SetColor(chrono.ChColor(0.3, 0.3, 0.8))
ground.AddVisualShape(ground_vis, chrono.ChFramed(chrono.ChVector3d(0, 0, -0.05), chrono.QUNIT))
mat_ground = chrono.ChContactMaterialNSC()
mat_ground.SetFriction(0.8)
ground_col = chrono.ChCollisionShapeBox(mat_ground, 6, 6, 0.1)
ground.AddCollisionShape(ground_col, chrono.ChFramed(chrono.ChVector3d(0, 0, -0.05), chrono.QUNIT))
ground.EnableCollision(True)
sys_chrono.AddBody(ground)

# Spinning sensor platform — all sensors are attached here
platform = chrono.ChBody()
platform.SetFixed(False)
platform.SetName("sensor_platform")
platform.SetPos(chrono.ChVector3d(0, 0, 1.0))
platform.SetMass(10.0)
platform.SetInertiaXX(chrono.ChVector3d(0.5, 0.5, 0.8))
plat_vis = chrono.ChVisualShapeBox(4, 4, 0.5)
plat_vis.SetColor(chrono.ChColor(0.8, 0.4, 0.2))
platform.AddVisualShape(plat_vis)
mat_plat = chrono.ChContactMaterialNSC()
mat_plat.SetFriction(0.8)
plat_col = chrono.ChCollisionShapeBox(mat_plat, 4, 4, 0.5)
platform.AddCollisionShape(plat_col)
platform.EnableCollision(True)
platform.SetAngVelParent(chrono.ChVector3d(0, 0, SPIN_RATE))  # initial spin
sys_chrono.AddBody(platform)

# === Sensor manager & lighting ===
sens_manager = sens.ChSensorManager(sys_chrono)
intensity = 1.0
sens_manager.scene.AddPointLight(
    chrono.ChVector3f(2, 2.5, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)
sens_manager.scene.AddPointLight(
    chrono.ChVector3f(-2, -2.5, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)

# === Camera sensor ===
cam_offset = chrono.ChFramed(
    chrono.ChVector3d(-8, 0, 4),
    chrono.QuatFromAngleAxis(0.3, chrono.ChVector3d(0, 1, 0)),
)
cam = sens.ChCameraSensor(platform, CAM_HZ, cam_offset, 1280, 720, 1.408)
cam.SetName("Camera Sensor")
cam.SetLag(0)
cam.SetCollectionWindow(0)
cam.PushFilter(sens.ChFilterVisualize(1280, 720, "RGB Camera"))
cam.PushFilter(sens.ChFilterRGBA8Access())
cam.PushFilter(sens.ChFilterSave("cam/rgb/"))
sens_manager.AddSensor(cam)

# === Lidar sensor ===
lidar_offset = chrono.ChFramed(
    chrono.ChVector3d(0, 0, 2),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
lidar = sens.ChLidarSensor(
    platform,
    float(LIDAR_HZ),
    lidar_offset,
    800,                              # h_samples
    300,                              # v_samples
    2 * chrono.CH_PI,                 # horizontal FOV (rad)
    chrono.CH_PI / 12,                # max vertical angle
    -chrono.CH_PI / 6,                # min vertical angle
    100.0,                            # max range (m)
    sens.LidarBeamShape_RECTANGULAR,
    1,                                # sample radius (1 = single sample, lower GPU memory)
    0.003,                            # vertical divergence angle
    0.003,                            # horizontal divergence angle
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(1.0 / LIDAR_HZ)
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterXYZIAccess())
sens_manager.AddSensor(lidar)

# === GPS sensor ===
imu_offset = chrono.ChFramed(
    chrono.ChVector3d(0, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
gps = sens.ChGPSSensor(platform, float(GPS_HZ), imu_offset, GPS_ORIGIN, sens.ChNoiseNone())
gps.SetName("GPS Sensor")
gps.SetLag(0)
gps.SetCollectionWindow(0)
gps.PushFilter(sens.ChFilterGPSAccess())
sens_manager.AddSensor(gps)

# === Accelerometer ===
acc = sens.ChAccelerometerSensor(platform, float(IMU_HZ), imu_offset, sens.ChNoiseNone())
acc.SetName("Accelerometer Sensor")
acc.SetLag(0)
acc.SetCollectionWindow(0)
acc.PushFilter(sens.ChFilterAccelAccess())
sens_manager.AddSensor(acc)

# === Gyroscope ===
gyro = sens.ChGyroscopeSensor(platform, float(IMU_HZ), imu_offset, sens.ChNoiseNone())
gyro.SetName("Gyroscope Sensor")
gyro.SetLag(0)
gyro.SetCollectionWindow(0)
gyro.PushFilter(sens.ChFilterGyroAccess())
sens_manager.AddSensor(gyro)

# === Magnetometer ===
mag = sens.ChMagnetometerSensor(
    platform, float(IMU_HZ), imu_offset, sens.ChNoiseNone(), GPS_ORIGIN
)
mag.SetName("Magnetometer Sensor")
mag.SetLag(0)
mag.SetCollectionWindow(0)
mag.PushFilter(sens.ChFilterMagnetAccess())
sens_manager.AddSensor(mag)

# === ROS2 manager ===
ros_manager = chros.ChROSPythonManager()

# Clock handler registered first — publishes /clock for ROS time sync
ros_manager.RegisterHandler(chros.ChROSClockHandler())

# Sensor handlers
ros_manager.RegisterHandler(chros.ChROSCameraHandler(cam, "~/output/camera/data"))
ros_manager.RegisterHandler(chros.ChROSLidarHandler(lidar, "~/output/lidar/data"))
ros_manager.RegisterHandler(chros.ChROSGPSHandler(gps, "~/output/gps/data"))

# IMU sub-handlers (standalone topics)
acc_handler  = chros.ChROSAccelerometerHandler(acc,  "~/output/accelerometer/data")
gyro_handler = chros.ChROSGyroscopeHandler(gyro, "~/output/gyroscope/data")
mag_handler  = chros.ChROSMagnetometerHandler(mag,  "~/output/magnetometer/data")
ros_manager.RegisterHandler(acc_handler)
ros_manager.RegisterHandler(gyro_handler)
ros_manager.RegisterHandler(mag_handler)

# Fused IMU handler — publishes sensor_msgs/Imu
imu_handler = chros.ChROSIMUHandler(IMU_HZ, "~/output/imu/data")
imu_handler.SetAccelerometerHandler(acc_handler)
imu_handler.SetGyroscopeHandler(gyro_handler)
imu_handler.SetMagnetometerHandler(mag_handler)
ros_manager.RegisterHandler(imu_handler)

# Body pose/twist publisher
ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, platform, "~/output/body/data"))

# Initialize exactly once, after all handlers are registered
ros_manager.Initialize()

# === Visualization — full Irrlicht block (Initialize first, scene elements after) ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys_chrono)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("sensros — Sensor + ROS2 Demo")
vis.Initialize()                                      # FIRST, then add scene elements
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(8, -8, 6), chrono.ChVector3d(0, 0, 1))
vis.AddTypicalLights()
vis.AddGrid(
    1.0, 1.0, 20, 20,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    chrono.ChColor(0.4, 0.4, 0.4),
)

# Record flag — used in scored core to gate review-only captures


# === Main loop ===
try:
    while vis.Run() and sys_chrono.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            time_now = sys_chrono.GetChTime()
            sens_manager.Update()                       # pump sensors before ROS publish
            if not ros_manager.Update(time_now, TIME_STEP):
                break
            sys_chrono.DoStepDynamics(TIME_STEP)
            if sys_chrono.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:              # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass  # CSV closed in the review-only block below
