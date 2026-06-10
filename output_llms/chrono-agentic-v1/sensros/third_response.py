"""
Sensor + ROS2 demo: a spinning box body carries a full sensor suite
(camera, 2D lidar, GPS, accelerometer, gyroscope, magnetometer) and a
mesh body. Every sensor is published over ROS2 via ChROSPythonManager.
The 2D lidar filter chain includes ChFilterDIAccess, ChFilterPCfromDepth,
and ChFilterXYZIAccess. The simulation loop exits when ros_manager.Update()
returns False (ROS node has shut down).

System type : ChSystemNSC
Protagonist  : spinning box body (+ mesh body added to system)
Sensors      : camera, 2D lidar, GPS, accelerometer, gyroscope, magnetometer
ROS handlers : clock, body, camera, lidar, GPS, accel, gyro, mag, IMU fused
Expected     : sensors spin with the body; ROS topics are published; loop
               exits gracefully when ROS shuts down.
"""

# === Imports ===
import os
import math
import csv
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens
import pychrono.ros as chros

# === Simulation parameters ===
time_step  = 1e-3          # physics step size [s]
sim_end    = 30.0          # simulation duration [s]
render_fps = 50.0          # Irrlicht render cadence [Hz]
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

# === System & gravity ===
# ChSystemNSC — no contact between bodies (pure MBS), collision not needed
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))  # zero gravity: free-floating spin demo

# === Bodies ===
# --- Floor / reference body (fixed) ---
floor = chrono.ChBody()
floor.SetFixed(True)
floor.SetName("base_link")
floor_shape = chrono.ChVisualShapeBox(10.0, 0.1, 10.0)
floor_shape.SetColor(chrono.ChColor(0.4, 0.4, 0.4))
floor.AddVisualShape(floor_shape, chrono.ChFramed(chrono.ChVector3d(0, -0.05, 0), chrono.QUNIT))
sys.AddBody(floor)

# --- Spinning sensor platform body ---
platform = chrono.ChBody()
platform.SetFixed(False)
platform.SetName("platform")
platform.SetPos(chrono.ChVector3d(0, 0, 1))
platform.SetMass(10.0)
platform.SetInertiaXX(chrono.ChVector3d(0.5, 0.5, 0.5))
# Constant angular velocity so sensors see motion (20 deg/s about Z)
platform.SetAngVelParent(chrono.ChVector3d(0, 0, chrono.CH_DEG_TO_RAD * 20))
box_shape = chrono.ChVisualShapeBox(0.5, 0.5, 0.5)
box_shape.SetColor(chrono.ChColor(0.2, 0.5, 0.8))
platform.AddVisualShape(box_shape)
sys.AddBody(platform)

# --- Mesh body (added to system per turn-3 requirement) ---
mesh_body = chrono.ChBody()
mesh_body.SetFixed(True)
mesh_body.SetName("mesh_body")
mesh_body.SetPos(chrono.ChVector3d(3, 0, 0.5))
# Use a simple visual box as a stand-in mesh shape
mesh_vis = chrono.ChVisualShapeBox(0.8, 0.8, 1.0)
mesh_vis.SetColor(chrono.ChColor(0.6, 0.3, 0.1))
mesh_body.AddVisualShape(mesh_vis)
sys.Add(mesh_body)  # sys.Add() (not AddBody) — as specified in turn-3 requirement

# === Sensor manager ===
manager = sens.ChSensorManager(sys)
intensity = 1.0
manager.scene.AddPointLight(
    chrono.ChVector3f(2, 2.5, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(-2, 2.5, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)

# === Camera sensor ===
cam_offset = chrono.ChFramed(
    chrono.ChVector3d(-5, 0, 2),
    chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0)),
)
cam = sens.ChCameraSensor(platform, 30, cam_offset, 1280, 720, 1.408)
cam.SetName("Camera Sensor")
cam.SetLag(0)
cam.SetCollectionWindow(0)
cam.PushFilter(sens.ChFilterVisualize(1280, 720, "RGB Camera"))
cam.PushFilter(sens.ChFilterRGBA8Access())
cam.PushFilter(sens.ChFilterSave("cam/rgb/"))
manager.AddSensor(cam)

# === 2D Lidar sensor ===
# 2D lidar: v_samples=1, both vert angles=0
lidar_offset = chrono.ChFramed(
    chrono.ChVector3d(-12, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
lidar = sens.ChLidarSensor(
    platform,                              # attached body
    5.0,                                   # update_rate Hz
    lidar_offset,                          # offset pose
    800,                                   # h_samples
    1,                                     # v_samples = 1 → 2D lidar
    2 * chrono.CH_PI,                      # horizontal_fov
    0,                                     # max_vert_angle = 0 for 2D
    0,                                     # min_vert_angle = 0 for 2D
    100.0,                                 # max_range [m]
    sens.LidarBeamShape_RECTANGULAR,       # beam shape
    2,                                     # sample_radius
    0.003,                                 # vert divergence_angle
    0.003,                                 # hori divergence_angle
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(1.0 / 5.0)  # collection window = 1 / update_rate

# 2D Lidar filter chain (order matters):
lidar.PushFilter(sens.ChFilterVisualize(800, 1, "Raw Lidar Depth"))
lidar.PushFilter(sens.ChFilterDIAccess())          # host access depth+intensity (turn-3)
lidar.PushFilter(sens.ChFilterPCfromDepth())       # depth → XYZI point cloud (turn-3)
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
lidar.PushFilter(sens.ChFilterXYZIAccess())        # host access XYZI (turn-3)
manager.AddSensor(lidar)

# === GPS sensor ===
gps_offset = chrono.ChFramed(
    chrono.ChVector3d(-8, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
gps = sens.ChGPSSensor(
    platform, 10, gps_offset,
    chrono.ChVector3d(-89.400, 43.070, 260.0),  # ref lat/lon/alt
    sens.ChNoiseNone(),
)
gps.SetName("GPS Sensor")
gps.SetLag(0)
gps.SetCollectionWindow(0)
gps.PushFilter(sens.ChFilterGPSAccess())
manager.AddSensor(gps)

# === Accelerometer sensor ===
imu_offset = chrono.ChFramed(
    chrono.ChVector3d(0, 0, 0),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
acc = sens.ChAccelerometerSensor(platform, 100, imu_offset, sens.ChNoiseNone())
acc.SetName("Accelerometer")
acc.SetLag(0)
acc.SetCollectionWindow(0)
acc.PushFilter(sens.ChFilterAccelAccess())
manager.AddSensor(acc)

# === Gyroscope sensor ===
gyro = sens.ChGyroscopeSensor(platform, 100, imu_offset, sens.ChNoiseNone())
gyro.SetName("Gyroscope")
gyro.SetLag(0)
gyro.SetCollectionWindow(0)
gyro.PushFilter(sens.ChFilterGyroAccess())
manager.AddSensor(gyro)

# === Magnetometer sensor ===
mag = sens.ChMagnetometerSensor(
    platform, 100, imu_offset,
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

# Clock handler FIRST
ros_manager.RegisterHandler(chros.ChROSClockHandler())

# Body handler — publishes platform pose/twist
ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, platform, "~/output/body/data"))

# Sensor handlers
ros_manager.RegisterHandler(chros.ChROSCameraHandler(30, cam, "~/output/camera/image"))
ros_manager.RegisterHandler(
    chros.ChROSLidarHandler(lidar, "~/output/lidar/pointcloud",
                            chros.ChROSLidarHandlerMessageType_POINT_CLOUD2)
)
ros_manager.RegisterHandler(chros.ChROSGPSHandler(gps, "~/output/gps/data"))

# IMU sub-handlers — standalone + fused
acc_handler  = chros.ChROSAccelerometerHandler(acc,  "~/output/accelerometer/data")
gyro_handler = chros.ChROSGyroscopeHandler(gyro, "~/output/gyroscope/data")
mag_handler  = chros.ChROSMagnetometerHandler(mag,  "~/output/magnetometer/data")
ros_manager.RegisterHandler(acc_handler)
ros_manager.RegisterHandler(gyro_handler)
ros_manager.RegisterHandler(mag_handler)

imu_handler = chros.ChROSIMUHandler(100, "~/output/imu/data")
imu_handler.SetAccelerometerHandler(acc_handler)
imu_handler.SetGyroscopeHandler(gyro_handler)
imu_handler.SetMagnetometerHandler(mag_handler)
ros_manager.RegisterHandler(imu_handler)

# Initialize ONCE, after all registration
ros_manager.Initialize()

# === Visualization ===
# Full Irrlicht block: window + Initialize() + scene elements added AFTER Initialize
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Sensor + ROS2 Demo")
vis.Initialize()                                                        # FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(5, -5, 4), chrono.ChVector3d(0, 0, 1))
vis.AddTypicalLights()
vis.AddGrid(1.0, 1.0, 20, 20,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Review-only setup ===

# === Main loop ===
frame = 0
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            time = sys.GetChTime()
            manager.Update()   # pump sensors every physics step
            sys.DoStepDynamics(time_step)
            # ROS manager update — exit loop if ROS has shut down (turn-3 requirement)
            if not ros_manager.Update(time, time_step):
                break
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:
    import traceback
    traceback.print_exc()
    raise
finally:
    pass

# === Post-processing ===
