"""
sensros simulation — sensor + ROS2 bridge demo.

Models a moving body that carries a full sensor suite (camera, 2D lidar, GPS,
accelerometer, gyroscope, magnetometer) on a ChSystemNSC. The body spins
continuously so all sensors observe dynamic motion. Each sensor is published
over ROS2 via a matching ChROS*Handler; IMU sub-handlers are also fused into a
ChROSIMUHandler. A mesh visual is added to the spinning body and added to the
system. The 2D lidar uses ChFilterDIAccess + ChFilterPCfromDepth + ChFilterXYZIAccess
for host access. The simulation loop breaks if the ROS manager update fails.

System: ChSystemNSC (NSC, no collision needed — no contact in this scene)
Bodies: floor (fixed), spinning ground_body carrying sensors, mesh_body (fixed visual)
Expected behavior: sensors publish data to ROS topics; body spins for sensor_lifetime.
"""

import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens
import pychrono.ros as chros

# === Constants ===
TIME_STEP = 1e-3         # physics time step (s)
SIM_END   = 30.0         # simulation duration (s)
RENDER_FPS = 50.0        # Irrlicht render cadence
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# Sensor update rates (physical Hz — not 1/time_step)
CAM_RATE   = 30
LIDAR_RATE = 5
GPS_RATE   = 10
IMU_RATE   = 100

# Body geometry
BODY_HALF  = 0.5   # half-side of the spinning box (m)
SPIN_RATE  = 1.0   # angular velocity (rad/s) about Z

# GPS reference origin
GPS_REF = chrono.ChVector3d(-89.400, 43.070, 260.0)

# === System ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
# No contact shapes in this scene — collision system omitted intentionally.

# === Bodies ===
# Floor (fixed reference)
floor = chrono.ChBodyEasyBox(10, 10, 0.1, 1000, True, False)
floor.SetFixed(True)
floor.SetPos(chrono.ChVector3d(0, 0, -0.05))
floor.SetName("base_link")
sys.Add(floor)

# Spinning body carrying all sensors
ground_body = chrono.ChBodyEasyBox(
    BODY_HALF * 2, BODY_HALF * 2, BODY_HALF * 2, 1000, True, False
)
ground_body.SetPos(chrono.ChVector3d(0, 0, BODY_HALF + 0.1))
ground_body.SetFixed(False)
ground_body.SetAngVelParent(chrono.ChVector3d(0, 0, SPIN_RATE))
ground_body.SetName("spinning_body")
sys.Add(ground_body)

# Mesh body — added to the simulation system so the mesh is visible
mesh_body = chrono.ChBody()
mesh_body.SetFixed(True)
mesh_body.SetPos(chrono.ChVector3d(3, 0, 0.5))
mesh_body.SetName("mesh_body")
mesh_shape = chrono.ChVisualShapeModelFile()
mesh_shape.SetFilename(chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"))
mesh_body.AddVisualShape(mesh_shape)
sys.Add(mesh_body)   # added to simulation system

# === Sensor Manager ===
manager = sens.ChSensorManager(sys)

# Point lights for camera rendering
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
    chrono.ChVector3d(-8, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
cam = sens.ChCameraSensor(ground_body, CAM_RATE, cam_offset, 1280, 720, 1.408)
cam.SetName("Camera Sensor")
cam.SetLag(0)
cam.SetCollectionWindow(0)
cam.PushFilter(sens.ChFilterVisualize(1280, 720, "RGB Camera"))
cam.PushFilter(sens.ChFilterRGBA8Access())
cam.PushFilter(sens.ChFilterSave("cam/rgb/"))
manager.AddSensor(cam)

# === 2D Lidar Sensor ===
# v_samples=1, both vertical angles=0 → 2D scan
lidar_offset = chrono.ChFramed(
    chrono.ChVector3d(-12, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
H_SAMPLES = 800
lidar = sens.ChLidarSensor(
    ground_body,
    float(LIDAR_RATE),
    lidar_offset,
    H_SAMPLES,          # h_samples
    1,                  # v_samples = 1 → 2D lidar
    2 * chrono.CH_PI,   # horizontal_fov
    0,                  # max_vert_angle = 0 (2D)
    0,                  # min_vert_angle = 0 (2D)
    100.0,              # max_range (m)
    sens.LidarBeamShape_RECTANGULAR,
    2,                  # sample_radius
    0.003,              # vert divergence angle
    0.003,              # hori divergence angle
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(1.0 / LIDAR_RATE)   # collection window = 1/update_rate

# 2D lidar filter chain — ChFilterDIAccess + ChFilterPCfromDepth + ChFilterXYZIAccess
lidar.PushFilter(sens.ChFilterVisualize(H_SAMPLES, 1, "Raw Lidar Depth"))
lidar.PushFilter(sens.ChFilterDIAccess())          # host access to depth+intensity
lidar.PushFilter(sens.ChFilterPCfromDepth())       # convert depth → XYZI point cloud
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
lidar.PushFilter(sens.ChFilterXYZIAccess())        # host access to XYZI
manager.AddSensor(lidar)

# === GPS Sensor ===
gps_offset = chrono.ChFramed(
    chrono.ChVector3d(-6, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
gps = sens.ChGPSSensor(ground_body, float(GPS_RATE), gps_offset, GPS_REF, sens.ChNoiseNone())
gps.SetName("GPS Sensor")
gps.SetLag(0)
gps.SetCollectionWindow(0)
gps.PushFilter(sens.ChFilterGPSAccess())
manager.AddSensor(gps)

# === Accelerometer (IMU) ===
acc_offset = chrono.ChFramed(
    chrono.ChVector3d(-4, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
acc = sens.ChAccelerometerSensor(ground_body, float(IMU_RATE), acc_offset, sens.ChNoiseNone())
acc.SetName("Accelerometer Sensor")
acc.SetLag(0)
acc.SetCollectionWindow(0)
acc.PushFilter(sens.ChFilterAccelAccess())
manager.AddSensor(acc)

# === Gyroscope ===
gyro_offset = chrono.ChFramed(
    chrono.ChVector3d(-4, 0.5, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
gyro = sens.ChGyroscopeSensor(ground_body, float(IMU_RATE), gyro_offset, sens.ChNoiseNone())
gyro.SetName("Gyroscope Sensor")
gyro.SetLag(0)
gyro.SetCollectionWindow(0)
gyro.PushFilter(sens.ChFilterGyroAccess())
manager.AddSensor(gyro)

# === Magnetometer ===
mag_offset = chrono.ChFramed(
    chrono.ChVector3d(-4, -0.5, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
mag = sens.ChMagnetometerSensor(
    ground_body, float(IMU_RATE), mag_offset, sens.ChNoiseNone(), GPS_REF
)
mag.SetName("Magnetometer Sensor")
mag.SetLag(0)
mag.SetCollectionWindow(0)
mag.PushFilter(sens.ChFilterMagnetAccess())
manager.AddSensor(mag)

# === ROS Manager ===
ros_manager = chros.ChROSPythonManager()

# Clock handler first (publishes /clock)
ros_manager.RegisterHandler(chros.ChROSClockHandler())

# Camera handler
ros_manager.RegisterHandler(chros.ChROSCameraHandler(CAM_RATE, cam, "~/output/camera/data"))

# Lidar handler
ros_manager.RegisterHandler(chros.ChROSLidarHandler(lidar, "~/output/lidar/data"))

# GPS handler
ros_manager.RegisterHandler(chros.ChROSGPSHandler(gps, "~/output/gps/data"))

# Accelerometer, gyroscope, magnetometer handlers (standalone)
acc_handler  = chros.ChROSAccelerometerHandler(acc, "~/output/accelerometer/data")
gyro_handler = chros.ChROSGyroscopeHandler(gyro, "~/output/gyroscope/data")
mag_handler  = chros.ChROSMagnetometerHandler(mag, "~/output/magnetometer/data")
ros_manager.RegisterHandler(acc_handler)
ros_manager.RegisterHandler(gyro_handler)
ros_manager.RegisterHandler(mag_handler)

# Fused IMU handler
imu_handler = chros.ChROSIMUHandler(IMU_RATE, "~/output/imu/data")
imu_handler.SetAccelerometerHandler(acc_handler)
imu_handler.SetGyroscopeHandler(gyro_handler)
imu_handler.SetMagnetometerHandler(mag_handler)
ros_manager.RegisterHandler(imu_handler)

# Body handler for the spinning body
ros_manager.RegisterHandler(
    chros.ChROSBodyHandler(25, ground_body, "~/output/body/data")
)

# Initialize exactly once, after all handlers registered
ros_manager.Initialize()

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("sensros — Sensor + ROS2 Bridge Demo")
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.Initialize()                                          # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(5, 5, 3), chrono.ChVector3d(0, 0, 0.5))
vis.AddTypicalLights()

# === Review-only setup ===


# === Main Loop ===
frame = 0
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            manager.Update()   # pump all sensors every physics step
            sys.DoStepDynamics(TIME_STEP)
            time = sys.GetChTime()


            # ROS update — check return status; break if ROS has shut down
            if not ros_manager.Update(time, TIME_STEP):
                break
            if time >= SIM_END:
                break

except (RuntimeError, ValueError) as exc:    # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
