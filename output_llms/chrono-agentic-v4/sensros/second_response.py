"""
Sensros Turn 2: Sensor + ROS scene with 2D Lidar and ROS handler.
A moving ground body carries camera, 2D lidar, GPS, accelerometer, gyroscope,
magnetometer sensors. Each sensor has a ROS handler. The body is spun with
SetAngVelParent so sensors see motion.

System: ChSystemNSC
Sensors: camera, 2D lidar, GPS, accel, gyro, mag
ROS: ChROSPythonManager with handlers for all sensors + fused IMU
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
CAMERA_RATE = 30.0
LIDAR_RATE = 5.0
GPS_RATE = 10.0
IMU_RATE = 100.0

# Body motion
ANG_VEL_X = 0.5
ANG_VEL_Y = 0.3

# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Ground body (moving, carries all sensors) ===
ground_density = 1000.0
ground_size_x = 2.0
ground_size_y = 0.1
ground_size_z = 2.0

mat_ground = chrono.ChContactMaterialNSC()
mat_ground.SetFriction(0.5)
mat_ground.SetRestitution(0.1)

ground_body = chrono.ChBodyEasyBox(
    ground_size_x, ground_size_y, ground_size_z,
    ground_density, True, True, mat_ground
)
ground_body.SetPos(chrono.ChVector3d(0.0, 0.0, 0.0))
# Spin the body so sensors see motion
ground_body.SetAngVelParent(chrono.ChVector3d(ANG_VEL_X, ANG_VEL_Y, 0.0))
ground_body.SetFixed(False)
sys.AddBody(ground_body)

# === Sensor Manager ===
manager = sens.ChSensorManager(sys)

# Scene lighting for camera (point lights)
intensity = 1.0
manager.scene.AddPointLight(
    chrono.ChVector3f(2, 2.5, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(-5, 3.0, 50),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)

# === Camera Sensor ===
cam_offset_pose = chrono.ChFramed(
    chrono.ChVector3d(-5, 0, 2),
    chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0))
)
cam = sens.ChCameraSensor(
    ground_body,
    CAMERA_RATE,
    cam_offset_pose,
    1280, 720, 1.408,
)
cam.SetName("Camera Sensor")
cam.SetLag(0)
cam.SetCollectionWindow(0)
cam.PushFilter(sens.ChFilterVisualize(1280, 720, "Camera"))
cam.PushFilter(sens.ChFilterRGBA8Access())
cam.PushFilter(sens.ChFilterSave("cam/rgb/"))
manager.AddSensor(cam)

# === 2D Lidar Sensor (v_samples=1 for 2D) ===
lidar_offset_pose = chrono.ChFramed(
    chrono.ChVector3d(0, 0, 1),
    chrono.QuatFromAngleAxis(0.0, chrono.ChVector3d(0, 1, 0))
)
lidar = sens.ChLidarSensor(
    ground_body,
    LIDAR_RATE,
    lidar_offset_pose,
    800,    # horizontal_samples
    1,      # vertical_samples (1 = 2D lidar)
    2 * chrono.CH_PI,          # horizontal_fov
    0.0,                          # max_vert_angle
    0.0,                          # min_vert_angle
    100.0,                        # max_range
    sens.LidarBeamShape_RECTANGULAR,
    2,                            # sample_radius
    0.003,                        # vert_divergence_angle
    0.003,                        # hori_divergence_angle
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar.SetName("2D Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(1.0 / LIDAR_RATE)

# Lidar filter chain - with names for visualization
lidar.PushFilter(sens.ChFilterVisualize(800, 1, "Raw Lidar Depth"))
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
lidar.PushFilter(sens.ChFilterXYZIAccess())
manager.AddSensor(lidar)

# === GPS Sensor ===
gps_offset_pose = chrono.ChFramed(
    chrono.ChVector3d(-1, 0, 0),
    chrono.QuatFromAngleAxis(0.0, chrono.ChVector3d(0, 1, 0))
)
gps = sens.ChGPSSensor(
    ground_body,
    GPS_RATE,
    gps_offset_pose,
    chrono.ChVector3d(-89.400, 43.070, 260.0),  # reference lat/lon/alt
    sens.ChNoiseNone(),
)
gps.SetName("GPS Sensor")
gps.SetLag(0)
gps.SetCollectionWindow(0)
gps.PushFilter(sens.ChFilterGPSAccess())
manager.AddSensor(gps)

# === Accelerometer Sensor ===
imu_offset_pose = chrono.ChFramed(
    chrono.ChVector3d(1, 0, 0),
    chrono.QuatFromAngleAxis(0.0, chrono.ChVector3d(0, 1, 0))
)
acc = sens.ChAccelerometerSensor(
    ground_body,
    IMU_RATE,
    imu_offset_pose,
    sens.ChNoiseNone(),
)
acc.SetName("Accelerometer Sensor")
acc.SetLag(0)
acc.SetCollectionWindow(0)
acc.PushFilter(sens.ChFilterAccelAccess())
manager.AddSensor(acc)

# === Gyroscope Sensor ===
gyro = sens.ChGyroscopeSensor(
    ground_body,
    IMU_RATE,
    imu_offset_pose,
    sens.ChNoiseNone(),
)
gyro.SetName("Gyroscope Sensor")
gyro.SetLag(0)
gyro.SetCollectionWindow(0)
gyro.PushFilter(sens.ChFilterGyroAccess())
manager.AddSensor(gyro)

# === Magnetometer Sensor ===
mag_reference = chrono.ChVector3d(-89.400, 43.070, 260.0)
mag = sens.ChMagnetometerSensor(
    ground_body,
    IMU_RATE,
    imu_offset_pose,
    sens.ChNoiseNone(),
    mag_reference,
)
mag.SetName("Magnetometer Sensor")
mag.SetLag(0)
mag.SetCollectionWindow(0)
mag.PushFilter(sens.ChFilterMagnetAccess())
manager.AddSensor(mag)

# === ROS Manager ===
ros_manager = chros.ChROSPythonManager()
ros_manager.RegisterHandler(chros.ChROSClockHandler())

# Camera ROS handler
ros_cam = chros.ChROSCameraHandler(cam, "~/output/camera/data/image")
ros_manager.RegisterHandler(ros_cam)

# 2D Lidar ROS handler - NEW for turn 2
ros_lidar = chros.ChROSLidarHandler(lidar, "~/output/lidar2d/data/scan")
ros_manager.RegisterHandler(ros_lidar)

# GPS ROS handler
ros_gps = chros.ChROSGPSHandler(gps, "~/output/gps/data")
ros_manager.RegisterHandler(ros_gps)

# Accelerometer ROS handler
ros_acc = chros.ChROSAccelerometerHandler(acc, "~/output/accelerometer/data")
ros_manager.RegisterHandler(ros_acc)

# Gyroscope ROS handler
ros_gyro = chros.ChROSGyroscopeHandler(gyro, "~/output/gyroscope/data")
ros_manager.RegisterHandler(ros_gyro)

# Magnetometer ROS handler
ros_mag = chros.ChROSMagnetometerHandler(mag, "~/output/magnetometer/data")
ros_manager.RegisterHandler(ros_mag)

# Fused IMU handler
ros_imu = chros.ChROSIMUHandler(IMU_RATE, "~/output/imu/data")
ros_imu.SetAccelerometerHandler(ros_acc)
ros_imu.SetGyroscopeHandler(ros_gyro)
ros_imu.SetMagnetometerHandler(ros_mag)
ros_manager.RegisterHandler(ros_imu)

ros_manager.Initialize()

# === Irrlicht Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Sensros Turn 2 - 2D Lidar + ROS")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(8, -8, 4), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 40, 40, chrono.ChCoordsysd(), chrono.ChColor(0.4, 0.4, 0.4))

# === CSV Logging ===
os.makedirs("cam", exist_ok=True)
csv_file = open("simulation_data.csv", "w", newline="")
fieldnames = ["time", "body_x", "body_y", "body_z", "body_angvel_x", "body_angvel_y", "body_angvel_z"]
data_writer = csv_file.__enter__()
import csv as csv_module
data_writer = csv_module.DictWriter(csv_file, fieldnames=fieldnames)
data_writer.writeheader()

# === Main Loop ===
frame = 0
while vis.Run() and sys.GetChTime() < SIM_END:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    if bool(os.environ.get("SIMBENCH_RECORD")):
        vis.WriteImageToFile(f"cam/frames/img_{frame:06d}.png")
    frame += 1

    for _ in range(RENDER_EVERY):
        time = sys.GetChTime()

        # Update all sensors
        manager.Update()

        # ROS update
        if not ros_manager.Update(time, TIME_STEP):
            break

        # Log CSV
        data_writer.writerow({
            "time": time,
            "body_x": ground_body.GetPos().x,
            "body_y": ground_body.GetPos().y,
            "body_z": ground_body.GetPos().z,
            "body_angvel_x": ground_body.GetAngVelParent().x,
            "body_angvel_y": ground_body.GetAngVelParent().y,
            "body_angvel_z": ground_body.GetAngVelParent().z,
        })

        # Advance physics
        sys.DoStepDynamics(TIME_STEP)

        if sys.GetChTime() >= SIM_END:
            break

csv_file.close()

# === Review-only: Assemble Videos ===
if bool(os.environ.get("SIMBENCH_RECORD")):
    import sim_recording as rec
    irr_dir = rec.frame_dir("cam/frames")
    rec.assemble_all_videos(irr_dir, sensor_dirs=["cam/rgb"])
    rec.cleanup_frames(irr_dir, "cam/rgb")
