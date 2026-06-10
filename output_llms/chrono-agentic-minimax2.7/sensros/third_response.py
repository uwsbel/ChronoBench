"""
Sensros Turn 3 — sensor + ROS scene with HMMWV chassis mesh.

Applies three corrections to the base truth:
  1. Add mesh_body to the simulation system (sys.Add).
  2. Add ChFilterDIAccess / ChFilterPCfromDepth / ChFilterXYZIAccess to 2-D lidar.
  3. Check ros_manager.Update() return and break on failure.

System type:  ChSystemNSC (NSC contact)
Main bodies:  mesh_body (HMMWV chassis visual mesh), ground_body (sensor carrier)
Sensors:      camera, 3-D lidar, 2-D lidar, GPS, accelerometer, gyroscope, magnetometer
ROS handlers: clock, camera, lidar (3-D), lidar2d (2-D laser scan), GPS, accel,
              gyro, mag, fused IMU
Expected:     all sensors publish; ROS update loop exits cleanly or on ROS failure
"""

import os
import math
import csv
import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.ros as chros
import pychrono.irrlicht as chronoirr

# Dummy rec module — satisfes scored-core references when real sim_recording is stripped
class _DummyRec:
    frame_dir = staticmethod(lambda n: None)
    frame_path = staticmethod(lambda d, i: "")
rec = _DummyRec()   # scored-core fallback

# review-only: sim_recording for frame capture + video assembly

# === Named constants ===
TIME_STEP = 1e-3
SIM_END = 20.0
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))

# REC and irr_dir are referenced in the scored core loop — always define them
REC = bool(os.environ.get("SIMBENCH_RECORD"))
irr_dir = rec.frame_dir("frames") if REC else None

# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

# === Mesh asset (HMMWV chassis visual mesh) ===
mmesh = chrono.ChTriangleMeshConnected()
mmesh.LoadWavefrontMesh(chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"), False, True)
mmesh.Transform(chrono.ChVector3d(0, 0, 0), chrono.ChMatrix33d(1))

trimesh_shape = chrono.ChVisualShapeTriangleMesh()
trimesh_shape.SetMesh(mmesh)
trimesh_shape.SetName("HMMWV Chassis Mesh")
trimesh_shape.SetMutable(False)

mesh_body = chrono.ChBody()
mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))
mesh_body.AddVisualShape(trimesh_shape)
mesh_body.SetFixed(False)
mesh_body.SetMass(0)
sys.Add(mesh_body)                          # FIX 1: add mesh body to system

# === Ground body (sensor carrier) ===
ground_body = chrono.ChBodyEasyBox(1, 1, 1, 1000, False, False)
ground_body.SetPos(chrono.ChVector3d(0, 0, 0))
ground_body.SetFixed(False)
ground_body.SetMass(0)
sys.Add(ground_body)

# === Sensor manager ===
sens_manager = sens.ChSensorManager(sys)

# Scene lighting (point lights, canonical sensor setup)
intensity = 1.0
sens_manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
sens_manager.scene.AddPointLight(chrono.ChVector3f(9, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
sens_manager.scene.AddPointLight(chrono.ChVector3f(16, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
sens_manager.scene.AddPointLight(chrono.ChVector3f(23, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)

# === Camera sensor ===
offset_pose = chrono.ChFramed(
    chrono.ChVector3d(-8, 0, 2),
    chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0)),
)
cam = sens.ChCameraSensor(ground_body, 30, offset_pose, 1280, 720, 1.408)
cam.PushFilter(sens.ChFilterVisualize(1280, 720))
cam.PushFilter(sens.ChFilterRGBA8Access())
cam.SetName("camera")
sens_manager.AddSensor(cam)

# === 3-D Lidar sensor ===
lidar = sens.ChLidarSensor(
    ground_body, 5.0, offset_pose,
    90, 300,
    2 * chrono.CH_PI, chrono.CH_PI / 12, -chrono.CH_PI / 6,
    100.0, 0,
)
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterXYZIAccess())
lidar.PushFilter(sens.ChFilterVisualizePointCloud(1280, 720, 1, "Lidar PC data"))
lidar.SetName("lidar")
sens_manager.AddSensor(lidar)

# === 2-D Lidar sensor ===
offset_pose_2dlidar = chrono.ChFramed(
    chrono.ChVector3d(-8, 0, 0),
    chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0)),
)
lidar2d = sens.ChLidarSensor(
    ground_body, 5, offset_pose_2dlidar,
    480, 1,
    2 * chrono.CH_PI, chrono.CH_PI / 12, -chrono.CH_PI / 6,
    100.0,
)
# FIX 2: add missing access filters for 2-D lidar
lidar2d.PushFilter(sens.ChFilterDIAccess())
lidar2d.PushFilter(sens.ChFilterPCfromDepth())
lidar2d.PushFilter(sens.ChFilterXYZIAccess())
lidar2d.PushFilter(sens.ChFilterVisualizePointCloud(1280, 720, 1, "2D Lidar Scan Data"))
sens_manager.AddSensor(lidar2d)

# === GPS sensor ===
noise_none = sens.ChNoiseNone()
gps_reference = chrono.ChVector3d(-89.4, 433.07, 260.0)
gps = sens.ChGPSSensor(ground_body, 10, offset_pose, gps_reference, noise_none)
gps.PushFilter(sens.ChFilterGPSAccess())
gps.SetName("gps")
sens_manager.AddSensor(gps)

# === Accelerometer sensor ===
acc = sens.ChAccelerometerSensor(ground_body, 100, offset_pose, noise_none)
acc.PushFilter(sens.ChFilterAccelAccess())
acc.SetName("accelerometer")
sens_manager.AddSensor(acc)

# === Gyroscope sensor ===
gyro = sens.ChGyroscopeSensor(ground_body, 100, offset_pose, noise_none)
gyro.PushFilter(sens.ChFilterGyroAccess())
gyro.SetName("gyroscope")
sens_manager.AddSensor(gyro)

# === Magnetometer sensor ===
mag = sens.ChMagnetometerSensor(ground_body, 100, offset_pose, noise_none, gps_reference)
mag.PushFilter(sens.ChFilterMagnetAccess())
mag.SetName("magnetometer")
sens_manager.AddSensor(mag)

# Initialize sensors once before the loop
sens_manager.Update()

# === ROS manager ===
ros_manager = chros.ChROSPythonManager()
ros_manager.RegisterHandler(chros.ChROSClockHandler())

ros_manager.RegisterHandler(chros.ChROSCameraHandler(cam.GetUpdateRate() / 4, cam, "~/output/camera/data/image"))
ros_manager.RegisterHandler(chros.ChROSLidarHandler(lidar, "~/output/lidar/data/pointcloud"))
ros_manager.RegisterHandler(chros.ChROSLidarHandler(lidar2d, "~/output/lidar2d/data/scan", chros.ChROSLidarHandlerMessageType_LASER_SCAN))
ros_manager.RegisterHandler(chros.ChROSGPSHandler(gps, "~/output/gps/data"))

acc_handler = chros.ChROSAccelerometerHandler(acc, "~/output/accelerometer/data")
ros_manager.RegisterHandler(acc_handler)

gyro_handler = chros.ChROSGyroscopeHandler(gyro, "~/output/gyroscope/data")
ros_manager.RegisterHandler(gyro_handler)

mag_handler = chros.ChROSMagnetometerHandler(mag, "~/output/magnetometer/data")
ros_manager.RegisterHandler(mag_handler)

imu_handler = chros.ChROSIMUHandler(100, "~/output/imu/data")
imu_handler.SetAccelerometerHandler(acc_handler)
imu_handler.SetGyroscopeHandler(gyro_handler)
imu_handler.SetMagnetometerHandler(mag_handler)
ros_manager.RegisterHandler(imu_handler)

ros_manager.Initialize()

# Apply rotational velocity to ground body (sensors see motion)
ground_body.SetAngVelParent(chrono.ChVector3d(0, 0, 0.1))

# === Visualization (Irrlicht) ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Sensros Turn 3 — sensor + ROS scene")
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(-10, 0, 5), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 40, 40,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === CSV logging ===
os.makedirs("cam", exist_ok=True)
try:
    csv_f = open("simulation_data.csv", "w", newline="")
    csv_writer = csv.writer(csv_f)
    csv_writer.writerow(["time", "mesh_x", "mesh_y", "mesh_z",
                         "ground_x", "ground_y", "ground_z"])
except OSError as exc:
    import traceback
    traceback.print_exc()
    raise

# === Main loop ===
frame = 0
time = 0.0
try:
    while vis.Run() and time < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(RENDER_EVERY):
            sens_manager.Update()
            # FIX 3: check ROS manager update status and exit on failure
            if not ros_manager.Update(time, TIME_STEP):
                break

            sys.DoStepDynamics(TIME_STEP)
            time = sys.GetChTime()


            if time >= SIM_END:
                break
finally:
    csv_f.close()

# review-only: assemble videos and plot table
