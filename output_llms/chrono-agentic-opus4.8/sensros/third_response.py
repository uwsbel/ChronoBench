"""Multi-sensor + ROS2 publishing simulation (PyChrono 9.0.0, NSC system).

Models a rotating ground body that carries a full sensor suite — an RGB camera, a
3D lidar, a 2D (single-layer) lidar, a GPS, an accelerometer, a gyroscope and a
magnetometer — all driven by a ChSensorManager. A visualization mesh (HMMWV
chassis) is added to the system so the camera/lidar have geometry to observe. Each
sensor is wrapped by a ChROS handler and published over a ROS2 graph through a
ChROSPythonManager (clock + per-sensor topics + a fused IMU message). The ground
body is given a constant angular velocity so the sensors observe motion.

Expected behavior: the ground body spins about +Z, the sensors continuously fill
their buffers, and the ROS manager publishes camera/lidar/lidar2d/gps/imu topics
each step until the ROS layer reports shutdown or the run reaches its time bound.
"""

import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens
import pychrono.ros as chros

# === Constants === geometry / physics / sensor rates (no bare literals downstream)
TIME_STEP = 1e-3                     # solver step
SIM_END = 12.0                       # bounded recording horizon
RENDER_FPS = 50.0                    # Irrlicht review-frame cadence
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once
SPIN_RATE = 0.1                      # ground-body angular velocity about +Z (rad/s)
GPS_REFERENCE = chrono.ChVector3d(-89.4, 433.07, 260.0)        # lat/lon/alt origin


# === System & gravity === NSC multibody system (sensor scene, no contact dynamics)
sys = chrono.ChSystemNSC()

# === Bodies === a visualization mesh + the sensor-carrying ground body
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
sys.Add(mesh_body)                                  # mesh registered so sensors observe it

ground_body = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, False)
ground_body.SetPos(chrono.ChVector3d(0, 0, 0))
ground_body.SetFixed(False)
ground_body.SetMass(0)
sys.Add(ground_body)

# === Sensors === ChSensorManager + camera lighting + the full sensor suite
sens_manager = sens.ChSensorManager(sys)

intensity = 1.0
sens_manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
sens_manager.scene.AddPointLight(chrono.ChVector3f(9, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
sens_manager.scene.AddPointLight(chrono.ChVector3f(16, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
sens_manager.scene.AddPointLight(chrono.ChVector3f(23, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)

offset_pose = chrono.ChFramed(chrono.ChVector3d(-8, 0, 2), chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0)))

# RGB camera — physical 30 Hz update rate; visualize + host-access filters.
cam = sens.ChCameraSensor(ground_body, 30, offset_pose, 1280, 720, 1.408)
cam.PushFilter(sens.ChFilterVisualize(1280, 720))
cam.PushFilter(sens.ChFilterRGBA8Access())
cam.SetName("camera")
sens_manager.AddSensor(cam)

# 3D lidar — 90 x 300 beams over a full horizontal sweep, depth -> XYZI point cloud.
lidar = sens.ChLidarSensor(ground_body, 5.0, offset_pose, 90, 300, 2 * chrono.CH_PI,
                           chrono.CH_PI / 12, -chrono.CH_PI / 6, 100.0, 0)
lidar.PushFilter(sens.ChFilterDIAccess())            # host access to depth+intensity
lidar.PushFilter(sens.ChFilterPCfromDepth())         # depth -> XYZI point cloud
lidar.PushFilter(sens.ChFilterXYZIAccess())          # host access to XYZI
lidar.PushFilter(sens.ChFilterVisualizePointCloud(1280, 720, 1, "Lidar PC data"))
lidar.SetName("lidar")
sens_manager.AddSensor(lidar)

# 2D lidar — single vertical layer (v_samples = 1) -> a planar laser scan.
offset_pose_2dlidar = chrono.ChFramed(chrono.ChVector3d(-8, 0, 0), chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0)))
lidar2d = sens.ChLidarSensor(ground_body, 5, offset_pose_2dlidar, 480, 1, 2 * chrono.CH_PI,
                             chrono.CH_PI / 12, -chrono.CH_PI / 6, 100.0)
lidar2d.PushFilter(sens.ChFilterDIAccess())
lidar2d.PushFilter(sens.ChFilterPCfromDepth())
lidar2d.PushFilter(sens.ChFilterXYZIAccess())
lidar2d.PushFilter(sens.ChFilterVisualizePointCloud(1280, 720, 1, "2D Lidar Scan Data"))
lidar2d.SetName("lidar2d")
sens_manager.AddSensor(lidar2d)

# GPS / IMU-family sensors — no rendering, no lights; share one no-noise model.
noise_model_none = sens.ChNoiseNone()
gps = sens.ChGPSSensor(ground_body, 10, offset_pose, GPS_REFERENCE, noise_model_none)
gps.PushFilter(sens.ChFilterGPSAccess())
gps.SetName("gps")
sens_manager.AddSensor(gps)

acc = sens.ChAccelerometerSensor(ground_body, 100, offset_pose, noise_model_none)
acc.PushFilter(sens.ChFilterAccelAccess())
acc.SetName("accelerometer")
sens_manager.AddSensor(acc)

gyro = sens.ChGyroscopeSensor(ground_body, 100, offset_pose, noise_model_none)
gyro.PushFilter(sens.ChFilterGyroAccess())
gyro.SetName("gyroscope")
sens_manager.AddSensor(gyro)

mag = sens.ChMagnetometerSensor(ground_body, 100, offset_pose, noise_model_none, GPS_REFERENCE)
mag.PushFilter(sens.ChFilterMagnetAccess())
mag.SetName("magnetometer")
sens_manager.AddSensor(mag)

sens_manager.Update()                                # prime buffers before publishing

# === ROS publishing === ChROSPythonManager with clock + per-sensor + fused-IMU handlers
ros_manager = chros.ChROSPythonManager()
ros_manager.RegisterHandler(chros.ChROSClockHandler())   # /clock FIRST — time-sync the graph

ros_manager.RegisterHandler(chros.ChROSCameraHandler(cam.GetUpdateRate() / 4, cam, "~/output/camera/data/image"))
ros_manager.RegisterHandler(chros.ChROSLidarHandler(lidar, "~/output/lidar/data/pointcloud"))
ros_manager.RegisterHandler(chros.ChROSLidarHandler(lidar2d, "~/output/lidar2d/data/scan",
                                                     chros.ChROSLidarHandlerMessageType_LASER_SCAN))
ros_manager.RegisterHandler(chros.ChROSGPSHandler(gps, "~/output/gps/data"))
acc_handler = chros.ChROSAccelerometerHandler(acc, "~/output/accelerometer/data")
ros_manager.RegisterHandler(acc_handler)
gyro_handler = chros.ChROSGyroscopeHandler(gyro, "~/output/gyroscope/data")
ros_manager.RegisterHandler(gyro_handler)
mag_handler = chros.ChROSMagnetometerHandler(mag, "~/output/magnetometer/data")
ros_manager.RegisterHandler(mag_handler)

imu_handler = chros.ChROSIMUHandler(100, "~/output/imu/data")    # fused sensor_msgs/Imu
imu_handler.SetAccelerometerHandler(acc_handler)
imu_handler.SetGyroscopeHandler(gyro_handler)
imu_handler.SetMagnetometerHandler(mag_handler)
ros_manager.RegisterHandler(imu_handler)

ros_manager.Initialize()                             # exactly once, after all registration

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Multi-sensor ROS publisher")
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(-8, -6, 4), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 40, 40,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop === spin the ground body; pump sensors, publish to ROS, step physics
ground_body.SetAngVelParent(chrono.ChVector3d(0, 0, SPIN_RATE))   # constant +Z spin

os.makedirs("cam", exist_ok=True)              # guard against missing output dir
try:

    frame = 0
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            time = sys.GetChTime()
            sens_manager.Update()                    # pump every sensor each physics step
            if not ros_manager.Update(time, TIME_STEP):   # publish; stop if ROS shut down
                break
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:        # solver divergence / bad sensor state
    import traceback
    traceback.print_exc()
    raise
