"""Sensor + ROS2 publishing demo (PyChrono 9.0.0, NSC system, Irrlicht review).

Models a single rotating ground body (a textured box fixed in translation but
spun about Z) that carries a suite of OptiX sensors: an RGB camera, a 2D Lidar,
a GPS, and an IMU (accelerometer + gyroscope + magnetometer). A
`ChROSPythonManager` bridges every sensor onto a ROS2 graph: the camera, GPS,
the three IMU sub-handlers + a fused IMU message, and — the focus of this scene —
a 2D Lidar whose laser-scan output is published to ``~/output/lidar2d/data/scan``.

Expected behavior: the body spins in place, so each sensor observes a changing
view of a few static landmark boxes; the lidar's single-row (2D) scan sweeps the
horizontal plane and is published as a sensor_msgs/LaserScan on the ROS2 graph.
No contact/collision dynamics are required (sensors render from collision
geometry only), so the body is driven kinematically with SetAngVelParent.
"""

import os
import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens
import pychrono.ros as chros


# === Constants === simulation timing, body geometry, sensor rates (precomputed once)
time_step = 1e-3
sim_end = 10.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))           # precomputed once

box_size = 1.0                      # protagonist body edge length (m)
spin_rate = 0.3                     # body spin about Z (rad/s)

cam_rate = 30.0                     # camera update rate (Hz)
lidar_rate = 5.0                    # 2D lidar update rate (Hz)
gps_rate = 10.0                     # GPS update rate (Hz)
imu_rate = 100.0                    # IMU update rate (Hz)

lidar_h_samples = 800               # horizontal samples around the sweep
lidar_v_samples = 1                 # 2D lidar: a SINGLE vertical row


# === System & gravity === NSC world; sensors need a collision system for OptiX geometry
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Bodies === a spun protagonist body plus static landmark boxes the sensors observe
mat = chrono.ChContactMaterialNSC()
mat.SetFriction(0.8)
mat.SetRestitution(0.0)

ground_body = chrono.ChBodyEasyBox(box_size, box_size, box_size,
                                   1000.0, True, True, mat)
ground_body.SetPos(chrono.ChVector3d(0, 0, 0))
ground_body.SetFixed(False)
ground_body.SetName("base_link")
# Drive the body kinematically: spin about world Z so sensors see a changing view.
ground_body.SetAngVelParent(chrono.ChVector3d(0, 0, spin_rate))
sys.Add(ground_body)

# Static landmark boxes give the lidar / camera something to range against.
for i, (lx, ly) in enumerate([(6, 0), (-6, 0), (0, 6), (0, -6)]):
    landmark = chrono.ChBodyEasyBox(1.0, 1.0, 2.0, 1000.0, True, True, mat)
    landmark.SetPos(chrono.ChVector3d(lx, ly, 1.0))
    landmark.SetFixed(True)
    landmark.SetName(f"landmark_{i}")
    sys.Add(landmark)

# === Sensors === ChSensorManager + camera/lidar/GPS/IMU, each with its access/visualize filters
manager = sens.ChSensorManager(sys)
intensity = 1.0
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100),
                            chrono.ChColor(intensity, intensity, intensity), 500.0)
manager.scene.AddPointLight(chrono.ChVector3f(9, 2.5, 100),
                            chrono.ChColor(intensity, intensity, intensity), 500.0)

# --- RGB camera ---
cam_offset = chrono.ChFramed(chrono.ChVector3d(-5, 0, 2),
                             chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0)))
cam = sens.ChCameraSensor(ground_body, cam_rate, cam_offset, 1280, 720, 1.408)
cam.SetName("Camera Sensor")
cam.SetLag(0)
cam.SetCollectionWindow(0)
cam.PushFilter(sens.ChFilterVisualize(1280, 720, "RGB Camera"))
cam.PushFilter(sens.ChFilterRGBA8Access())
cam.PushFilter(sens.ChFilterSave("cam/sensor_rgb/"))
manager.AddSensor(cam)

# --- 2D Lidar --- single-row horizontal sweep; named filters for point-cloud visualization
lidar_offset = chrono.ChFramed(chrono.ChVector3d(0, 0, 1),
                               chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
lidar2d = sens.ChLidarSensor(
    ground_body,                       # attach to the spinning protagonist body
    lidar_rate,                        # update rate (Hz) — physical rate
    lidar_offset,                      # offset pose on the body
    lidar_h_samples,                   # horizontal samples
    lidar_v_samples,                   # vertical samples = 1 (2D lidar)
    2 * chrono.CH_PI,                  # horizontal FOV (full circle)
    0.0,                               # max vertical angle (2D -> 0)
    0.0,                               # min vertical angle (2D -> 0)
    100.0,                             # max range (m)
    sens.LidarBeamShape_RECTANGULAR,
    2,                                 # sample radius
    0.003,                             # vertical divergence angle
    0.003,                             # horizontal divergence angle
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar2d.SetName("2D Lidar Sensor")
lidar2d.SetLag(0)
lidar2d.SetCollectionWindow(1.0 / lidar_rate)   # lidar: collection window = 1 / update_rate
# Named visualize filters so the 2D scan is inspectable; DI/PC access feed the ROS handler.
lidar2d.PushFilter(sens.ChFilterVisualize(lidar_h_samples, lidar_v_samples, "2D Lidar Depth"))
lidar2d.PushFilter(sens.ChFilterDIAccess())
lidar2d.PushFilter(sens.ChFilterPCfromDepth())
lidar2d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "2D Lidar Point Cloud"))
lidar2d.PushFilter(sens.ChFilterXYZIAccess())
manager.AddSensor(lidar2d)

# --- GPS ---
gps_offset = chrono.ChFramed(chrono.ChVector3d(0, 0, 1),
                             chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
gps = sens.ChGPSSensor(ground_body, gps_rate, gps_offset,
                       chrono.ChVector3d(-89.400, 43.070, 260.0), sens.ChNoiseNone())
gps.SetName("GPS Sensor")
gps.SetLag(0)
gps.SetCollectionWindow(0)
gps.PushFilter(sens.ChFilterGPSAccess())
manager.AddSensor(gps)

# --- IMU sub-sensors (accelerometer + gyroscope + magnetometer) ---
imu_offset = chrono.ChFramed(chrono.ChVector3d(0, 0, 1),
                             chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
acc = sens.ChAccelerometerSensor(ground_body, imu_rate, imu_offset, sens.ChNoiseNone())
acc.SetName("Accelerometer")
acc.SetLag(0); acc.SetCollectionWindow(0)
acc.PushFilter(sens.ChFilterAccelAccess())
manager.AddSensor(acc)

gyro = sens.ChGyroscopeSensor(ground_body, imu_rate, imu_offset, sens.ChNoiseNone())
gyro.SetName("Gyroscope")
gyro.SetLag(0); gyro.SetCollectionWindow(0)
gyro.PushFilter(sens.ChFilterGyroAccess())
manager.AddSensor(gyro)

mag = sens.ChMagnetometerSensor(ground_body, imu_rate, imu_offset, sens.ChNoiseNone(),
                                chrono.ChVector3d(-89.400, 43.070, 260.0))
mag.SetName("Magnetometer")
mag.SetLag(0); mag.SetCollectionWindow(0)
mag.PushFilter(sens.ChFilterMagnetAccess())
manager.AddSensor(mag)

# === ROS bridge === ChROSPythonManager + clock + one handler per sensor; 2D lidar -> LaserScan
ros_manager = chros.ChROSPythonManager()
ros_manager.RegisterHandler(chros.ChROSClockHandler())

ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, ground_body, "~/output/ground/state"))
ros_manager.RegisterHandler(chros.ChROSCameraHandler(cam_rate, cam, "~/output/camera/data/image"))

# 2D Lidar handler: publish the scan as a LaserScan message on the requested topic.
lidar2d_handler = chros.ChROSLidarHandler(
    lidar2d, "~/output/lidar2d/data/scan",
    chros.ChROSLidarHandlerMessageType_LASER_SCAN,
)
ros_manager.RegisterHandler(lidar2d_handler)

ros_manager.RegisterHandler(chros.ChROSGPSHandler(gps, "~/output/gps/data"))

acc_handler = chros.ChROSAccelerometerHandler(acc, "~/output/accelerometer/data")
gyro_handler = chros.ChROSGyroscopeHandler(gyro, "~/output/gyroscope/data")
mag_handler = chros.ChROSMagnetometerHandler(mag, "~/output/magnetometer/data")
ros_manager.RegisterHandler(acc_handler)
ros_manager.RegisterHandler(gyro_handler)
ros_manager.RegisterHandler(mag_handler)

imu_handler = chros.ChROSIMUHandler(imu_rate, "~/output/imu/data")
imu_handler.SetAccelerometerHandler(acc_handler)
imu_handler.SetGyroscopeHandler(gyro_handler)
imu_handler.SetMagnetometerHandler(mag_handler)
ros_manager.RegisterHandler(imu_handler)

ros_manager.Initialize()   # exactly once, after all handlers are registered

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Sensor + ROS2 2D Lidar publishing demo")
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(-12, -12, 8), chrono.ChVector3d(0, 0, 1))
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 40, 40,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop === render-cadence outer loop; pump sensors + ROS once per physics step
os.makedirs("cam", exist_ok=True)     # guard against missing output dir for sensor frames

try:

    frame = 0
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            manager.Update()                 # pump ALL sensors once per physics step
            t = sys.GetChTime()
            if not ros_manager.Update(t, time_step):   # publish sensor/body state to ROS2
                break
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad sensor state
    import traceback
    traceback.print_exc()
    raise
