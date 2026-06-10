"""Multi-sensor suite on a rotating ground body, published over ROS2.

Models an NSC system containing a visual HMMWV-chassis mesh and a movable
``ground_body`` carrying a full sensor stack: an RGB camera, a lidar, a GPS,
an accelerometer, a gyroscope and a magnetometer, all overseen by a single
``ChSensorManager``. Each sensor is bridged to a ROS2 topic through a matching
``ChROS*Handler`` (plus a fused IMU handler) registered on a
``ChROSPythonManager`` whose clock handler keeps the ROS graph time-synced.

The ground body is given a constant z-axis angular velocity so the attached
sensors perceive motion; the gyroscope therefore reads ~0.1 rad/s on z. The
scene has no contact (sensors only ride a kinematically spun body), so no
collision system is configured. Expected behavior: a steadily yawing sensor
rig streaming camera/lidar/gps/imu data to ROS topics for the run duration.
"""

import os
import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens
import pychrono.ros as chros


# === Constants === sim timing + render cadence + sensor rates
time_step = 1e-3
sim_end = 30.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))   # precomputed once
spin_rate = 0.1                       # ground-body yaw rate (rad/s) about world z
gps_reference = chrono.ChVector3d(-89.4, 433.07, 260.0)        # lat/lon/alt origin


# === System & gravity === NSC world; no contact, so no collision system
sys = chrono.ChSystemNSC()

# === Bodies === a visual mesh for scene context + the movable sensor carrier
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
sys.Add(mesh_body)

# The body all sensors are attached to; spun about z so the sensors see motion.
ground_body = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, False)
ground_body.SetPos(chrono.ChVector3d(0, 0, 0))
ground_body.SetFixed(False)
ground_body.SetMass(0)
ground_body.SetName("ground_body")
sys.Add(ground_body)

# === Sensors === one ChSensorManager oversees the whole stack
sens_manager = sens.ChSensorManager(sys)
intensity = 1.0
sens_manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
sens_manager.scene.AddPointLight(chrono.ChVector3f(9, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
sens_manager.scene.AddPointLight(chrono.ChVector3f(16, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
sens_manager.scene.AddPointLight(chrono.ChVector3f(23, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)

# Shared offset pose: camera/lidar look back along -x and slightly up.
offset_pose = chrono.ChFramed(chrono.ChVector3d(-8, 0, 2),
                              chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0)))

# Camera: RGB image stream published to ROS; save its own frames for review.
cam = sens.ChCameraSensor(ground_body, 30, offset_pose, 1280, 720, 1.408)
cam.SetName("camera")
cam.SetLag(0)
cam.SetCollectionWindow(0)
cam.PushFilter(sens.ChFilterVisualize(1280, 720))
cam.PushFilter(sens.ChFilterRGBA8Access())
cam.PushFilter(sens.ChFilterSave("cam/camera/"))
sens_manager.AddSensor(cam)

# Lidar: depth -> XYZI point cloud, with a live point-cloud preview.
lidar = sens.ChLidarSensor(ground_body, 5.0, offset_pose, 90, 300,
                           2 * chrono.CH_PI, chrono.CH_PI / 12, -chrono.CH_PI / 6, 100.0, 0)
lidar.SetName("lidar")
lidar.SetLag(0)
lidar.SetCollectionWindow(1.0 / 5.0)
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterXYZIAccess())
lidar.PushFilter(sens.ChFilterVisualizePointCloud(1280, 720, 1))
sens_manager.AddSensor(lidar)

noise_model_none = sens.ChNoiseNone()

# GPS: NavSatFix relative to the lat/lon/alt reference origin.
gps = sens.ChGPSSensor(ground_body, 10, offset_pose, gps_reference, noise_model_none)
gps.SetName("gps")
gps.SetLag(0)
gps.SetCollectionWindow(0)
gps.PushFilter(sens.ChFilterGPSAccess())
sens_manager.AddSensor(gps)

# Accelerometer.
acc = sens.ChAccelerometerSensor(ground_body, 100, offset_pose, noise_model_none)
acc.SetName("accelerometer")
acc.SetLag(0)
acc.SetCollectionWindow(0)
acc.PushFilter(sens.ChFilterAccelAccess())
sens_manager.AddSensor(acc)

# Gyroscope: reads the imposed ~0.1 rad/s about z.
gyro = sens.ChGyroscopeSensor(ground_body, 100, offset_pose, noise_model_none)
gyro.SetName("gyroscope")
gyro.SetLag(0)
gyro.SetCollectionWindow(0)
gyro.PushFilter(sens.ChFilterGyroAccess())
sens_manager.AddSensor(gyro)

# Magnetometer: needs the geographic reference for the field model.
mag = sens.ChMagnetometerSensor(ground_body, 100, offset_pose, noise_model_none, gps_reference)
mag.SetName("magnetometer")
mag.SetLag(0)
mag.SetCollectionWindow(0)
mag.PushFilter(sens.ChFilterMagnetAccess())
sens_manager.AddSensor(mag)

sens_manager.Update()   # prime the sensor buffers before the first publish

# === ROS bridge === one handler per sensor + a fused IMU + the clock
ros_manager = chros.ChROSPythonManager()
ros_manager.RegisterHandler(chros.ChROSClockHandler())   # /clock first, time-syncs the graph
ros_manager.RegisterHandler(chros.ChROSCameraHandler(cam.GetUpdateRate() / 4, cam, "~/output/camera/data/image"))
ros_manager.RegisterHandler(chros.ChROSLidarHandler(lidar, "~/output/lidar/data/pointcloud"))
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
ros_manager.Initialize()   # exactly once, after all handlers are registered

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Multi-sensor ground body published over ROS2")
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(-8, -8, 4), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 40, 40,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop === spin the body, pump sensors, publish to ROS, step physics
# Constant yaw so the attached sensors perceive motion (gyro reads ~0.1 on z).
ground_body.SetAngVelParent(chrono.ChVector3d(0, 0, spin_rate))


try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            time = sys.GetChTime()
            sens_manager.Update()                  # pump all sensors each physics step
            if not ros_manager.Update(time, time_step):   # publish; break on ROS shutdown
                break
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad sensor state
    import traceback
    traceback.print_exc()
    raise
