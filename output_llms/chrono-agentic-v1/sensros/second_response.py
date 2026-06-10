"""
PyChrono 9.0.x simulation: Sensor + ROS2 demo (sensros, turn 2).

System: ChSystemNSC, Z-up, no gravity.
Bodies: a fixed floor body (base_link) and a rotating sensor-carrier sphere.
Sensors: RGB camera, 2D lidar (planar scan), GPS, accelerometer, gyroscope,
         magnetometer.
ROS handlers: ChROSClockHandler, ChROSBodyHandler, ChROSTFHandler,
              ChROSCameraHandler,
              ChROSLidarHandler (2D LaserScan, ~/output/lidar2d/data/scan),
              ChROSGPSHandler, ChROSAccelerometerHandler, ChROSGyroscopeHandler,
              ChROSMagnetometerHandler, ChROSIMUHandler (fused).
Behavior: the carrier body spins at constant angular velocity so all sensors
          observe time-varying poses and publish data over ROS2.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens
import pychrono.ros as chros

# === Named constants ===
time_step   = 1e-3           # physics step [s]
sim_end     = 20.0           # simulation end time [s]
render_fps  = 50.0           # Irrlicht render rate [Hz]

# Sensor-carrier body
CARRIER_RADIUS  = 0.5        # sphere radius [m]
CARRIER_DENSITY = 1000.0     # density [kg/m³]
CARRIER_POS_Z   = 1.0        # spawn height [m]
CARRIER_ANGVEL  = chrono.ChVector3d(0.0, 0.0, 0.5)  # constant Z-spin [rad/s]

# Floor body (thin fixed slab)
FLOOR_X = 10.0
FLOOR_Y = 10.0
FLOOR_Z = 0.1
FLOOR_POS_Z = -0.5

# Camera sensor
CAM_RATE      = 30           # Hz (physical update rate)
CAM_W, CAM_H  = 1280, 720
CAM_FOV       = 1.408        # horizontal FOV [rad]

# 2D Lidar (planar: v_samples=1, vertical angles = 0)
LIDAR2D_RATE      = 10.0
LIDAR2D_H_SAMPLES = 800
LIDAR2D_V_SAMPLES = 1        # 2D — single horizontal layer
LIDAR2D_H_FOV     = 2 * chrono.CH_PI
LIDAR2D_MAX_VERT  = 0.0      # no vertical spread
LIDAR2D_MIN_VERT  = 0.0
LIDAR2D_RANGE     = 50.0

# GPS / IMU
GNSS_RATE = 10               # Hz
IMU_RATE  = 100              # Hz
GPS_REF   = chrono.ChVector3d(-89.400, 43.070, 260.0)  # reference lat/lon/alt

# Irrlicht camera viewpoint
VIS_EYE    = chrono.ChVector3d(-3.0, -3.0, 2.0)
VIS_TARGET = chrono.ChVector3d(0.0, 0.0, 0.0)

# precomputed once — physics steps per rendered frame
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once


# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, 0.0))  # no gravity
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Contact material ===
mat = chrono.ChContactMaterialNSC()
mat.SetFriction(0.5)
mat.SetRestitution(0.0)

# === Bodies ===
# Fixed floor — provides visual reference and collision ground
floor = chrono.ChBodyEasyBox(FLOOR_X, FLOOR_Y, FLOOR_Z,
                             1000.0, True, True, mat)
floor.SetFixed(True)
floor.SetPos(chrono.ChVector3d(0.0, 0.0, FLOOR_POS_Z))
floor.SetName("base_link")   # TF root frame
sys.AddBody(floor)

# Rotating sensor-carrier sphere — spun so sensors see motion
carrier = chrono.ChBodyEasySphere(CARRIER_RADIUS, CARRIER_DENSITY,
                                  True, False)  # visualize=True, collide=False
carrier.SetPos(chrono.ChVector3d(0.0, 0.0, CARRIER_POS_Z))
carrier.SetAngVelParent(CARRIER_ANGVEL)
carrier.SetName("carrier")
sys.AddBody(carrier)

# === Visualization (Irrlicht) ===
# Irrlicht call order: Initialize FIRST, then add all scene elements AFTER.
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("PyChrono SensROS — 2D Lidar + ROS")
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.Initialize()                                    # FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(VIS_EYE, VIS_TARGET)                  # AFTER Initialize
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 40, 40,
            chrono.ChCoordsysd(chrono.ChVector3d(0.0, 0.0, 0.0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Sensor manager & lighting ===
manager = sens.ChSensorManager(sys)
intensity = 1.0
manager.scene.AddPointLight(
    chrono.ChVector3f(2.0, 2.5, 100.0),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(-2.0, -2.5, 100.0),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)

# === Camera sensor ===
cam_offset_pose = chrono.ChFramed(
    chrono.ChVector3d(-1.5, 0.0, 0.5),
    chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0.0, 1.0, 0.0)),
)
cam = sens.ChCameraSensor(
    carrier, CAM_RATE, cam_offset_pose, CAM_W, CAM_H, CAM_FOV,
)
cam.SetName("Camera Sensor")
cam.SetLag(0)
cam.SetCollectionWindow(0)
cam.PushFilter(sens.ChFilterVisualize(CAM_W, CAM_H, "RGB Camera"))
cam.PushFilter(sens.ChFilterRGBA8Access())
cam.PushFilter(sens.ChFilterSave("cam/rgb/"))
manager.AddSensor(cam)

# === 2D Lidar sensor (planar scan: v_samples=1, vertical angles = 0) ===
lidar2d_offset = chrono.ChFramed(
    chrono.ChVector3d(-0.5, 0.0, 0.0),
    chrono.QuatFromAngleAxis(0.0, chrono.ChVector3d(0.0, 1.0, 0.0)),
)
lidar2d = sens.ChLidarSensor(
    carrier,
    LIDAR2D_RATE,
    lidar2d_offset,
    LIDAR2D_H_SAMPLES,
    LIDAR2D_V_SAMPLES,   # 1 → 2D planar
    LIDAR2D_H_FOV,
    LIDAR2D_MAX_VERT,    # 0.0 → no vertical spread
    LIDAR2D_MIN_VERT,    # 0.0 → no vertical spread
    LIDAR2D_RANGE,
    sens.LidarBeamShape_RECTANGULAR,
    2,
    0.003,
    0.003,
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar2d.SetName("2D Lidar Sensor")
lidar2d.SetLag(0)
lidar2d.SetCollectionWindow(1.0 / LIDAR2D_RATE)
# 2D lidar filters — named for visualization; ChFilterVisualizePointCloud omitted
# for the planar 1-ring scan (raw-depth visualize is height=1 which would crash)
lidar2d.PushFilter(sens.ChFilterVisualize(LIDAR2D_H_SAMPLES, 50, "Raw 2D Lidar Depth"))
lidar2d.PushFilter(sens.ChFilterDIAccess())
lidar2d.PushFilter(sens.ChFilterPCfromDepth())
lidar2d.PushFilter(sens.ChFilterXYZIAccess())
manager.AddSensor(lidar2d)

# === GPS sensor ===
gps_offset = chrono.ChFramed(
    chrono.ChVector3d(0.0, 0.0, 0.2),
    chrono.QuatFromAngleAxis(0.0, chrono.ChVector3d(0.0, 1.0, 0.0)),
)
gps = sens.ChGPSSensor(carrier, GNSS_RATE, gps_offset, GPS_REF, sens.ChNoiseNone())
gps.SetName("GPS Sensor")
gps.SetLag(0)
gps.SetCollectionWindow(0)
gps.PushFilter(sens.ChFilterGPSAccess())
manager.AddSensor(gps)

# === Accelerometer sensor ===
acc_offset = chrono.ChFramed(
    chrono.ChVector3d(0.0, 0.0, 0.0),
    chrono.QuatFromAngleAxis(0.0, chrono.ChVector3d(0.0, 1.0, 0.0)),
)
acc = sens.ChAccelerometerSensor(carrier, IMU_RATE, acc_offset, sens.ChNoiseNone())
acc.SetName("Accelerometer Sensor")
acc.SetLag(0)
acc.SetCollectionWindow(0)
acc.PushFilter(sens.ChFilterAccelAccess())
manager.AddSensor(acc)

# === Gyroscope sensor ===
gyro_offset = chrono.ChFramed(
    chrono.ChVector3d(0.0, 0.0, 0.0),
    chrono.QuatFromAngleAxis(0.0, chrono.ChVector3d(0.0, 1.0, 0.0)),
)
gyro = sens.ChGyroscopeSensor(carrier, IMU_RATE, gyro_offset, sens.ChNoiseNone())
gyro.SetName("Gyroscope Sensor")
gyro.SetLag(0)
gyro.SetCollectionWindow(0)
gyro.PushFilter(sens.ChFilterGyroAccess())
manager.AddSensor(gyro)

# === Magnetometer sensor ===
mag_offset = chrono.ChFramed(
    chrono.ChVector3d(0.0, 0.0, 0.0),
    chrono.QuatFromAngleAxis(0.0, chrono.ChVector3d(0.0, 1.0, 0.0)),
)
mag = sens.ChMagnetometerSensor(carrier, IMU_RATE, mag_offset, sens.ChNoiseNone(), GPS_REF)
mag.SetName("Magnetometer Sensor")
mag.SetLag(0)
mag.SetCollectionWindow(0)
mag.PushFilter(sens.ChFilterMagnetAccess())
manager.AddSensor(mag)

# === ROS manager — register all handlers, then Initialize once ===
ros_manager = chros.ChROSPythonManager()

# Clock handler FIRST — syncs ROS graph to simulation time
ros_manager.RegisterHandler(chros.ChROSClockHandler())

# Body handler — publishes carrier pose/twist
ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, carrier, "~/output/body/state"))

# TF handler — publishes floor→carrier transform
tf_handler = chros.ChROSTFHandler(30)
tf_handler.AddTransform(floor, floor.GetName(), carrier, carrier.GetName())
ros_manager.RegisterHandler(tf_handler)

# Camera image publisher
ros_manager.RegisterHandler(chros.ChROSCameraHandler(CAM_RATE, cam, "~/output/camera/image"))

# 2D Lidar → LaserScan (~/output/lidar2d/data/scan)
ros_manager.RegisterHandler(
    chros.ChROSLidarHandler(
        lidar2d,
        "~/output/lidar2d/data/scan",
        chros.ChROSLidarHandlerMessageType_LASER_SCAN,
    )
)

# GPS publisher
ros_manager.RegisterHandler(chros.ChROSGPSHandler(GNSS_RATE, gps, "~/output/gps/data"))

# IMU sub-handlers — standalone topics AND fed into the fused IMU handler
acc_handler  = chros.ChROSAccelerometerHandler(IMU_RATE, acc,  "~/output/accelerometer/data")
gyro_handler = chros.ChROSGyroscopeHandler(IMU_RATE, gyro, "~/output/gyroscope/data")
mag_handler  = chros.ChROSMagnetometerHandler(IMU_RATE, mag,  "~/output/magnetometer/data")
ros_manager.RegisterHandler(acc_handler)
ros_manager.RegisterHandler(gyro_handler)
ros_manager.RegisterHandler(mag_handler)

# Fused IMU handler (sensor_msgs/Imu)
imu_handler = chros.ChROSIMUHandler(IMU_RATE, "~/output/imu/data")
imu_handler.SetAccelerometerHandler(acc_handler)
imu_handler.SetGyroscopeHandler(gyro_handler)
imu_handler.SetMagnetometerHandler(mag_handler)
ros_manager.RegisterHandler(imu_handler)

# Initialize ONCE, after ALL handlers are registered
ros_manager.Initialize()


# === Main loop ===
frame = 0
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            manager.Update()             # pump all sensors — exactly once per physics step
            sys.DoStepDynamics(time_step)
            sim_time = sys.GetChTime()
            # cache: fetch carrier state once per step for CSV
            cpos = carrier.GetPos()           # cache: position, reused below
            cvel = carrier.GetAngVelParent()  # cache: angular velocity in parent frame
            if not ros_manager.Update(sim_time, time_step):
                break                        # ROS layer shut down
            if sim_time >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad physics state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
