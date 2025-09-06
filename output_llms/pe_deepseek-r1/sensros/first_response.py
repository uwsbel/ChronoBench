import pychrono as chrono
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr
import numpy as np
import os
import time

# ========================
# 1. SYSTEM INITIALIZATION
# ========================
system = chrono.ChSystemSMC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

# Set Chrono data path
chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), 'data/'))

# =====================
# 2. PHYSICAL SETUP
# =====================
# Create ground body with oscillatory motion
ground = chrono.ChBodyEasyBox(10, 10, 0.2, 1000, True, True)
ground.SetName("MovingGround")
ground.SetPos(chrono.ChVector3d(0, 0, -1))
system.Add(ground)

# Motion function for harmonic oscillation
motion_func = chrono.ChFunctionSine(0.5, 0.5)  # Amplitude=0.5m, Frequency=0.5Hz
motion = chrono.ChLinkMotorLinPosition()
motion.Initialize(ground, system.GetChBody(0), 
                 chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
motion.SetPositionFunction(motion_func)
system.Add(motion)

# Add visual mesh object
mesh = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh(chrono.GetChronoDataFile("models/cube.obj"))
mesh_asset = chrono.ChVisualShapeTriangleMesh()
mesh_asset.SetMesh(mesh)
mesh_asset.SetColor(chrono.ChColor(0.6, 0.2, 0.2))
ground.AddVisualShape(mesh_asset)

# =====================
# 3. SENSOR SETUP
# =====================
manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(chrono.ChVector3d(2, 2, 2), chrono.ChColor(1, 1, 1), 100)

# Camera sensor
cam_offset = chrono.ChFramed(chrono.ChVector3d(-2, 0, 1), 
                            chrono.QuatFromAngleY(np.pi/2))
camera = sens.ChCameraSensor(
    ground,             # Parent body
    30,                 # Update rate [Hz]
    cam_offset,         # Offset pose
    1280,               # Image width
    720,                # Image height
    np.deg2rad(75)      # FOV
)
camera.SetName("RGB_Camera")
camera.PushFilter(sens.ChFilterRGBA8())
manager.AddSensor(camera)

# Lidar sensor
lidar_offset = chrono.ChFramed(chrono.ChVector3d(0, 0, 0.5))
lidar = sens.ChLidarSensor(
    ground,             # Parent body
    10,                 # Update rate [Hz]
    lidar_offset,       # Offset pose
    1000,               # Samples per channel
    50,                 # Number of channels
    np.deg2rad(30),     # Horizontal FOV
    np.deg2rad(15),     # Vertical FOV
    0.1,                # Min range [m]
    100.0               # Max range [m]
)
lidar.SetName("Scanning_Lidar")
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
manager.AddSensor(lidar)

# IMU sensors (accelerometer, gyroscope, magnetometer)
imu_offset = chrono.ChFramed(chrono.ChVector3d(0.2, 0, 0.3))
accel = sens.ChAccelerometerSensor(ground, 100, imu_offset)
gyro = sens.ChGyroscopeSensor(ground, 100, imu_offset)
mag = sens.ChMagnetometerSensor(ground, 100, imu_offset)

accel.SetName("IMU_Accelerometer")
gyro.SetName("IMU_Gyroscope")
mag.SetName("IMU_Magnetometer")

# Add noise models
accel.PushFilter(sens.ChFilterAccelNoiseNormal(0.001, 0.01))
gyro.PushFilter(sens.ChFilterGyroNoiseNormal(0.001, 0.001))
mag.PushFilter(sens.ChFilterMagnetNoiseNormal(0.02, 0.02))

manager.AddSensor(accel)
manager.AddSensor(gyro)
manager.AddSensor(mag)

# GPS sensor
gps_offset = chrono.ChFramed(chrono.ChVector3d(0, 0, 0.5))
gps = sens.ChGPSSensor(
    ground,             # Parent body
    10,                 # Update rate [Hz]
    gps_offset,         # Offset pose
    chrono.ChVector3d(0, 0, 0),  # Reference location
    True                # Enable noise
)
gps.SetName("Positioning_GPS")
manager.AddSensor(gps)

# =====================
# 4. ROS INTEGRATION
# =====================
ros_manager = sens.ChROSParallelManager()
ros_manager.RegisterSensor(camera, "/pychrono/camera")
ros_manager.RegisterSensor(lidar, "/pychrono/lidar")
ros_manager.RegisterSensor(accel, "/pychrono/imu/accel")
ros_manager.RegisterSensor(gyro, "/pychrono/imu/gyro")
ros_manager.RegisterSensor(mag, "/pychrono/imu/mag")
ros_manager.RegisterSensor(gps, "/pychrono/gps")

# =====================
# 5. VISUALIZATION
# =====================
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("PyChrono Sensor Demo")
vis.Initialize()
vis.AddCamera(chrono.ChVector3d(3, 3, 3))
vis.AddTypicalLights()
vis.AddSkyBox()

# =====================
# 6. SIMULATION LOOP
# =====================
time_step = 0.001
real_time_factor = 1.0
last_time = time.time()

while vis.Run():
    # Maintain real-time execution
    current_time = time.time()
    elapsed = current_time - last_time
    last_time = current_time
    
    # Update sensors and ROS
    manager.Update()
    ros_manager.Update()
    
    # Advance simulation
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(time_step)
    
    # Real-time pacing
    computation_time = time.time() - current_time
    sleep_time = time_step/real_time_factor - computation_time
    if sleep_time > 0:
        time.sleep(sleep_time)