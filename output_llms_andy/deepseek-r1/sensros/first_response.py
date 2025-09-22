#!/usr/bin/env python3
import pychrono as chrono
import pychrono.sensor as sens
import pychrono.irrlicht as irr
import math

# ========================
# Initialize Chrono System
# ========================
chrono.SetChronoDataPath("")  # Add resource path if needed
system = chrono.ChSystemNSC()
system.SetGravity(chrono.ChVectorD(0, 0, -9.81))

# ========================
# Create Moving Ground Body
# ========================
ground = chrono.ChBodyEasyBox(10, 10, 0.2, 1000, True, False)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
ground.SetBodyFixed(False)  # Allow movement
system.Add(ground)

# Add visualization mesh
box_mesh = chrono.ChBoxShape()
box_mesh.GetBoxGeometry().Size = chrono.ChVectorD(5, 5, 0.1)
box_mesh.GetBoxGeometry().Pos = chrono.ChVectorD(0, 0, 0)
ground.AddVisualShape(box_mesh)

# Add motion (vertical oscillation)
motion_func = chrono.ChFunction_Sine(0, 0.5, 0.2)  # Amplitude=0.2m, Freq=0.5Hz
ground.SetPosDtFunction(motion_func)

# ========================
# Sensor Configuration
# ========================
sensor_manager = sens.ChSensorManager(system)
sensor_offset = chrono.ChVectorD(0, 0, 0.1)  # Slightly above ground

# Camera Sensor
cam = sens.ChCameraSensor(
    ground,                         # Parent body
    30,                             # Update rate (Hz)
    chrono.ChFrameD(sensor_offset), # Position
    1280,                           # Image width
    720,                            # Image height
    chrono.CH_C_PI / 3              # FOV (60 degrees)
)
cam.SetName("Camera")
cam.PushFilter(sens.ChFilterRGBA8Access())
sensor_manager.AddSensor(cam)

# LiDAR Sensor
lidar = sens.ChLidarSensor(
    ground,                         # Parent body
    15,                             # Update rate (Hz)
    chrono.ChFrameD(sensor_offset), # Position
    1000,                           # Samples per scan
    chrono.CH_C_PI / 3,             # Horizontal FOV
    chrono.CH_C_PI / 6,             # Vertical FOV
    0.1,                            # Min range (m)
    50.0                            # Max range (m)
)
lidar.SetName("Lidar")
lidar.PushFilter(sens.ChFilterDIAccess())
sensor_manager.AddSensor(lidar)

# IMU Sensors (Accel, Gyro, Magnetometer)
imu_offset = chrono.ChFrameD(sensor_offset)
imu_noise = sens.ChNoiseNormal(chrono.ChVectorD(0.001), chrono.ChVectorD(0.001))

# Accelerometer
accel = sens.ChAccelerometerSensor(
    ground, 10, imu_offset, imu_noise)
accel.SetName("Accelerometer")
sensor_manager.AddSensor(accel)

# Gyroscope
gyro = sens.ChGyroscopeSensor(
    ground, 10, imu_offset, imu_noise)
gyro.SetName("Gyroscope")
sensor_manager.AddSensor(gyro)

# Magnetometer
mag = sens.ChMagnetometerSensor(
    ground, 10, imu_offset, imu_noise)
mag.SetName("Magnetometer")
mag.SetMagneticField(chrono.ChVectorD(0, 0, -1e-5))  # Earth-like field
sensor_manager.AddSensor(mag)

# GPS Sensor
gps = sens.ChGPSSensor(
    ground, 10, imu_offset, chrono.ChVectorD(0,0,0), imu_noise)
gps.SetName("GPS")
sensor_manager.AddSensor(gps)

# ========================
# ROS Integration Setup
# ========================
ros_manager = sens.ChROSPublisher()
ros_manager.RegisterPublisher(cam, "/sensors/camera")
ros_manager.RegisterPublisher(lidar, "/sensors/lidar")
ros_manager.RegisterPublisher(accel, "/sensors/imu/accel")
ros_manager.RegisterPublisher(gyro, "/sensors/imu/gyro")
ros_manager.RegisterPublisher(mag, "/sensors/imu/mag")
ros_manager.RegisterPublisher(gps, "/sensors/gps")

# ========================
# Visualization Setup
# ========================
vis = irr.ChIrrApp(system, "Sensor Demo", irr.dimension2du(1280, 720))
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(5, 5, 5), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()
vis.AttachSystem(system)

# ========================
# Simulation Loop
# ========================
time_step = 0.001
end_time = 20.0
current_time = 0.0

while vis.Run() and current_time < end_time:
    # Update sensor positions and data
    sensor_manager.Update()
    
    # Publish data to ROS
    ros_manager.Process()
    
    # Advance simulation
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(time_step)
    
    # Maintain real-time execution
    chrono.ChRealtimeStepTimer.Spin(time_step)
    current_time += time_step