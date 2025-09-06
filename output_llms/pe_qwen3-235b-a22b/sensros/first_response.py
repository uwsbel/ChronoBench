import os
import math
import numpy as np
import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens
import pychrono.ros as chros

# Set Chrono data path
chrono.SetChronoDataPath(os.environ.get('CHRONO_DATA_DIR', ''))

# Initialize the Chrono system
sys = chrono.ChSystemNSC()

# Create the moving ground body
ground = chrono.ChBody()
ground.SetPos(chrono.ChVector3d(0, 0, 0))
ground.SetMass(1000)
ground.SetInertiaXX(chrono.ChVector3d(1e3, 1e3, 1e3))
ground_shape = chrono.ChVisualShapeBox(10, 1, 10)
ground_shape.SetColor(chrono.ChColor(0.3, 0.3, 0.3))
ground.AddVisualShape(ground_shape)
sys.Add(ground)

# Set up prismatic joint and motor for ground movement
prismatic = chrono.ChLinkLockPrismatic()
prismatic.Initialize(ground, chrono.ChBody(), chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
sys.Add(prismatic)

motor = chrono.ChLinkMotorLinearSpeed()
motor.Initialize(ground, chrono.ChBody(), chrono.ChCoordsysD())
func = chrono.ChFunction_Sine(0, 1, 1.0)  # Amplitude 1m, frequency 1Hz
motor.SetFunction(func)
sys.Add(motor)

# Visualization setup
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Sensors on Moving Ground')
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 5, 10))
vis.AddTypicalLights()

# Sensor manager setup
manager = sens.ChSensorManager(sys)
manager.SetTimeStep(1e-3)  # Sensor update rate

# Add sensors to the manager
# Camera
camera = sens.ChCameraSensor(
    ground,
    60,  # Update rate (Hz)
    chrono.ChFrameD(chrono.ChVector3d(0, 1, 0), chrono.ChQuaternionD(1, 0, 0, 0)),
    640, 480,  # Resolution
    0.01  # Lag
)
camera.SetName("camera")
manager.AddSensor(camera)

# Lidar
lidar = sens.ChLidarSensor(
    ground,
    10,  # Update rate (Hz)
    chrono.ChFrameD(chrono.ChVector3d(0, 1, 0)),
    100, 360,  # Horizontal samples and FOV
    1.0,  # Max distance
    0.1,  # Vertical samples
    0.01  # Lag
)
lidar.SetName("lidar")
manager.AddSensor(lidar)

# GPS
gps = sens.ChGPSSensor(
    ground,
    1,  # Update rate (Hz)
    chrono.ChFrameD(),
    chrono.ChVector3d(0, 0, 0),  # Reference position
    1.0  # Noise
)
gps.SetName("gps")
manager.AddSensor(gps)

# Accelerometer
accelerometer = sens.ChAccelerometerSensor(
    ground,
    100,  # Update rate (Hz)
    chrono.ChFrameD()
)
accelerometer.SetName("accelerometer")
manager.AddSensor(accelerometer)

# Gyroscope
gyro = sens.ChGyroscopeSensor(
    ground,
    100,  # Update rate (Hz)
    chrono.ChFrameD()
)
gyro.SetName("gyro")
manager.AddSensor(gyro)

# Magnetometer
magnetometer = sens.ChMagnetometerSensor(
    ground,
    100,  # Update rate (Hz)
    chrono.ChFrameD()
)
magnetometer.SetName("magnetometer")
manager.AddSensor(magnetometer)

# ROS manager setup
ros_manager = chros.ChROSPublisherManager()
ros_manager.RegisterSensor(camera, "/camera/image")
ros_manager.RegisterSensor(lidar, "/lidar/scan")
ros_manager.RegisterSensor(gps, "/gps/data")
ros_manager.RegisterSensor(accelerometer, "/imu/accel")
ros_manager.RegisterSensor(gyro, "/imu/gyro")
ros_manager.RegisterSensor(magnetometer, "/imu/mag")

# Simulation loop
step_size = 1e-3
time = 0
realtime_timer = chrono.ChRealtimeStepTimer()

while vis.Run():
    # Update sensors
    manager.Update()
    
    # Step the simulation
    sys.DoStepDynamics(step_size)
    time += step_size
    
    # Publish ROS data
    ros_manager.Update(time)
    
    # Render scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    # Maintain real-time execution
    realtime_timer.Spin(step_size)