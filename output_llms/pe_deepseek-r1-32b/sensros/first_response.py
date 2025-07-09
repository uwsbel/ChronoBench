import math
import numpy as np
import pychrono as chrono
import pychrono.robot as rob
import pychrono.sensor as sensor
import pychrono.ros as ros

# Initialize ROS manager
ros_manager = ros.ChROSManager()
ros_manager.Initialize()

# Create Chrono system
system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create ground body
ground = chrono.ChBody()
ground.SetMass(0)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
ground.SetBodyFixed(False)  # Allow ground to move
system.Add(ground)

# Add mesh for visualization
mesh = chrono.ChMesh()
mesh.SetName("ground_mesh")
mesh.SetMaterialSurface(chrono.ChVisualMaterial())
mesh.SetScale(chrono.ChVectorD(1, 1, 1))
ground.AddVisualShape(mesh)

# Create sensor manager
sensor_manager = sensor.ChSensorManager(system)

# Create and register sensors
# Camera
camera = sensor.ChCameraSensor(ground, chrono.ChVectorD(0, 1, 3), chrono.ChVectorD(0, 0, 0))
ros_manager.RegisterSensor("camera", camera)

# Lidar
lidar = sensor.ChLidarSensor(ground, 360, 10, 0.1, 10)
ros_manager.RegisterSensor("lidar", lidar)

# GPS
gps = sensor.ChGPSSensor(ground)
ros_manager.RegisterSensor("gps", gps)

# Accelerometer
accelerometer = sensor.ChAccelerometerSensor(ground)
ros_manager.RegisterSensor("accelerometer", accelerometer)

# Gyroscope
gyroscope = sensor.ChGyroscopeSensor(ground)
ros_manager.RegisterSensor("gyroscope", gyroscope)

# Magnetometer
magnetometer = sensor.ChMagnetometerSensor(ground)
ros_manager.RegisterSensor("magnetometer", magnetometer)

# Add all sensors to sensor manager
sensor_manager.AddSensor(camera)
sensor_manager.AddSensor(lidar)
sensor_manager.AddSensor(gps)
sensor_manager.AddSensor(accelerometer)
sensor_manager.AddSensor(gyroscope)
sensor_manager.AddSensor(magnetometer)

# Initialize ROS publishers
ros_manager.InitializePublishers()

# Visualization setup
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Sensor Simulation")
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 10, -10))
vis.AddLight(chrono.ChVectorD(0, 10, 0), chrono.ChColor(1, 1, 1))

# Simulation parameters
time_step = 0.001
real_time_factor = 1.0
simulation_time = 0

# Main simulation loop
while vis.Run():
    # Update sensors
    sensor_manager.Update()
    
    # Publish sensor data
    ros_manager.PublishSensorData()
    
    # Advance simulation
    system.DoStepDynamics(time_step)
    
    # Update visualization
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    # Control ground movement (example: move with keyboard)
    key = vis.GetKeyboard()
    if key == ord('w'):
        ground.SetPos(ground.GetPos() + chrono.ChVectorD(0, 0, 0.1))
    elif key == ord('s'):
        ground.SetPos(ground.GetPos() + chrono.ChVectorD(0, 0, -0.1))
    elif key == ord('a'):
        ground.SetPos(ground.GetPos() + chrono.ChVectorD(-0.1, 0, 0))
    elif key == ord('d'):
        ground.SetPos(ground.GetPos() + chrono.ChVectorD(0.1, 0, 0))
    
    # Check for exit
    if vis.GetKeyboard() == ord('q'):
        break
    
    # Update simulation time
    simulation_time += time_step

# Cleanup
ros_manager.Shutdown()
vis.Close()