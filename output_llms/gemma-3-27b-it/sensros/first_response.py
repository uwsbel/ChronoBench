import pychrono as chrono
import pychrono.sensor
import pychrono.postprocess
import pychrono.irrlicht
import math
import numpy as np

# Initialize the PyChrono system
chrono.SetChronoDataPath("../data/")  # Adjust path if needed

system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Set gravity

# Create a ground body
body_ground = chrono.ChBodyEasy()
body_ground.SetBodyFixed(True)
body_ground.SetPos(chrono.ChVectorD(0, -0.5, 0))
body_ground.SetShape(chrono.ChBoxShape())
body_ground.SetMass(1000)  # Large mass for stability
system.Add(body_ground)

# Add a mesh for visualization
mesh_ground = chrono.ChVisualShapeFile()
mesh_ground.SetFile("ground.obj")  # Replace with your ground mesh file
body_ground.AddVisualShape(mesh_ground)
mesh_ground.SetLocalPos(chrono.ChVectorD(0, 0, 0))
mesh_ground.SetLocalRot(chrono.Q_from_Ang3(0, 0, 0))

# Create a moving platform (example)
body_platform = chrono.ChBodyEasy()
body_platform.SetPos(chrono.ChVectorD(0, 1, 0))
body_platform.SetShape(chrono.ChBoxShape())
body_platform.SetMass(1)
system.Add(body_platform)

# Add a revolute joint to the ground
joint = chrono.ChLinkRevolute()
joint.Initialize(body_ground, body_platform, chrono.ChCoordsys(chrono.ChVectorD(0, 0, 0)))
system.Add(joint)

# Sensor Manager
sensor_manager = chrono.ChSensorManager()

# Create sensors
camera = chrono.ChSensorCamera()
camera.Initialize(body_platform, chrono.ChVectorD(0, 0, 0), chrono.Q_from_Ang3(0, 0, 0))
camera.SetResolution(640, 480)
sensor_manager.AddSensor(camera)

lidar = chrono.ChSensorLidar()
lidar.Initialize(body_platform, chrono.ChVectorD(0, 0, 0), chrono.Q_from_Ang3(0, 0, 0))
lidar.SetScanAngle(math.pi / 4)
lidar.SetRange(10)
sensor_manager.AddSensor(lidar)

gps = chrono.ChSensorGPS()
gps.Initialize(body_platform, chrono.ChVectorD(0, 0, 0), chrono.Q_from_Ang3(0, 0, 0))
sensor_manager.AddSensor(gps)

accelerometer = chrono.ChSensorAccelerometer()
accelerometer.Initialize(body_platform, chrono.ChVectorD(0, 0, 0), chrono.Q_from_Ang3(0, 0, 0))
sensor_manager.AddSensor(accelerometer)

gyroscope = chrono.ChSensorGyroscope()
gyroscope.Initialize(body_platform, chrono.ChVectorD(0, 0, 0), chrono.Q_from_Ang3(0, 0, 0))
sensor_manager.AddSensor(gyroscope)

magnetometer = chrono.ChSensorMagnetometer()
magnetometer.Initialize(body_platform, chrono.ChVectorD(0, 0, 0), chrono.Q_from_Ang3(0, 0, 0))
sensor_manager.AddSensor(magnetometer)

# ROS Manager (requires ROS installed and configured)
try:
    ros_manager = chrono.ChRosManager()
    ros_manager.Initialize()

    # Register sensors with ROS
    ros_manager.RegisterSensor(camera, "/chrono/camera")
    ros_manager.RegisterSensor(lidar, "/chrono/lidar")
    ros_manager.RegisterSensor(gps, "/chrono/gps")
    ros_manager.RegisterSensor(accelerometer, "/chrono/accelerometer")
    ros_manager.RegisterSensor(gyroscope, "/chrono/gyroscope")
    ros_manager.RegisterSensor(magnetometer, "/chrono/magnetometer")

except Exception as e:
    print(f"ROS initialization failed: {e}.  ROS integration skipped.")
    ros_manager = None


# Simulation loop
time_step = 0.01
time_end = 10

while system.GetChTime() < time_end:
    system.DoStepDynamics(time_step)

    # Update sensors
    sensor_manager.Update(system.GetChTime())

    # Process sensor data (example)
    if ros_manager:
        ros_manager.PublishSensorData()

    # Example: Move the platform in a circle
    angle = system.GetChTime() * 2
    body_platform.SetPos(chrono.ChVectorD(math.cos(angle), 1, math.sin(angle)))

# Print simulation end message
print("Simulation finished.")