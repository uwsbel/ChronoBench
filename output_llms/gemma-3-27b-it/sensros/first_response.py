import pychrono as chrono
import pychrono.sensor as sensor
import pychrono.postprocess as postprocess
import pychrono.irrlicht as chronoirr
import numpy as np

# Initialize the PyChrono system
chrono.SetChronoDataPath("./")  # Set the path to the Chrono data directory

system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Set gravity

# Create a ground body
body_ground = chrono.ChBodyEasy()
body_ground.SetBodyFixed(True)
body_ground.SetPos(chrono.ChVectorD(0, -1, 0))
system.Add(body_ground)

# Add a box shape to the ground for visualization
box_shape = chrono.ChBoxShape()
box_shape.GetBoxGeometry().SetMinMax(chrono.ChVectorD(-10, -1, -10), chrono.ChVectorD(10, 0, 10))
body_ground.AddAsset(box_shape)

# Create a mesh object for visualization (example: a simple sphere)
mesh_sphere = chrono.ChSphereShape()
mesh_sphere.GetSphereGeometry().SetRadius(0.5)
body_sphere = chrono.ChBodyEasy()
body_sphere.SetPos(chrono.ChVectorD(0, 2, 0))
body_sphere.AddAsset(mesh_sphere)
body_sphere.SetMass(1.0)
system.Add(body_sphere)

# Create a sensor manager
sensor_manager = sensor.ChSensorManager(system)

# Create sensors
camera = sensor.ChCameraSensor()
camera.Setup(body_sphere, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 1), 60, 0.1, 100)
sensor_manager.AddSensor(camera)

lidar = sensor.ChLidarSensor()
lidar.Setup(body_sphere, chrono.ChVectorD(0, 0, 0), 360, 0.1, 10)
sensor_manager.AddSensor(lidar)

gps = sensor.ChGPSSensor()
gps.Setup(body_sphere)
sensor_manager.AddSensor(gps)

accelerometer = sensor.ChAccelerometerSensor()
accelerometer.Setup(body_sphere, chrono.ChVectorD(0, 0, 0))
sensor_manager.AddSensor(accelerometer)

gyroscope = sensor.ChGyroscopeSensor()
gyroscope.Setup(body_sphere, chrono.ChVectorD(0, 0, 0))
sensor_manager.AddSensor(gyroscope)

magnetometer = sensor.ChMagnetometerSensor()
magnetometer.Setup(body_sphere, chrono.ChVectorD(0, 0, 0))
sensor_manager.AddSensor(magnetometer)


# Create a ROS manager (requires ROS installation and setup)
try:
    ros_manager = sensor.ChRosSensorManager()
    ros_manager.Setup("chrono", "sensor_data")  # Namespace and topic prefix

    # Register sensors with ROS
    ros_manager.AddSensor(camera, "camera_data")
    ros_manager.AddSensor(lidar, "lidar_data")
    ros_manager.AddSensor(gps, "gps_data")
    ros_manager.AddSensor(accelerometer, "accelerometer_data")
    ros_manager.AddSensor(gyroscope, "gyroscope_data")
    ros_manager.AddSensor(magnetometer, "magnetometer_data")

except Exception as e:
    print(f"ROS integration failed: {e}.  Ensure ROS is installed and configured correctly.")
    ros_manager = None # disable ros if not available



# Simulation loop
time_step = 0.005
time_end = 10

while system.GetChTime() < time_end:
    # Update sensors
    sensor_manager.Update()

    # Advance the simulation
    system.DoStepDynamics(time_step)

    # Optional: Visualize the simulation
    # if ros_manager is None:
    #     chrono.ChIrrApp.Instance().Update()
    #     chrono.ChIrrApp.Instance().Draw()

    #if ros_manager:
    #    print("Simulation running, sensor data published to ROS topics.")
    #else:
    #    print("Simulation running, sensor data not published to ROS.")

# Print simulation end message
print("Simulation finished.")