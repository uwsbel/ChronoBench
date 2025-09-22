import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sensors
import pychrono.ros as ros
import numpy as np
import time

# Initialize Chrono system
my_system = chrono.ChSystemNSC()
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create ground body
ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
ground.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
my_system.Add(ground)

# Add visualization mesh to ground
ground_vis = chrono.ChTriangleMeshShape()
ground_vis.SetMesh(chrono.GetChObjFileMesh("path/to/ground.obj"))  # Replace with actual path
ground.AddAsset(ground_vis)

# Create a moving platform (child of ground)
platform = chrono.ChBody()
platform.SetPos(chrono.ChVectorD(0, 1, 0))
platform.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
my_system.Add(platform)

# Link platform to ground with a prescribed motion
link = chrono.ChLinkLockPrismatic()
link.Initialize(ground, platform, chrono.ChCoordinator(0, 1, 0))
my_system.Add(link)

# Function to update platform position (sine wave motion)
def update_platform(time):
    platform_pos = chrono.ChVectorD(0, 1 + 0.5 * np.sin(time), 0)
    link.SetPrismaticDisplacement(platform_pos.y - 1)
    platform.SetPos(platform_pos)

# Create sensor manager
sensor_manager = sensors.ChSensorManager(my_system)

# Create ROS manager
ros_manager = ros.ChROSManager()
ros_manager.Initialize("chrono_simulation_node")

# Create and add sensors
# 1. Camera sensor
camera = sensors.ChCameraSensor(
    platform,                     # parent body
    10,                           # update rate in Hz
    chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),  # position and orientation
    640, 480,                     # resolution
    1.0472,                       # fov (60 degrees in radians)
    0.1, 100                      # near and far clipping planes
)
sensor_manager.AddSensor(camera)

# 2. LiDAR sensor
lidar = sensors.ChLidarSensor(
    platform,
    10,
    chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
    360,                          # horizontal resolution
    180,                          # vertical resolution
    0.1, 100,                     # min and max range
    0.01                          # angular resolution
)
sensor_manager.AddSensor(lidar)

# 3. GPS sensor
gps = sensors.ChGPSSensor(
    platform,
    10,
    chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT)
)
sensor_manager.AddSensor(gps)

# 4. IMU (Accelerometer + Gyroscope + Magnetometer)
imu = sensors.ChIMUSensor(
    platform,
    100,                          # higher update rate for IMU
    chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT)
)
sensor_manager.AddSensor(imu)

# Register sensors with ROS manager
ros_manager.RegisterSensor(camera, "/camera/image_raw", "sensor_msgs/Image")
ros_manager.RegisterSensor(lidar, "/lidar/points", "sensor_msgs/PointCloud2")
ros_manager.RegisterSensor(gps, "/gps/fix", "sensor_msgs/NavSatFix")
ros_manager.RegisterSensor(imu, "/imu/data", "sensor_msgs/Imu")

# Create visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(my_system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle("PyChrono Sensor Simulation")
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(5, 5, 5))
vis.AddTypicalLights()

# Simulation parameters
time_step = 0.01
max_time = 10.0
current_time = 0

# Simulation loop
while vis.Run() and current_time < max_time:
    # Update platform position
    update_platform(current_time)

    # Update sensors
    sensor_manager.Update()

    # Publish sensor data to ROS
    ros_manager.Publish()

    # Advance simulation
    my_system.DoStepDynamics(time_step)
    current_time += time_step

    # Maintain real-time execution
    time.sleep(time_step)

    # Print simulation time
    print(f"Simulation time: {current_time:.2f} s")

# Cleanup
ros_manager.Shutdown()