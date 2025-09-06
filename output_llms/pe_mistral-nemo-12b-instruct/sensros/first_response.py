import os
import math
import numpy as np
import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.sensors as sens
import rospy
from sensor_msgs.msg import Image, PointCloud2, NavSatFix, Imu, MagneticField

# Set Chrono data path
chrono.SetChronoDataPath(os.path.join(os.getcwd(), '..', 'data'))

# Initialize ROS node
rospy.init_node('pychrono_sensor_publisher', anonymous=True)

# Initialize Chrono system and visualization
sys = chrono.ChSystemSMC()
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('PyChrono Sensors Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()

# Set up ground body and mesh
ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground_shape = chrono.ChVisualShapeBox(chrono.ChVector3d(10, 10, 0.1))
ground.AddVisualShape(ground_shape)
sys.Add(ground)

# Initialize sensor manager and ROS manager
sensor_manager = sens.ChSensorManager(sys)
ros_manager = sens.ChRosManager(sys)

# Add sensors to sensor manager and register with ROS manager
camera = sens.ChCamera()
camera.SetChassisBody(ground)
camera.SetFieldOfView(90)
sensor_manager.Add(camera)
ros_manager.RegisterSensor('camera', '/camera/image_raw', Image)

lidar = sens.ChLidar()
lidar.SetChassisBody(ground)
lidar.SetRange(50)
sensor_manager.Add(lidar)
ros_manager.RegisterSensor('lidar', '/lidar/points', PointCloud2)

gps = sens.ChGPS()
gps.SetChassisBody(ground)
sensor_manager.Add(gps)
ros_manager.RegisterSensor('gps', '/gps/fix', NavSatFix)

accelerometer = sens.ChAccelerometer()
accelerometer.SetChassisBody(ground)
sensor_manager.Add(accelerometer)
ros_manager.RegisterSensor('accelerometer', '/imu/data', Imu)

gyroscope = sens.ChGyroscope()
gyroscope.SetChassisBody(ground)
sensor_manager.Add(gyroscope)
ros_manager.RegisterSensor('gyroscope', '/imu/data', Imu)

magnetometer = sens.ChMagnetometer()
magnetometer.SetChassisBody(ground)
sensor_manager.Add(magnetometer)
ros_manager.RegisterSensor('magnetometer', '/magnetometer/data', MagneticField)

# Simulation loop
while not rospy.is_shutdown():
    # Update sensors and publish data
    sensor_manager.Update()
    ros_manager.PublishAll()

    # Advance simulation
    sys.DoStepDynamics(0.01)

    # Render frame
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Wait for next frame
    vis.WaitForFrameEnd()

# Clean up ROS node
rospy.signal_shutdown('Simulation ended')