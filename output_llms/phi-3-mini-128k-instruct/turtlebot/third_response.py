import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as turtlebot
from pychrono import irrlicht as chronoirr
import pychrono.sensor as sens

# Create Chrono system
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  # Set gravity in negative Z direction
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# Create ground body as terrain plane that robot will drive on
ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.6))  # Adjusted ground position
ground.SetFixed(True)  # Fix the ground in place
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# Create Turtlebot Robot
init_pos = chrono.ChVector3d(0, 0.2, 0)  # Initial position of the robot
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)  # Initial orientation of the robot
robot = turtlebot.TurtleBot(system, init_pos, init_rot)  # Create Turtlebot instance
robot.Initialize()  # Initialize the robot

# Create sensor manager and configure lidar sensor
sensor_manager = sens.ChSensorManager()
lidar_sensor = sens.ChLidarSensor(sensor_manager, system)
lidar_sensor.SetLidarType(sens.ChLidarSensor.Type_3D_LIDAR)
lidar_sensor.SetLidarRange(5.0)
lidar_sensor.SetLidarFieldOfView(180)
lidar_sensor.SetLidarRangeResolution(0.1)
lidar_sensor.SetLidarRotationFrequency(10)
lidar_sensor.SetLidarRangeAverageFilter(5)
lidar_sensor.SetLidarPointCloudFilter(10)
lidar_sensor.SetLidarPointCloudFilterAverage(5)
lidar_sensor.SetLidarPointCloudFilterMinimum(1)
lidar_sensor.SetLidarPointCloudFilterMaximum(100)
sensor_manager.AddSensor(lidar_sensor)

# Create 5 randomly placed boxes for interaction
for i in range(5):
    box_pos = chrono.ChVector3d(np.random.uniform(-5, 5), np.random.uniform(-5, 5), np.random.uniform(-0.5, 0.5))
    box_size = chrono.ChVector3d(1, 1, 0.5)
    box = chrono.ChBodyEasyBox(box_size.x, box_size.y, box_size.z, 1000, True, True, chrono.ChContactMaterialNSC())
    box.SetPos(box_d,
,
,
,
0,   0,    #   
   
,   
,