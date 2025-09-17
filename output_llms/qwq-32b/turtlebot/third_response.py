import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as turtlebot
from pychrono import irrlicht as chronoirr
import pychrono.sensor as sens

# Constants for wheel IDs
LEFT_DRIVE_WHEEL = 0
RIGHT_DRIVE_WHEEL = 1

# Create Chrono system
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVectorD(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# Create ground body
ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVectorD(0, 0, -0.6))  # Adjusted position
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# Create Turtlebot Robot
init_pos = chrono.ChVectorD(0, 0.2, 0)
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)
robot = turtlebot.TurtleBot(system, init_pos, init_rot)
robot.Initialize()

# Add sensor manager and lidar
sensor_mgr = sens.ChSensorManager(system)
system.AddSensorManager(sensor_mgr)

# Configure lidar
lidar = sens.ChLidar()
lidar.SetName("lidar")
lidar.SetParentFrame(robot.GetChassisBody().GetFrame_REF_to_abs())  # Attach to robot's chassis
lidar.SetPosition(chrono.ChVectorD(0, 0, 0.2))  # Position on the robot
lidar_params = sens.ChLidarParameters()
lidar_params.m_fov_horizontal = 270  # degrees
lidar_params.m_fov_vertical = 30
lidar_params.m_range = 10.0  # meters
lidar_params.m_resolution_horiz = 0.5  # degrees per sample
lidar_params.m_resolution_vert = 1.0
lidar_params.m_noise = 0.01
lidar.SetParameters(lidar_params)

# Add filters to lidar
range_filter = sens.ChFilterRange()
range_filter.SetMinDistance(0.1)
range_filter.SetMaxDistance(10.0)
lidar.AddFilter(range_filter)

point_cloud_filter = sens.ChFilterPointCloud()
lidar.AddFilter(point_cloud_filter)

sensor_mgr.AddSensor(lidar)

# Add randomly placed boxes
for _ in range(5):
    x = np.random.uniform(-5, 5)
    y = np.random.uniform(-5, 5)
    z = 0.15  # Position above ground
    box = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 1, True, True, ground_mat)
    box.SetPos(chrono.ChVectorD(x, y, z))
    box.SetBodyFixed(False)
    system.Add(box)

# Create visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Turtlebot Robot - Rigid terrain with Sensors')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 1.5, 0.2), chrono.ChVectorD(0, 0, 0.2))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(1.5, -2.5, 5.5), chrono.ChVectorD(0, 0, 0.5), 3, 4, 10, 40, 512)

def move(robot, mode):
    if mode == 'straight':
        robot.SetMotorSpeed(math.pi, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(math.pi, RIGHT_DRIVE_WHEEL)
    elif mode == 'left':
        robot.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(math.pi, RIGHT_DRIVE_WHEEL)
    elif mode == 'right':
        robot.SetMotorSpeed(math.pi, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL)

# Simulation loop
time_step = 2e-3
time = 0
while vis.Run():
    # Control the robot using move function
    move(robot, 'straight')

    # Advance simulation
    system.DoStepDynamics(time_step)

    # Update sensors
    sensor_mgr.Update()

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Increment time
    time += time_step

import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as turtlebot
from pychrono import irrlicht as chronoirr
import pychrono.sensor as sens

# Constants for wheel IDs
LEFT_DRIVE_WHEEL = 0
RIGHT_DRIVE_WHEEL = 1

# Create Chrono system
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVectorD(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# Create ground body
ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVectorD(0, 0, -0.6))  # Adjusted ground position
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# Create Turtlebot Robot
init_pos = chrono.ChVectorD(0, 0.2, 0)
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)
robot = turtlebot.TurtleBot(system, init_pos, init_rot)
robot.Initialize()

# Add sensor manager and lidar
sensor_mgr = sens.ChSensorManager(system)
system.AddSensorManager(sensor_mgr)

# Configure lidar sensor
lidar = sens.ChLidar()
lidar.SetName("lidar")
lidar.SetParentFrame(robot.GetBody().GetFrame_REF_to_abs())  # Attach to robot's chassis
lidar.SetPosition(chrono.ChVectorD(0, 0, 0.2))  # Position on the robot

lidar_params = sens.ChLidarParameters()
lidar_params.m_fov_horizontal = 270  # degrees
lidar_params.m_fov_vertical = 30
lidar_params.m_range = 10.0  # meters
lidar_params.m_resolution_horiz = 0.5  # degrees per sample
lidar_params.m_resolution_vert = 1.0
lidar_params.m_noise = 0.01
lidar.SetParameters(lidar_params)

# Add filters to lidar
range_filter = sens.ChFilterRange()
range_filter.SetMinDistance(0.1)
range_filter.SetMaxDistance(10.0)
lidar.AddFilter(range_filter)

point_cloud_filter = sens.ChFilterPointCloud()
lidar.AddFilter(point_cloud_filter)

sensor_mgr.AddSensor(lidar)

# Add randomly placed boxes
for _ in range(5):
    x = np.random.uniform(-5, 5)
    y = np.random.uniform(-5, 5)
    z = 0.15  # Position above ground
    box = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 1, True, True, ground_mat)
    box.SetPos(chrono.ChVectorD(x, y, z))
    box.SetBodyFixed(False)
    system.Add(box)

# Create visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Turtlebot Robot - Rigid terrain with Sensors')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 1.5, 0.2), chrono.ChVectorD(0, 0, 0.2))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(1.5, -2.5, 5.5), chrono.ChVectorD(0, 0, 0.5), 3, 4, 10, 40, 512)

def move(robot, mode):
    if mode == 'straight':
        robot.SetMotorSpeed(math.pi, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(math.pi, RIGHT_DRIVE_WHEEL)
    elif mode == 'left':
        robot.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(math.pi, RIGHT_DRIVE_WHEEL)
    elif mode == 'right':
        robot.SetMotorSpeed(math.pi, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL)

# Simulation loop
time_step = 2e-3
time = 0
while vis.Run():
    # Control the robot using move function
    move(robot, 'straight')

    # Advance simulation
    system.DoStepDynamics(time_step)

    # Update sensors
    sensor_mgr.Update()

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Increment time
    time += time_step