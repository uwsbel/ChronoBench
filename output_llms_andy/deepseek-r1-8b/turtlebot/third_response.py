import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as turtlebot
from pychrono import irrlicht as chronoirr
from pychrono.sensor import *

# Create Chrono system
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# Create ground body as terrain plane that robot will drive on
ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.6))  # Adjusted ground position
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# Create Turtlebot Robot
init_pos = chrono.ChVector3d(0, 0.2, 0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
robot = turtlebot.TurtleBot(system, init_pos, init_rot)
robot.Initialize()

# Create sensor manager and lidar sensor
sensor_manager = sens.ChSensorManager(system)
lidar = sens.ChLidarSensor(system)
lidar.SetRange(10)
lidar.SetResolution(0.1)
lidar.SetFilterMode(sens.ChLidarSensor.FilterMode_GRID)
lidar.SetGroundFilter(True)
lidar.SetReflectionFilter(True)
lidar.SetMaxHeight(5)
lidar.SetMinHeight(0.5)
sensor_manager.AddSensor(lidar)

# Create function to add random boxes
def AddRandomBoxes(count):
    for _ in range(count):
        box = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 1000, True, True, ground_mat)
        box.SetPos(chrono.ChVector3d(
            np.random.uniform(-10, 10),
            np.random.uniform(-10, 10),
            -1  # Place boxes on ground (z=-1)
        ))
        box.SetRot(chrono.ChQuaterniond(
            np.random.uniform(-1, 1),
            np.random.uniform(-1, 1),
            np.random.uniform(-1, 1),
            np.random.uniform(-1, 1)
        ))
        system.Add(box)

# Add 5 random boxes
AddRandomBoxes(5)

# Define motion control function for Turtlebot
def move(mode):
    if mode == 'straight':
        robot.SetMotorSpeed(1, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(1, RIGHT_DRIVE_WHEEL)
    elif mode == 'left':
        robot.SetMotorSpeed(-math.pi, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(1, RIGHT_DRIVE_WHEEL)
    elif mode == 'right':
        robot.SetMotorSpeed(1, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(-math.pi, RIGHT_DRIVE_WHEEL)

# Simulation loop
time = 0
while vis.Run():
    # Update sensor manager
    sensor_manager.Update()

    # Control robot movement
    move('straight')

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance simulation by one time step
    system.DoStepDynamics(time_step)
    time += time_step