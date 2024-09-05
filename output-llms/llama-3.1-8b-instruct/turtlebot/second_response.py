"""
Import necessary modules
"""
import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as turtlebot
from pychrono import irrlicht as chronoirr

"""
Create Chrono system
"""
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  # Set gravity in negative Z direction
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

"""
Create ground body as terrain plane that robot will drive on
"""
ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.6))  # Adjusted ground position
ground.SetFixed(True)  # Fix the ground in place
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

"""
Create Turtlebot Robot
"""
init_pos = chrono.ChVector3d(0, 0.2, 0)  # Initial position of the robot
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)  # Initial orientation of the robot
robot = turtlebot.TurtleBot(system, init_pos, init_rot)  # Create Turtlebot instance
robot.Initialize()  # Initialize the robot

"""
Create run-time visualization
"""
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Turtlebot Robot - Rigid terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1.5, 0.2), chrono.ChVector3d(0, 0, 0.2))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5), 3, 4, 10, 40, 512)
vis.EnableShadows()  # Removed comment to enable shadows

"""
Define WheelID enum values for motor control
"""
class WheelID:
    LEFT_DRIVE_WHEEL = 0
    RIGHT_DRIVE_WHEEL = 1

"""
Function to control the robot's movement
"""
def move(mode):
    if mode == 'straight':
        robot.SetMotorSpeed(0, WheelID.LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0, WheelID.RIGHT_DRIVE_WHEEL)
    elif mode == 'left':
        robot.SetMotorSpeed(-math.pi, WheelID.LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0, WheelID.RIGHT_DRIVE_WHEEL)
    elif mode == 'right':
        robot.SetMotorSpeed(0, WheelID.LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(-math.pi, WheelID.RIGHT_DRIVE_WHEEL)
    else:
        raise ValueError('Invalid mode. Please use "straight", "left", or "right".')

"""
Simulation loop
"""
time = 0
while vis.Run():
    # Move the robot straight for the first 5 seconds
    if time < 5:
        print('Robot is moving straight.')
        move('straight')
    # Turn the robot left for the next 5 seconds
    elif time < 10:
        print('Robot is turning left.')
        move('left')
    # Turn the robot right thereafter
    else:
        print('Robot is turning right.')
        move('right')

    # Increment time counter
    time += 2e-3  # Adjusted time step

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance the simulation by one time step
    system.DoStepDynamics(2e-3)