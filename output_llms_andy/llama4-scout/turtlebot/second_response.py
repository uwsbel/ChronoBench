import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as turtlebot
from pychrono import irrlicht as chronoirr

# Create Chrono system
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0,0, -9.81)) # Set gravity in negative Z direction
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# Create ground body as terrain plane that robot will drive on
ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20,20,1,1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0,0, -0.6)) # Adjusted ground position
ground.SetFixed(True) # Fix the ground in place
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# Create Turtlebot Robot
init_pos = chrono.ChVector3d(0,0.2,0) # Initial position of the robot
init_rot = chrono.ChQuaterniond(1,0,0,0) # Initial orientation of the robot
robot = turtlebot.TurtleBot(system, init_pos, init_rot) # Create Turtlebot instance
robot.Initialize() # Initialize the robot

# Function to control the robot's movement
def move(mode):
    """
    Control the robot's movement based on the given mode.

    Parameters:
    mode (str): Movement mode. Can be 'straight', 'left', or 'right'.

    Raises:
    ValueError: If the mode is not one of 'straight', 'left', or 'right'.
    """
    LEFT_DRIVE_WHEEL = 0
    RIGHT_DRIVE_WHEEL = 1
    speed = math.pi

    if mode == 'straight':
        robot.SetMotorSpeed(speed, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(speed, RIGHT_DRIVE_WHEEL)
    elif mode == 'left':
        robot.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(speed, RIGHT_DRIVE_WHEEL)
    elif mode == 'right':
        robot.SetMotorSpeed(speed, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL)
    else:
        raise ValueError("Invalid mode. Mode should be 'straight', 'left', or 'right'.")

# Create run-time visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280,720)
vis.SetWindowTitle('Turtlebot Robot - Rigid terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0,1.5,0.2), chrono.ChVector3d(0,0,0.2))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5,5.5), chrono.ChVector3d(0,0,0.5),3,4,10,40,512)

vis.EnableShadows()  # Enable shadows

# Set the simulation time step
time_step = 2e-3

# Simulation loop
time = 0
while vis.Run():
    # Move the robot straight for the first 5 seconds
    if time < 5:
        move('straight')
        print("Moving straight")
    # Turn the robot left for the next 5 seconds
    elif time < 10:
        move('left')
        print("Turning left")
    # Turn the robot right thereafter
    else:
        move('right')
        print("Turning right")

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance the simulation by one time step
    system.DoStepDynamics(time_step)

    # Increment time counter
    time += time_step