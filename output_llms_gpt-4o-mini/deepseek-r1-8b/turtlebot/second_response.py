import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as turtlebot
from pychrono import irrlicht as chronoirr

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

# Create run-time visualization
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

# Remove commented-out line for enabling shadows
# vis.EnableShadows()

# Define movement modes
class MovementMode:
    STRAIGHT = "straight"
    LEFT = "left"
    RIGHT = "right"

def move(mode):
    """Control the robot's movement with the specified mode."""
    if mode not in MovementMode.__dict__.values():
        raise ValueError(f"Invalid mode: {mode}")
    
    # Initialize motor speeds to zero
    left_speed = 0
    right_speed = 0
    
    if mode == MovementMode.STRAIGHT:
        left_speed = 1
        right_speed = 1
    elif mode == MovementMode.LEFT:
        left_speed = 1
        right_speed = -1
    elif mode == MovementMode.RIGHT:
        left_speed = -1
        right_speed = 1
    
    robot.SetMotorSpeed(left_speed, LEFT_DRIVE_WHEEL)
    robot.SetMotorSpeed(right_speed, RIGHT_DRIVE_WHEEL)
    
    return

# Set the simulation time step
time_step = 2e-3

# Simulation loop
time = 0
while vis.Run():
    # Define WheelID enum values for motor control
    LEFT_DRIVE_WHEEL = 0
    RIGHT_DRIVE_WHEEL = 1
    
    # Update robot movement based on time
    move_mode = MovementMode.STRAIGHT
    if time >= 5:
        if time >= 10:
            move_mode = MovementMode.RIGHT
        else:
            move_mode = MovementMode.LEFT
    elif time >= 5:
        move_mode = MovementMode.LEFT
    
    # Print current action
    print(f"Robot action: {move_mode}")
    
    # At time = 1 s, start left turn
    if abs(time - 1.0) < 1e-4:
        move(MovementMode.LEFT)
        
    # At time = 2 s, start right turn
    if abs(time - 2.0) < 1e-4:
        move(MovementMode.RIGHT)
    
    # Increment time counter
    time += time_step

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance the simulation by one time step
    system.DoStepDynamics(time_step)