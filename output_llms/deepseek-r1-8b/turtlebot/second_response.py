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

def move(mode):
    """Control the robot's movement with the following modes:
    - 'straight': Move forward straight
    - 'left': Turn left (clockwise)
    - 'right': Turn right (counterclockwise)
    Raises ValueError for invalid mode."""
    valid_modes = ['straight', 'left', 'right']
    if mode not in valid_modes:
        raise ValueError(f"Invalid mode: {mode}. Must be one of {valid_modes}")
    
    if mode == 'straight':
        robot.SetMotorSpeed(1.0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(1.0, RIGHT_DRIVE_WHEEL)
    elif mode == 'left':
        robot.SetMotorSpeed(-math.pi, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(math.pi, RIGHT_DRIVE_WHEEL)
    elif mode == 'right':
        robot.SetMotorSpeed(math.pi, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(-math.pi, RIGHT_DRIVE_WHEEL)

# Simulation loop
time = 0
while vis.Run():
    # Define WheelID enum values for motor control
    LEFT_DRIVE_WHEEL = 0
    RIGHT_DRIVE_WHEEL = 1
    
    current_time = time
    elapsed_time = current_time - 0.0  # Track time since last mode change
    
    # Update robot position and orientation
    robot.Update()
    
    # Print current action
    print(f"Current action: {mode} for {max(0, 5 - elapsed_time)}s remaining")
    
    # Check if elapsed time has passed to switch modes
    if elapsed_time >= 5.0:
        if mode == 'straight':
            print("Switching to left turn mode...")
            mode = 'left'
        elif mode == 'left':
            print("Switching to right turn mode...")
            mode = 'right'
        elif mode == 'right':
            print("Switching to straight forward mode...")
            mode = 'straight'
        
        # Update time to reset the mode change timer
        time = current_time
    
    # Advance the simulation by one time step
    system.DoStepDynamics(time_step)

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()