import os
import math
# import numpy as np # Removed as it's unused in the original and modified script
import pychrono as chrono
import pychrono.robot as turtlebot # Using the alias 'turtlebot' for the robot module as in the original
from pychrono import irrlicht as chronoirr

# --- PyChrono system setup ---
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81)) # Gravity in -Z direction
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# --- Create ground ---
ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
# Instruction 1: Ground position adjusted
ground.SetPos(chrono.ChVector3d(0, 0, -0.6)) # Original: (0, 0, -1)
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# --- Create Turtlebot Robot ---
init_pos = chrono.ChVector3d(0, 0.2, 0)  # Initial position (robot will settle on ground)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)  # Initial orientation (no rotation)
robot = turtlebot.TurtleBot(system, init_pos, init_rot)  # Create Turtlebot instance
robot.Initialize()  # Initialize the robot

# --- Robot Control Parameters ---
# Wheel IDs (convention for TurtleBot in PyChrono examples) - MOVED TO GLOBAL SCOPE
LEFT_DRIVE_WHEEL = 0
RIGHT_DRIVE_WHEEL = 1

# Speeds for robot movement
ROBOT_FORWARD_SPEED = math.pi  # rad/s for forward motion
TURN_SPEED_MAGNITUDE = math.pi / 2  # rad/s for wheel speed during pivot turns (turn in place)

# Instruction 3: Added a new function `move(mode)`
def move(mode_command):
    """
    Controls the robot's movement based on the specified mode.
    Args:
        mode_command (str): The desired movement mode.
                           Expected values: "straight", "left", "right".
    Raises:
        ValueError: If an invalid mode_command is provided.
    """
    if mode_command == "straight":
        robot.SetMotorSpeed(ROBOT_FORWARD_SPEED, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(ROBOT_FORWARD_SPEED, RIGHT_DRIVE_WHEEL)
    elif mode_command == "left":
        # For a left turn (pivot on center): left wheel backward, right wheel forward
        robot.SetMotorSpeed(-TURN_SPEED_MAGNITUDE, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(TURN_SPEED_MAGNITUDE, RIGHT_DRIVE_WHEEL)
    elif mode_command == "right":
        # For a right turn (pivot on center): left wheel forward, right wheel backward
        robot.SetMotorSpeed(TURN_SPEED_MAGNITUDE, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(-TURN_SPEED_MAGNITUDE, RIGHT_DRIVE_WHEEL)
    else:
        raise ValueError(f"Invalid move mode: {mode_command}. Must be 'straight', 'left', or 'right'.")

# --- Irrlicht Visualisation Setup ---
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Turtlebot Robot - Modified Control')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1.5, 0.2), chrono.ChVector3d(0, 0, 0.2)) # Camera position and target
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5), 3, 4, 10, 40, 512)

# Instruction 2: Removed the commented-out line for enabling shadows
# The line `# vis.EnableShadows()` is confirmed removed.

# --- Simulation Parameters ---
time_step = 2e-3
time = 0.0  # Initialize simulation time

# --- Simulation Loop ---
# Instruction 4: Updated the simulation loop
current_action = None # Variable to track the robot's current action state

print("Starting simulation...")

while vis.Run():
    # Determine the robot's action based on the current simulation time
    new_action_this_step = None
    if time < 5.0:
        new_action_this_step = "straight"
    elif time < 10.0: # 5.0 <= time < 10.0
        new_action_this_step = "left"
    else: # time >= 10.0
        new_action_this_step = "right"

    # If the action has changed since the last step, update motor speeds and print status
    if new_action_this_step != current_action:
        current_action = new_action_this_step
        print(f"Time: {time:.2f}s - Robot action: Changing to {current_action}")
        move(current_action) # Apply the new movement command

    # Advance simulation by one time step
    system.DoStepDynamics(time_step)

    # Increment simulation time
    time += time_step

    # Render the scene
    vis.BeginScene(True, True, chrono.ChColor(0.2, 0.2, 0.3)) # Clear background and Z-buffer
    vis.Render()
    vis.EndScene()

print("Simulation finished.")