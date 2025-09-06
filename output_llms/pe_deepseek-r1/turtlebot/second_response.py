import os
import math
import numpy as np
import pychrono as chrono
from pychrono.robots.turtlebot import TurtleBot  # Corrected import
import pychrono.irrlicht as chronoirr

# Wheel ID definitions (moved outside loop)
LEFT_DRIVE_WHEEL = 0
RIGHT_DRIVE_WHEEL = 1

# Movement control function
def move(robot, mode):
    """Control robot movement based on specified mode"""
    if mode == "straight":
        robot.SetMotorSpeed(LEFT_DRIVE_WHEEL, math.pi)  # Both wheels same speed
        robot.SetMotorSpeed(RIGHT_DRIVE_WHEEL, math.pi)
    elif mode == "left":
        robot.SetMotorSpeed(LEFT_DRIVE_WHEEL, 0)        # Left wheel stops
        robot.SetMotorSpeed(RIGHT_DRIVE_WHEEL, math.pi)  # Right wheel moves forward
    elif mode == "right":
        robot.SetMotorSpeed(LEFT_DRIVE_WHEEL, math.pi)   # Left wheel moves forward
        robot.SetMotorSpeed(RIGHT_DRIVE_WHEEL, 0)        # Right wheel stops
    else:
        raise ValueError("Invalid movement mode")

# Create Chrono system
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravity(chrono.ChVector3d(0, 0, -9.81))  # Corrected gravity setting
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# Create ground body
ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.6))  # Adjusted ground position
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# Create Turtlebot Robot
init_pos = chrono.ChVector3d(0, 0.2, 0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
robot = TurtleBot(system, init_pos, init_rot)  # Corrected class name
robot.Initialize()

# Create run-time visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Turtlebot Robot - Rigid terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 1), chrono.ChVector3d(0, 0, 0.2))  # Better camera position
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5), 3, 4, 10, 40, 512)

# Set the simulation time step
time_step = 2e-3

# Simulation loop
time = 0
current_mode = None
while vis.Run():
    # Determine movement mode based on time
    if time < 5:
        mode = "straight"
    elif time < 10:
        mode = "left"
    else:
        mode = "right"
    
    # Update movement if mode changed
    if mode != current_mode:
        move(robot, mode)
        print(f"Time {time:.1f}s: Robot now {mode}")
        current_mode = mode

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance simulation
    system.DoStepDynamics(time_step)
    time += time_step