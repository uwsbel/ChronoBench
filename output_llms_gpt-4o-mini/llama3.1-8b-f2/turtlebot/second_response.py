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
ground.SetPos(chrono.ChVector3d(0, 0, -0.6))  # Position the ground slightly below the origin
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

# Enable shadows (commented out to improve performance)
# vis.EnableShadows()

# Set the simulation time step
time_step = 2e-3

# Simulation loop
time = 0
robot_mode = "straight"
while vis.Run():
    # Define WheelID enum values for motor control
    LEFT_DRIVE_WHEEL = 0
    RIGHT_DRIVE_WHEEL = 1
    
    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance the simulation by one timestep
    system.DoStepDynamics(time_step)

    # Get the current robot mode
    print(f"Robot mode: {robot_mode}")
    
    # Update the robot's motor speeds based on the current mode
    if robot_mode == "straight":
        robot.SetMotorSpeed(0, LEFT_DRIVE_WHEEL, 0)
        robot.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL, 0)
        if time >= 5:
            robot_mode = "left"
    elif robot_mode == "left":
        robot.SetMotorSpeed(-math.pi, LEFT_DRIVE_WHEEL, 0)
        robot.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL, 0)
        if time >= 10:
            robot_mode = "right"
    elif robot_mode == "right":
        robot.SetMotorSpeed(0, LEFT_DRIVE_WHEEL, 0)
        robot.SetMotorSpeed(math.pi, RIGHT_DRIVE_WHEEL, 0)
    else:
        raise ValueError("Invalid robot mode")
    
    # Increment time counter
    time += time_step