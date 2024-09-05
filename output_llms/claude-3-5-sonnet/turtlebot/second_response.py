import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as turtlebot
from pychrono import irrlicht as chronoirr

# Create Chrono system
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector(0, 0, -9.81))  # Set gravity in negative Z direction
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# Create ground body as terrain plane that robot will drive on
ground_mat = chrono.ChMaterialSurfaceNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVectorD(0, 0, -0.6))  # Adjusted ground position
ground.SetBodyFixed(True)  # Fix the ground in place
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"), 20, 20)
system.Add(ground)

# Create Turtlebot Robot
init_pos = chrono.ChVectorD(0, 0, 0.2)  # Initial position of the robot
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)  # Initial orientation of the robot
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
vis.AddCamera(chrono.ChVectorD(0, 1.5, 0.2), chrono.ChVectorD(0, 0, 0.2))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(1.5, -2.5, 5.5), chrono.ChVectorD(0, 0, 0.5), 3, 4, 10, 40, 512)

# Set the simulation time step
time_step = 2e-3

# Define WheelID enum values for motor control
LEFT_DRIVE_WHEEL = 0
RIGHT_DRIVE_WHEEL = 1

# Function to control robot movement
def move(mode):
    if mode == "straight":
        robot.SetMotorSpeed(math.pi, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(math.pi, RIGHT_DRIVE_WHEEL)
    elif mode == "left":
        robot.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(math.pi, RIGHT_DRIVE_WHEEL)
    elif mode == "right":
        robot.SetMotorSpeed(math.pi, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL)
    else:
        raise ValueError("Invalid movement mode")

# Simulation loop
time = 0
while vis.Run():
    # Control robot movement based on time
    if time < 5:
        move("straight")
        print("Moving straight")
    elif 5 <= time < 10:
        move("left")
        print("Turning left")
    else:
        move("right")
        print("Turning right")

    # Increment time counter
    time += time_step

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance the simulation by one time step
    system.DoStepDynamics(time_step)