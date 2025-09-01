import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as turtlebot
from pychrono import irrlicht as chronoirr

# Create Chrono system
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
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
robot = turtlebot.TurtleBot(system, init_pos, init_rot)
robot.Initialize()

# Define motor IDs
LEFT_DRIVE_WHEEL = 0
RIGHT_DRIVE_WHEEL = 1

# Movement control function
def move(mode):
    if mode == 'straight':
        robot.SetMotorSpeed(math.pi, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(math.pi, RIGHT_DRIVE_WHEEL)
    elif mode == 'left':
        robot.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(-math.pi, RIGHT_DRIVE_WHEEL)
    elif mode == 'right':
        robot.SetMotorSpeed(-math.pi, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL)
    else:
        raise ValueError(f"Invalid movement mode: {mode}")

# Create visualization
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

# Simulation time step
time_step = 2e-3

# Simulation loop
time = 0
previous_mode = None

while vis.Run():
    current_time = time

    # Determine current movement mode
    if current_time < 5.0:
        current_mode = 'straight'
    elif 5.0 <= current_time < 10.0:
        current_mode = 'left'
    else:
        current_mode = 'right'

    # Print action on mode change
    if current_mode != previous_mode:
        if current_mode == 'straight':
            print("Moving straight")
        elif current_mode == 'left':
            print("Turning left")
        else:
            print("Turning right")
        previous_mode = current_mode

    # Apply movement
    move(current_mode)

    # Increment time
    time += time_step

    # Render and step simulation
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(time_step)