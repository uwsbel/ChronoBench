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

# Define wheel IDs
LEFT_DRIVE_WHEEL = 0
RIGHT_DRIVE_WHEEL = 1

# Create ground body as terrain plane
ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.6))  # Adjusted position
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# Create Turtlebot Robot
init_pos = chrono.ChVector3d(0, 0.2, 0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
robot = turtlebot.TurtleBot(system, init_pos, init_rot)
robot.Initialize()

# Visualization setup
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

# Removed the shadows line

def move(mode):
    if mode == 'straight':
        robot.SetMotorSpeed(LEFT_DRIVE_WHEEL, math.pi)
        robot.SetMotorSpeed(RIGHT_DRIVE_WHEEL, math.pi)
    elif mode == 'left':
        robot.SetMotorSpeed(LEFT_DRIVE_WHEEL, 0)
        robot.SetMotorSpeed(RIGHT_DRIVE_WHEEL, math.pi)
    elif mode == 'right':
        robot.SetMotorSpeed(LEFT_DRIVE_WHEEL, math.pi)
        robot.SetMotorSpeed(RIGHT_DRIVE_WHEEL, 0)
    else:
        raise ValueError(f"Invalid mode: {mode}")

# Simulation loop
time_step = 2e-3
time = 0.0

while vis.Run():
    # Determine current mode based on time
    if time < 5.0:
        current_mode = 'straight'
    elif time < 10.0:
        current_mode = 'left'
    else:
        current_mode = 'right'
    
    move(current_mode)
    print(f"Current action: {current_mode}")

    # Advance simulation
    system.DoStepDynamics(time_step)
    time += time_step  # Update time after step

    # Render
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

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

# Define wheel IDs
LEFT_DRIVE_WHEEL = 0
RIGHT_DRIVE_WHEEL = 1

# Create ground body with adjusted position
ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.6))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# Create Turtlebot Robot
init_pos = chrono.ChVector3d(0, 0.2, 0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
robot = turtlebot.TurtleBot(system, init_pos, init_rot)
robot.Initialize()

# Visualization setup
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

def move(mode):
    if mode == 'straight':
        robot.SetMotorSpeed(LEFT_DRIVE_WHEEL, math.pi)
        robot.SetMotorSpeed(RIGHT_DRIVE_WHEEL, math.pi)
    elif mode == 'left':
        robot.SetMotorSpeed(LEFT_DRIVE_WHEEL, 0)
        robot.SetMotorSpeed(RIGHT_DRIVE_WHEEL, math.pi)
    elif mode == 'right':
        robot.SetMotorSpeed(LEFT_DRIVE_WHEEL, math.pi)
        robot.SetMotorSpeed(RIGHT_DRIVE_WHEEL, 0)
    else:
        raise ValueError(f"Invalid mode: {mode}")

# Simulation loop
time_step = 2e-3
time = 0.0

while vis.Run():
    # Determine current mode based on time
    if time < 5.0:
        current_mode = 'straight'
    elif time < 10.0:
        current_mode = 'left'
    else:
        current_mode = 'right'
    
    move(current_mode)
    print(f"Current action: {current_mode}")

    # Advance simulation
    system.DoStepDynamics(time_step)
    time += time_step

    # Render
    vis.BeginScene()
    vis.Render()
    vis.EndScene()