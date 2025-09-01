import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as robot
from pychrono import irrlicht as chronoirr

# Create Chrono system
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# Create ground body with contact material and add it to the system
ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.5))  # Position the ground slightly below the origin
ground.SetFixed(True)  # Fix the ground in place
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# Create a long box obstacle
obstacle_mat = chrono.ChContactMaterialNSC()
obstacle = chrono.ChBodyEasyBox(4.0, 0.5, 0.2, 1000, True, True, obstacle_mat)
obstacle.SetPos(chrono.ChVector3d(0, 0, 0.1))  # Positioned in the rover's path
obstacle.SetFixed(True)
obstacle.GetVisualShape(0).SetColor(chrono.ChColor(0.9, 0.4, 0.1))  # Orange color for visibility
system.Add(obstacle)

# Create Curiosity rover and add it to the system
rover = robot.Curiosity(system)

# Create driver for rover and set throttle to move forward
driver = robot.CuriosityDCMotorControl()
driver.SetThrottle(0.3)  # Set throttle to 30% for forward motion
rover.SetDriver(driver)

# Initialize rover position and orientation
init_pos = chrono.ChVector3d(-5, 0.0, 0)  # Updated initial position
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
rover.Initialize(chrono.ChFrameD(init_pos, init_rot))  # Corrected ChFrameD

# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Curiosity rover - Rigid terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 3), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0), 3, 4, 10, 40, 512)

# Set the simulation time step
time_step = 1e-3

# Simulation loop
time = 0
while vis.Run():
    time += time_step

    # Set steering input to zero (no steering)
    driver.SetSteering(0.0)

    # Update rover dynamics
    rover.Update()

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance simulation by one time step
    system.DoStepDynamics(time_step)