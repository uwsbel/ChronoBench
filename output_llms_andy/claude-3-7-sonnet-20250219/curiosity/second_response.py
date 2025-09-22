import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as robot
from pychrono import irrlicht as chronoirr

# Create Chrono system
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# Create ground body with contact material and add it to the system
ground_mat = chrono.ChMaterialSurfaceNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector(0, 0, -0.5))  # Position the ground slightly below the origin
ground.SetBodyFixed(True)  # Fix the ground in place
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# Create an obstacle (long box) for the rover to cross
obstacle_mat = chrono.ChMaterialSurfaceNSC()
obstacle = chrono.ChBodyEasyBox(1, 5, 0.2, 1000, True, True, obstacle_mat)
obstacle.SetPos(chrono.ChVector(-2, 0, 0.1))  # Position the obstacle in the rover's path
obstacle.SetBodyFixed(True)  # Fix the obstacle in place
system.Add(obstacle)

# Create Curiosity rover and add it to the system
rover = robot.Curiosity(system)

# Create driver for rover
driver = robot.CuriosityDCMotorControl()
rover.SetDriver(driver)

# Initialize rover position and orientation
init_pos = chrono.ChVector(-5, 0.0, 0)  # Changed from (0, 0.2, 0) to (-5, 0.0, 0)
init_rot = chrono.ChQuaternion(1, 0, 0, 0)
rover.Initialize(chrono.ChFrame(init_pos, init_rot))

# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Curiosity rover - Rigid terrain with obstacle')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector(-5, 3, 3), chrono.ChVector(-5, 0, 0))  # Adjusted camera to view the rover
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector(1.5, -2.5, 5.5), chrono.ChVector(0, 0, 0), 3, 4, 10, 40, 512)

# Enable shadows (commented out to improve performance)
# vis.EnableShadows()

# Set the simulation time step
time_step = 1e-3

# Simulation loop
time = 0
while vis.Run():
    time += time_step

    # Set constant forward motion with zero steering
    steering = 0  # Zero steering input
    driver.SetSteering(steering)
    
    # Set throttle to move forward
    driver.SetThrottle(0.4)  # Added throttle for forward movement

    # Update rover dynamics
    rover.Update()

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance simulation by one time step
    system.DoStepDynamics(time_step)