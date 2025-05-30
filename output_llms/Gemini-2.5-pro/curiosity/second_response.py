import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as robot
from pychrono import irrlicht as chronoirr

# Create Chrono system
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81)) # Z is up
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# Create ground body with contact material and add it to the system
# Use ChMaterialSurfaceNSC (current naming)
ground_mat = chrono.ChMaterialSurfaceNSC()
# Set explicit properties if desired, e.g.:
# ground_mat.SetFriction(0.7)
# ground_mat.SetRestitution(0.1)

ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.5))  # Position the ground so its top surface is at z=0
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# --- Modification 1: Add a long box obstacle ---
obs_dims = chrono.ChVector3d(0.5, 4.0, 0.2) # dx, dy, dz (0.5m long along X, 4m wide along Y, 0.2m high)
obs_pos = chrono.ChVector3d(0, 0, obs_dims.z / 2.0) # Centered at x=0, y=0, base on ground (z=0)

obstacle_mat = chrono.ChMaterialSurfaceNSC() # Can use ground_mat or a new one
# obstacle_mat.SetFriction(0.6) # Example property

obstacle = chrono.ChBodyEasyBox(obs_dims.x, obs_dims.y, obs_dims.z, 1000, True, True, obstacle_mat)
obstacle.SetPos(obs_pos)
obstacle.SetFixed(True)
obstacle.GetVisualShape(0).SetColor(chrono.ChColor(0.6, 0.6, 0.7)) # Grey color for obstacle
system.Add(obstacle)
# --- End Modification 1 ---

# Create Curiosity rover and add it to the system
rover = robot.Curiosity(system) # System and contact method passed here

# Create driver for rover
driver = robot.CuriosityDCMotorControl()
rover.SetDriver(driver)

# Initialize rover position and orientation
# --- Modification 2: Changed initial position of the rover ---
# Old: init_pos = chrono.ChVector3d(0, 0.2, 0)
# New position: (-5, 0.0, 0.3). Z=0.3m to place wheels slightly above ground (wheel radius ~0.255m)
init_pos = chrono.ChVector3d(-5, 0.0, 0.3)
# --- End Modification 2 ---
init_rot = chrono.ChQuaterniond(1, 0, 0, 0) # Facing positive X direction
rover.Initialize(chrono.ChFramed(init_pos, init_rot))


# --- Modification 3: Set rover to move forward with zero steering ---
# Set constant steering and motor voltage once after initialization.
# CuriosityDCMotorControl caches these values.
driver.SetSteering(0.0) # Zero steering input

forward_voltage = 10.0  # Adjust this voltage for desired speed.
                        # Positive voltage for forward, negative for reverse.
                        # Curiosity has 6 driven wheels.
for i in range(6):
    driver.SetMotorVoltage(forward_voltage, i)
# --- End Modification 3 ---


# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Curiosity rover - Obstacle crossing')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
# Adjusted camera to view rover start and approach to obstacle
vis.AddCamera(chrono.ChVector3d(-7, 3, 2), chrono.ChVector3d(0, 0, 0.5))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0), 3, 4, 10, 40, 512)

# Optional: Enable shadows for all lights (can impact performance)
# vis.EnableShadows()

# Set the simulation time step
time_step = 1e-3

# Simulation loop
time = 0
while vis.Run():
    current_time = system.GetChTime() # Get current simulation time

    # The old steering logic is removed as per Modification 3
    # driver.SetSteering(0.0) # Already set once, or can be set here if needed for some reason

    # Update rover dynamics (includes driver actions)
    rover.Update()

    # Render the scene
    vis.BeginScene(True, True, chrono.ChColor(0.2, 0.2, 0.3)) # Clear color
    vis.Render()
    vis.EndScene()

    # Advance simulation by one time step
    system.DoStepDynamics(time_step)

    # Optional: Print rover position or other info
    # if int(current_time * 100) % 10 == 0: # Print every 0.1s
    #     print(f"Time: {current_time:.2f} s, Rover Pos: {rover.GetChassis().GetPos()}")

    # Stop simulation after a certain time (e.g., 15 seconds)
    if current_time > 15:
        vis.GetDevice().closeDevice()


delপ্রেম