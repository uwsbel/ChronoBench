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

# Contact material
ground_mat = chrono.ChContactMaterialNSC()

# Create ground body
ground = chrono.ChBodyEasyBox(
    20, 20, 1,
    1000,
    True,
    True,
    ground_mat
)
ground.SetPos(chrono.ChVector3d(0, 0, -0.5))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(
    chrono.GetChronoDataFile("textures/concrete.jpg")
)
system.Add(ground)

# -------------------------------------------------------------------------
# Add a long box obstacle for the rover to cross
# The obstacle is fixed to the ground and placed across the rover path.
# Rover starts at x = -5 and moves in the +X direction.
# -------------------------------------------------------------------------
obstacle_mat = chrono.ChContactMaterialNSC()

obstacle_length_y = 6.0
obstacle_width_x = 0.35
obstacle_height_z = 0.25

obstacle = chrono.ChBodyEasyBox(
    obstacle_width_x,
    obstacle_length_y,
    obstacle_height_z,
    1000,
    True,
    True,
    obstacle_mat
)

# Put the obstacle on top of the ground, centered at x = 0
obstacle.SetPos(
    chrono.ChVector3d(
        0.0,
        0.0,
        obstacle_height_z / 2.0
    )
)
obstacle.SetFixed(True)
obstacle.GetVisualShape(0).SetTexture(
    chrono.GetChronoDataFile("textures/concrete.jpg")
)
system.Add(obstacle)

# Create Curiosity rover and add it to the system
rover = robot.Curiosity(system)

# Create driver for rover
driver = robot.CuriosityDCMotorControl()
rover.SetDriver(driver)

# Initialize rover position and orientation
# Modified initial position from (0, 0.2, 0) to (-5, 0.0, 0)
init_pos = chrono.ChVector3d(-5, 0.0, 0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
rover.Initialize(chrono.ChFramed(init_pos, init_rot))

# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Curiosity rover - Rigid terrain with obstacle")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(
    chrono.ChVector3d(-6, 4, 3),
    chrono.ChVector3d(0, 0, 0)
)
vis.AddTypicalLights()
vis.AddLightWithShadow(
    chrono.ChVector3d(1.5, -2.5, 5.5),
    chrono.ChVector3d(0, 0, 0),
    3,
    4,
    10,
    40,
    512
)

# Enable shadows if desired
# vis.EnableShadows()

# Set the simulation time step
time_step = 1e-3

# Simulation loop
while vis.Run():
    # Rover moves forward with zero steering input
    driver.SetSteering(0.0)

    # Update rover dynamics
    rover.Update()

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance simulation by one time step
    system.DoStepDynamics(time_step)