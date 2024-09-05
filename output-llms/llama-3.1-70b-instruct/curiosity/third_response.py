"""
Modified PyChrono script to add a lidar sensor to the Curiosity rover
"""

import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as robot
from pychrono import irrlicht as chronoirr
import pychrono.sensor as sens  # Added sensor module import

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

# Create a long box for rover to cross
box = chrono.ChBodyEasyBox(0.25, 5, 0.25, 1000, True, True, ground_mat)
box.SetPos(chrono.ChVector3d(0, 0, 0.0))
box.SetFixed(True)
box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
system.Add(box)

# Create Curiosity rover and add it to the system
rover = robot.Curiosity(system)

# Create driver for rover
driver = robot.CuriosityDCMotorControl()
rover.SetDriver(driver)

# Initialize rover position and orientation
init_pos = chrono.ChVector3d(-5, 0.0, 0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
rover.Initialize(chrono.ChFrameD(init_pos, init_rot))  # Corrected ChFramed to ChFrameD

# Create sensor manager
manager = sens.ChSensorManager(system)  # Created sensor manager

# Define lidar parameters
lidar_update_rate = 10  # Hz
lidar_horizontal_samples = 512
lidar_vertical_samples = 64
lidar_field_of_view = math.radians(60)
lidar_range = 10  # meters

# Create lidar sensor and attach it to the rover's chassis
lidar = sens.ChLidarSensor(
    rover.GetChassisBody(),
    chrono.ChVector3d(0, 0, 1.5),  # Position of the lidar on the chassis
    chrono.ChQuaterniond(1, 0, 0, 0),  # Orientation of the lidar
    lidar_update_rate,
    lidar_horizontal_samples,
    lidar_vertical_samples,
    lidar_field_of_view,
    lidar_range,
)
lidar.SetName("LidarSensor")

# Set up filters for the lidar data
lidar.SetFilter(sens.ChLidarFilter(sens.ChLidarFilter.Type_RAYCAST))

# Add the lidar sensor to the sensor manager
manager.AddSensor(lidar)

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

# Enable shadows (commented out to improve performance)
# vis.EnableShadows()

# Set the simulation time step
time_step = 1e-3

# Simulation loop
time = 0
while vis.Run():
    time += time_step

    # Ask rover to move forward
    driver.SetSteering(0.0)

    # Update rover dynamics
    rover.Update()

    # Update sensor manager
    manager.Update()  # Added sensor manager update

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance simulation by one time step
    system.DoStepDynamics(time_step)