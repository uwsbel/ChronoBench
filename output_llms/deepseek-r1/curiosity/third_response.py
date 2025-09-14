import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as robot
import pychrono.sensor as sens  # 1. Added sensor module import
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

# Initialize rover position and orientation (corrected ChFrameD typo)
init_pos = chrono.ChVector3d(-5, 0.0, 0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
rover.Initialize(chrono.ChFrameD(init_pos, init_rot))  # Fixed ChFramed -> ChFrameD

# Create sensor manager
manager = sens.ChSensorManager(system)  # 2. Created sensor manager

# 3. Add lidar sensor to rover's chassis
lidar_body = rover.GetChassisBody()  # Get chassis body
lidar_offset = chrono.ChVector3d(0.5, 0, 0.2)  # Offset from chassis center
lidar_pose = chrono.ChFrameD(lidar_offset, chrono.Q_from_AngZ(0))

# Lidar parameters
update_rate = 10
horizontal_samples = 4500
vertical_samples = 32
horizontal_fov = chrono.CH_PI  # 180 degrees
vertical_fov = chrono.CH_PI / 6  # 30 degrees
max_vert_angle = vertical_fov / 2
min_vert_angle = -vertical_fov / 2
lag = 0
exposure_time = 0.1

# Create lidar sensor
lidar = sens.ChLidarSensor(
    lidar_body,
    update_rate,
    lidar_pose,
    horizontal_samples,
    vertical_samples,
    horizontal_fov,
    max_vert_angle,
    min_vert_angle,
    lag,
    exposure_time,
    sens.LensModelType_PINHOLE,
    False
)

# Configure lidar filters
lidar.PushFilter(sens.FilterDIAccess())  # Allow depth/intensity access
lidar.PushFilter(sens.FilterXYZIAccess())  # Add XYZI format
lidar.SetName("Rover Lidar")
lidar.SetDescription("Custom Lidar Sensor")

# Add sensor to manager
manager.AddSensor(lidar)  # 3. Added lidar to manager

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

    # Ask rover to move forward
    driver.SetSteering(0.0)

    # Update rover dynamics
    rover.Update()

    # 4. Update sensor manager
    manager.Update()  # Added sensor manager update

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance simulation by one time step
    system.DoStepDynamics(time_step)