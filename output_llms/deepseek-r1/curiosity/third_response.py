import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as robot
import pychrono.sensor as sens  # Added sensor module import
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

# Initialize rover position and orientation (fixed ChFramed typo to ChFrameD)
init_pos = chrono.ChVector3d(-5, 0.0, 0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
rover.Initialize(chrono.ChFrameD(init_pos, init_rot))

# Create sensor manager
manager = sens.ChSensorManager(system)

# Lidar parameters
lidar_offset = chrono.ChVector3d(0.5, 0, 0.2)  # Offset from chassis
lidar_rot = chrono.ChQuaterniond(1, 0, 0, 0)
lidar_update_rate = 10
lidar_hor_samples = 450
lidar_ver_samples = 45
lidar_hor_fov = 2 * chrono.CH_PI  # 360 degree horizontal FOV
lidar_ver_fov = chrono.CH_PI / 6  # 30 degree vertical FOV
lidar_max_dist = 20.0

# Add lidar to rover's chassis
chassis = rover.GetChassis()
lidar = sens.ChLidarSensor(
    chassis,             # Parent body
    lidar_update_rate,   # Update rate
    chrono.ChFrameD(lidar_offset, lidar_rot),  # Pose
    lidar_hor_samples,   # Horizontal samples
    lidar_ver_samples,   # Vertical samples
    lidar_hor_fov,       # Horizontal FOV
    lidar_ver_fov,       # Vertical FOV
    lidar_max_dist       # Maximum distance
)

# Configure lidar
lidar.SetName("Lidar Sensor")
lidar.SetLag(0.0)
lidar.SetCollectionWindow(0.0)

# Add noise model
noise_model = sens.ChNoiseNormal(0.0, 0.1)
lidar.AddNoiseModel(noise_model)

# Add visualization to sensor
lidar.PushFilter(sens.ChFilterVisualize(lidar_hor_samples, lidar_ver_samples, "Lidar Data"))

# Add sensor to manager
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

# Set the simulation time step
time_step = 1e-3

# Create driver for rover
driver = robot.CuriosityDCMotorControl()
rover.SetDriver(driver)

# Simulation loop
time = 0
while vis.Run():
    time += time_step

    # Ask rover to move forward
    driver.SetSteering(0.0)

    # Update rover dynamics
    rover.Update()

    # Update sensor manager
    manager.Update()

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance simulation by one time step
    system.DoStepDynamics(time_step)