import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as robot
import pychrono.sensor as sens  # Added sensor module
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

# create a long box for rover to cross
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
rover.Initialize(chrono.ChFrameD(init_pos, init_rot))  # Fixed frame initialization

# Create sensor manager
manager = sens.ChSensorManager(system)

# Configure lidar sensor parameters
lidar_offset_pose = chrono.ChFramed(chrono.ChVector3d(0, 0, 0.2), chrono.Q_ROTATE_Y_TO_Z)
lidar_params = sens.ChLidarSensor.pointcloud()
lidar_params.hFOV = 180  # Horizontal FOV in degrees
lidar_params.max_distance = 100
lidar_params.samples = 2250  # Horizontal samples
lidar_params.vertical_samples = 64  # Vertical samples
lidar_params.vFOV = 20  # Vertical FOV in degrees
lidar_params.update_rate = 10  # Hz

# Create and configure lidar sensor
lidar = sens.ChLidarSensor(
    rover.GetChassisBody(),  # Attach to rover chassis
    update_rate=lidar_params.update_rate,
    lidar_offset_pose,
    lidar_params.samples,
    lidar_params.vertical_samples,
    lidar_params.hFOV,
    lidar_params.vFOV,
    lidar_params.max_distance
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0.0)
lidar.SetCollectionWindow(0.0)

# Add noise model and filter
noise_model = sens.ChNoiseNormalDist(0, 0.1)
lidar.AddNoiseModel(noise_model)
lidar.PushFilter(sens.ChFilterPCfromDepth())
manager.AddSensor(lidar)

# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Curiosity rover with Lidar')
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

    # Control rover movement
    driver.SetSteering(0.0)
    driver.SetSpeed(0.5)  # Added speed control for forward motion

    # Update rover dynamics
    rover.Update()
    
    # Update sensor manager
    manager.Update()  # Added sensor update

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance simulation by one time step
    system.DoStepDynamics(time_step)