import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as robot
import pychrono.sensor as sens
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
rover.Initialize(chrono.ChFramed(init_pos, init_rot))

# Create sensor manager
manager = sens.ChSensorManager(system)

# Create and configure lidar sensor
lidar_update_rate = 10  # Hz
lidar_horizontal_samples = 800
lidar_vertical_samples = 300
lidar_horizontal_fov = 2 * chrono.CH_PI  # 360 degrees
lidar_vertical_fov_max = chrono.CH_PI / 6  # 30 degrees up
lidar_vertical_fov_min = -chrono.CH_PI / 6  # 30 degrees down
lidar_max_distance = 100
lidar_sample_radius = 2

# Define lidar offset pose relative to rover chassis
lidar_offset_pose = chrono.ChFramed(
    chrono.ChVector3d(0.0, 0, 1.5), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
)

# Create lidar sensor
lidar = sens.ChLidarSensor(
    rover.GetChassis().GetBody(),  # Attach to rover chassis
    lidar_update_rate,
    lidar_offset_pose,
    lidar_horizontal_samples,
    lidar_vertical_samples,
    lidar_horizontal_fov,
    lidar_vertical_fov_max,
    lidar_vertical_fov_min,
    lidar_max_distance,
    lidar_sample_radius
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(0)

# Add noise filter
lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))

# Add visualization filter (optional)
lidar.PushFilter(sens.ChFilterVisualize(lidar_horizontal_samples, lidar_vertical_samples, "Lidar Point Cloud"))

# Add data access filter
lidar.PushFilter(sens.ChFilterDIAccess())

# Add lidar to sensor manager
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

    # Ask rover to move forward - set both steering and throttle
    driver.SetSteering(0.0)
    driver.SetThrottle(0.5)  # Added throttle to make rover move forward

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