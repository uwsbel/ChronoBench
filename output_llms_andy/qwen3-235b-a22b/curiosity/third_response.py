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

# Create a long box for rover to cross (corrected position to avoid ground intersection)
box = chrono.ChBodyEasyBox(0.25, 5, 0.25, 1000, True, True, ground_mat)
box.SetPos(chrono.ChVector3d(0, 0, 0.125))  # Position box on top of ground (z = 0.125)
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

# Create sensor manager
manager = sens.ChSensorManager(system)

# Create lidar sensor parameters
update_rate = 10  # Hz
horizontal_samples = 360
vertical_samples = 1
horizontal_fov = math.pi * 2  # 360 degrees
max_distance = 10.0
vertical_fov = 0.0  # 2D lidar

# Create offset pose for lidar (positioned above rover chassis)
offset_pose = chrono.ChFramed(
    chrono.ChVector3d(0, 0, 0.2),  # Position (x, y, z) relative to chassis
    chrono.ChQuaterniond(1, 0, 0, 0)  # No rotation
)

# Create lidar sensor and attach to rover's chassis
lidar = sens.ChLidarSensor(
    rover.GetChassis(),  # Attach to rover chassis
    update_rate,
    offset_pose,
    horizontal_samples,
    vertical_samples,
    horizontal_fov,
    max_distance,
    vertical_fov
)

# Add filters for lidar data processing
lidar.AddFilter(sens.ChFilterDIAccess())  # Access depth information
lidar.AddFilter(sens.ChFilterPCfromDepth())  # Convert depth to point cloud
lidar.AddFilter(sens.ChFilterVisualizePointCloud(360, 1, "Lidar Point Cloud"))  # Visualize point cloud

# Add lidar sensor to manager
manager.AddSensor(lidar)

# Simulation loop
time = 0
while vis.Run():
    time += time_step

    # Ask rover to move forward
    driver.SetSteering(0.0)

    # Update rover dynamics
    rover.Update()

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance simulation by one time step
    system.DoStepDynamics(time_step)
    
    # Update sensor manager to process lidar data
    manager.Update()