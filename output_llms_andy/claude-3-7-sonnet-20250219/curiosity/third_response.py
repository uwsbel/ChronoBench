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
system.SetGravitationalAcceleration(chrono.ChVector(0, 0, -9.81))  # Corrected ChVector3d to ChVector
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# Create ground body with contact material and add it to the system
ground_mat = chrono.ChMaterialSurfaceNSC()  # Corrected to ChMaterialSurfaceNSC
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector(0, 0, -0.5))  # Corrected ChVector3d to ChVector
ground.SetBodyFixed(True)  # Corrected SetFixed to SetBodyFixed
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# create a long box for rover to cross
box = chrono.ChBodyEasyBox(0.25, 5, 0.25, 1000, True, True, ground_mat)
box.SetPos(chrono.ChVector(0, 0, 0.0))  # Corrected ChVector3d to ChVector
box.SetBodyFixed(True)  # Corrected SetFixed to SetBodyFixed
box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
system.Add(box)

# Create Curiosity rover and add it to the system
rover = robot.Curiosity(system)

# Create driver for rover
driver = robot.CuriosityDCMotorControl()
rover.SetDriver(driver)

# Initialize rover position and orientation
init_pos = chrono.ChVector(-5, 0.0, 0)  # Corrected ChVector3d to ChVector
init_rot = chrono.ChQuaternion(1, 0, 0, 0)  # Corrected ChQuaterniond to ChQuaternion
rover.Initialize(chrono.ChFrame(init_pos, init_rot))  # Corrected ChFramed to ChFrame

# Create the sensor manager and add it to the system
manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(chrono.ChVector(100, 100, 100), chrono.ChColor(1, 1, 1), 500.0)

# Create a lidar sensor
lidar_update_rate = 10.0  # Hz
lidar_horizontal_samples = 1000
lidar_vertical_samples = 16
lidar_horizontal_fov = 2 * chrono.CH_C_PI  # 360 degrees
lidar_vertical_fov = chrono.CH_C_PI / 12  # 15 degrees
lidar_max_range = 100.0  # meters
lidar_min_range = 0.1  # meters

# Get the rover chassis body for sensor mounting
rover_body = rover.GetChassisBody()

# Define the position and orientation of the lidar relative to the rover
lidar_offset_pose = chrono.ChFrame(chrono.ChVector(0, 0, 1.0), chrono.ChQuaternion(1, 0, 0, 0))

# Create the lidar sensor
lidar = sens.ChLidarSensor(
    rover_body,              # body lidar is attached to
    lidar_update_rate,       # update rate in Hz
    lidar_offset_pose,       # offset pose relative to body
    lidar_horizontal_samples,# number of horizontal samples
    lidar_vertical_samples,  # number of vertical samples
    lidar_horizontal_fov,    # horizontal field of view
    lidar_vertical_fov,      # vertical field of view
    lidar_min_range,         # minimum range
    lidar_max_range          # maximum range
)

# Add noise model (optional)
lidar.SetLidarNoise(sens.ChLidarNoiseNone())

# Add visualization filter (wireframe visualization)
lidar_vis = sens.ChFilterVisualize(1280, 720, "Lidar")
lidar.AddFilter(lidar_vis)

# Add point cloud visualization
point_cloud_vis = sens.ChFilterVisualizePointCloud(1280, 720, "Lidar Point Cloud", 0.1)
lidar.AddFilter(point_cloud_vis)

# Add the lidar to the sensor manager
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
vis.AddCamera(chrono.ChVector(0, 3, 3), chrono.ChVector(0, 0, 0))  # Corrected ChVector3d to ChVector
vis.AddTypicalLights()
vis.AddLight(chrono.ChVectorD(1.5, -2.5, 5.5), 3, chrono.ChColor(0.7, 0.7, 0.7))  # Updated AddLightWithShadow to AddLight

# Set the simulation time step
time_step = 1e-3

# Simulation loop
time = 0
while vis.Run():
    time += time_step

    # ask rover to move forward
    driver.SetThrottle(0.5)  # Added throttle command to make the rover move
    driver.SetSteering(0.0)

    # Update rover dynamics
    rover.Update()
    
    # Update the sensor manager
    manager.Update()

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance simulation by one time step
    system.DoStepDynamics(time_step)