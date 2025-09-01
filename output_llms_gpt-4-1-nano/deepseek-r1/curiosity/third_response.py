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

# Create sensor manager
manager = sens.ChSensorManager(system)  # Added sensor manager

# Create ground body with contact material and add it to the system
ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.5))
ground.SetFixed(True)
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

# Initialize rover position and orientation (corrected ChFrameD)
init_pos = chrono.ChVector3d(-5, 0.0, 0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
rover.Initialize(chrono.ChFrameD(init_pos, init_rot))  # Fixed ChFramed to ChFrameD

# Add lidar sensor to rover's chassis
chassis = rover.GetChassis()
lidar_offset = chrono.ChVector3d(0, 0, 0.5)  # Offset from chassis
lidar_pose = chrono.ChFrameD(lidar_offset, chrono.QUNIT)

# Lidar parameters
update_rate = 10
horizontal_samples = 1000
vertical_samples = 50
horizontal_fov = 360 * chrono.CH_DEG_TO_RAD
vertical_fov = 30 * chrono.CH_DEG_TO_RAD
max_distance = 100.0

# Create and configure lidar
lidar = sens.ChLidarSensor(
    chassis,
    update_rate,
    lidar_pose,
    horizontal_samples,
    vertical_samples,
    horizontal_fov,
    vertical_fov,
    max_distance,
    sens.VisualizationType_POINTS
)

# Set lidar properties
lidar.SetName("Lidar Sensor")
lidar.SetLag(0.0)
lidar.SetCollectionWindow(0.0)

# Add filters
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Lidar Data"))
lidar.PushFilter(sens.ChFilterAccessXYZ())

# Add sensor to manager
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

    # Ask rover to move forward
    driver.SetSteering(0.0)

    # Update rover dynamics
    rover.Update()

    # Update sensor manager (added)
    manager.Update()

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance simulation by one time step
    system.DoStepDynamics(time_step)