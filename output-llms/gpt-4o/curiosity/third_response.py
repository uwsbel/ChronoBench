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
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# Create ground body with contact material and add it to the system
ground_mat = chrono.ChMaterialSurfaceNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVectorD(0, 0, -0.5))  # Position the ground slightly below the origin
ground.SetFixed(True)  # Fix the ground in place
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# create a long box for rover to cross
box = chrono.ChBodyEasyBox(0.25, 5, 0.25, 1000, True, True, ground_mat)
box.SetPos(chrono.ChVectorD(0, 0, 0.0))
box.SetFixed(True)
box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
system.Add(box)

# Create Curiosity rover and add it to the system
rover = robot.Curiosity(system)

# Create driver for rover
driver = robot.CuriosityDCMotorControl()
rover.SetDriver(driver)

# Initialize rover position and orientation
init_pos = chrono.ChVectorD(-5, 0.0, 0)
init_rot = chrono.ChQuaternion(1, 0, 0, 0)
rover.Initialize(chrono.ChFrame(init_pos, init_rot))

# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Curiosity rover - Rigid terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 3, 3), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(1.5, -2.5, 5.5), chrono.ChVectorD(0, 0, 0), 3, 4, 10, 40, 512)

# Enable shadows (commented out to improve performance)
# vis.EnableShadows()

# Create sensor manager
manager = sens.ChSensorManager(system)

# Define lidar parameters
update_rate = 10  # Hz
horizontal_samples = 450
vertical_samples = 32
horizontal_fov = chrono.CH_C_PI  # 180 degrees
vertical_fov = chrono.CH_C_PI / 4  # 45 degrees
max_distance = 100  # meters

# Create lidar sensor and attach it to the rover's chassis
lidar = sens.ChLidarSensor(
    rover.GetChassisBody(),  # body to which the sensor is attached
    update_rate,  # update rate in Hz
    chrono.ChFrameD(chrono.ChVectorD(0, 0, 1.5), chrono.QUNIT),  # offset pose
    horizontal_samples,  # horizontal samples
    vertical_samples,  # vertical samples
    horizontal_fov,  # horizontal field of view
    vertical_fov,  # vertical field of view
    max_distance  # max distance
)

# Set up filters for the lidar data
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterVisualizePointCloud())

# Add the lidar sensor to the sensor manager
manager.AddSensor(lidar)

# Set the simulation time step
time_step = 1e-3

# Simulation loop
time = 0
while vis.Run():
    time += time_step

    # ask rover to move forward
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