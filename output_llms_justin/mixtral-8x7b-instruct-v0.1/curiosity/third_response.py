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

# Create a long box for rover to cross
box = chrono.ChBodyEasyBox(0.25, 5, 0.25, 1000, True, True, ground_mat)
box.SetPos(chrono.ChVector3d(0, 0, 0.0))
box.SetFixed(True)
box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
system.Add(box)

# Create Curiosity rover and add it to the system
rover = robot.Curiosity(system)

# Create sensor manager
sens_manager = sens.ChSensorManager(system)

# Create Lidar sensor for the rover
lidar_update_rate = 10  # Hz
lidar_horizontal_samples = 360
lidar_vertical_samples = 1
lidar_range = 10  # meters
lidar_fov = 3.14159265359  # Full field of view (pi radians)
lidar_min_angle = -lidar_fov / 2
lidar_max_angle = lidar_fov / 2
lidar_noise = 0.001  # Noise factor
lidar = sens.ChLidarSensor()
lidar.SetUpdateRate(1 / lidar_update_rate)
lidar.SetRange(lidar_range)
lidar.SetHorizontalResolution(lidar_horizontal_samples)
lidar.SetVerticalResolution(lidar_vertical_samples)
lidar.SetMinAngle(lidar_min_angle)
lidar.SetMaxAngle(lidar_max_angle)
lidar.SetNoise(lidar_noise)
lidar.Initialize(rover.GetChassisBody(), sens.ChLidarSensor.Types.LIDAR_TYPE_2D)

# Add filters for the lidar data
lidar_filter = sens.ChFilterRange(0, lidar_range)
lidar_filter2 = sens.ChFilterMean()
lidar.AddFilter(lidar_filter)
lidar.AddFilter(lidar_filter2)

# Attach the lidar sensor to the rover's chassis
lidar.AttachToBody(rover.GetChassisBody())

# Add the lidar sensor to the sensor manager
sens_manager.AddSensor(lidar)

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

# Enable shadows (commented out to improve performance)
# vis.EnableShadows()

# Set the simulation time step
time_step = 1e-3

# Simulation loop
time = 0
while vis.Run():
    time += time_step

    # ask rover to move forward
    driver.SetSteering(0.0)

    # Update sensor manager
    sens_manager.Update()

    # Update rover dynamics
    rover.Update()

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance simulation by one time step
    system.DoStepDynamics(time_step)