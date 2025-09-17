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
system.SetGravitationalAcceleration(chrono.ChVectorD(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# Create ground body with contact material and add it to the system
ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, chrono.ChMaterialSurfaceBase.ContactMethod.NSC, ground_mat)
ground.SetPos(chrono.ChVectorD(0, 0, -0.5))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# Create a long box for the rover to cross
box = chrono.ChBodyEasyBox(0.25, 5, 0.25, 1000, True, True, chrono.ChMaterialSurfaceBase.ContactMethod.NSC, ground_mat)
box.SetPos(chrono.ChVectorD(0, 0, 0.0))
box.SetFixed(True)
box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
system.Add(box)

# Create Curiosity rover and add it to the system
rover = robot.Curiosity(system)

# Create driver for rover
driver = robot.CuriosityDCMotorControl()
rover.SetDriver(driver)

# Initialize rover position and orientation (fixed ChFrameD typo)
init_pos = chrono.ChVectorD(-5, 0.0, 0)
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)
rover.Initialize(chrono.ChFrameD(init_pos, init_rot))

# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Curiosity rover - Rigid terrain with Lidar')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 3, 3), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(1.5, -2.5, 5.5), chrono.ChVectorD(0, 0, 0), 3, 4, 10, 40, 512)

# Create sensor manager
sensor_manager = sens.ChSensorManager(system)

# Add lidar sensor to rover's chassis
chassis = rover.GetChassis()
lidar = sens.ChLidarSensor()
lidar.SetName("lidar")
lidar.SetPosition(chrono.ChFrameD(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT))  # Position on top of chassis
lidar.SetUpdatePeriod(0.1)  # 10 Hz update rate
lidar.SetHorizontalResolution(360)  # 360 horizontal samples (full circle)
lidar.SetVerticalResolution(1)     # 1 vertical sample (single plane)
lidar.SetHorizontalFOV(360)        # Full horizontal field of view
lidar.SetVerticalFOV(20)           # 20-degree vertical field of view
lidar.SetMinRange(0.1)
lidar.SetMaxRange(10.0)
lidar.SetNoiseStandardDeviation(0.01)
lidar.AddFilter(sens.ChLidarRangeClipFilter(0.1, 10.0))
lidar.AttachTo(chassis)
sensor_manager.AddSensor(lidar)

# Simulation loop
time_step = 1e-3
time = 0
while vis.Run():
    time += time_step

    # Control rover (steering set to 0, but may need throttle adjustment)
    driver.SetSteering(0.0)

    # Update rover dynamics
    rover.Update()

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance simulation
    system.DoStepDynamics(time_step)

    # Update sensors
    sensor_manager.Update()

# Cleanup (if needed, though Irrlicht handles it automatically)