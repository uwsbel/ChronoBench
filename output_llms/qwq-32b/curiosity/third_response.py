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

# Initialize rover position and orientation (fixed ChFrameD typo)
init_pos = chrono.ChVector3d(-5, 0.0, 0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
rover.Initialize(chrono.ChFrameD(init_pos, init_rot))  # Corrected to ChFrameD

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

# Create sensor manager
manager = sens.ChSensorManager(system)

# Add lidar sensor to rover's chassis
chassis = rover.GetChassisBody()
lidar = sens.ChLidarSensor()
lidar.SetName("lidar")
lidar.SetPosition(chrono.ChVectorD(0, 0, 0.5))  # Position relative to chassis
lidar.SetRotation(chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.VECT_Y))  # Rotate to point forward
lidar.SetRange(10.0)
lidar.SetFovHorizontal(math.radians(360))  # Full horizontal FOV
lidar.SetFovVertical(math.radians(30))  # Vertical FOV
lidar.SetSampleSizeHorizontal(360)  # 360 samples for full circle
lidar.SetSampleSizeVertical(1)  # Single row
lidar.SetUpdateRate(0.1)  # Update every 0.1 seconds
lidar.SetNoiseParams(0.0, 0.0)  # No noise
lidar.SetRenderPoints(True)  # Enable rendering
lidar.SetBody(chassis)  # Attach to chassis
manager.AddSensor(lidar)

# Set the simulation time step
time_step = 1e-3

# Simulation loop
time = 0
while vis.Run():
    time += time_step

    # Set driver inputs (added throttle)
    driver.SetSteering(0.0)  # Keep straight
    driver.SetThrottle(1.0)  # Move forward at full throttle

    # Update rover dynamics
    rover.Update()

    # Advance simulation step
    system.DoStepDynamics(time_step)

    # Update sensors
    manager.Update()

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()