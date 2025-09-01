import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as robot
import pychrono.sensor as sens
from pychrono import irrlicht as chronoirr

Create Chrono system
===================

system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type\_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

Create ground body with contact material and add it to the system
=================================================================

ground\_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground\_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.5)) # Position the ground slightly below the origin
ground.SetFixed(True) # Fix the ground in place
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

create a long box for rover to cross
=====================================

box = chrono.ChBodyEasyBox(0.25, 5, 0.25, 1000, True, True, ground\_mat)
box.SetPos(chrono.ChVector3d(0, 0, 0.0))
box.SetFixed(True)
box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
system.Add(box)

Create Curiosity rover and add it to the system
==============================================

rover = robot.Curiosity(system)

Create sensor manager and add it to the system
=============================================

manager = sens.ChSensorManager(system)

Create driver for rover
======================

driver = robot.CuriosityDCMotorControl()
rover.SetDriver(driver)

Create Lidar sensor for the rover
================================

lidar\_update\_rate = 20 # Hz
lidar\_horizontal\_samples = 360
lidar\_vertical\_samples = 1
lidar\_range = 10 # meters
lidar\_fov = math.radians(360)
lidar\_cone\_angle = math.radians(25)
lidar\_cone\_axis = chrono.ChVector3d(0, 0, 1)
lidar\_offset = chrono.ChVector3d(0, 0, 0.25) # 25 cm above the ground

lidar = sens.ChLidarSensor()
lidar.SetUpdateRate(1 / lidar\_update\_rate)
lidar.SetHorizontalResolution(lidar\_horizontal\_samples)
lidar.SetVerticalResolution(lidar\_vertical\_samples)
lidar.SetRange(lidar\_range)
lidar.SetFieldOfView(lidar\_fov)
lidar.SetConeAngle(lidar\_cone\_angle)
lidar.SetConeAxis(lidar\_cone\_axis)
lidar.SetOffset(lidar\_offset)

# Attach the Lidar sensor to the rover's chassis
lidar.AttachToBody(rover.GetChassisBody())

# Set up filters for the lidar data
lidar.SetDataFilter(sens.ChDataFilter_Mean(10))

# Add the lidar sensor to the sensor manager
manager.Add(lidar)

Initialize rover position and orientation
===========================================

init\_pos = chrono.ChVector3d(-5, 0.0, 0)
init\_rot = chrono.ChQuaterniond(1, 0, 0, 0)
rover.Initialize(chrono.ChFramed(init\_pos, init\_rot))

Create the Irrlicht visualization
================================

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir\_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Curiosity rover - Rigid terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo\_pychrono\_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 3), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0), 3, 4, 10, 40, 512)

# Enable shadows (commented out to improve performance)
# vis.EnableShadows()

# Set the simulation time step
time\_step = 1e-3

Simulation loop
==============

time = 0
while vis.Run():
time += time\_step

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
system.DoStepDynamics(time\_step)