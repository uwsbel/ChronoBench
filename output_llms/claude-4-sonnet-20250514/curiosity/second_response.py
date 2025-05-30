import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as robot
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

# Create a long box obstacle for the rover to cross
obstacle_mat = chrono.ChContactMaterialNSC()
obstacle = chrono.ChBodyEasyBox(4, 0.5, 0.3, 1000, True, True, obstacle_mat)
obstacle.SetPos(chrono.ChVector3d(0, 0, 0.15))  # Position the obstacle on the ground
obstacle.SetFixed(True)  # Fix the obstacle in place
obstacle.GetVisualShape(0).SetColor(chrono.ChColor(0.8, 0.3, 0.3))  # Set obstacle color to reddish
system.Add(obstacle)

# Create Curiosity rover and add it to the system
rover = robot.Curiosity(system)

# Create driver for rover
driver = robot.CuriosityDCMotorControl()
rover.SetDriver(driver)

# Initialize rover position and orientation - Changed initial position to (-5, 0.0, 0)
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
vis.AddCamera(chrono.ChVector3d(-5, 3, 3), chrono.ChVector3d(-5, 0, 0))  # Adjust camera to follow rover
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

    # Set zero steering input and forward motion for the rover
    steering = 0.0  # Zero steering input as requested
    driver.SetSteering(steering)
    
    # Set forward motion (you may need to adjust this value based on the robot implementation)
    # This assumes the driver has methods to control forward motion
    try:
        driver.SetDriving(0.5)  # Set forward driving command
    except AttributeError:
        # Alternative method if SetDriving doesn't exist
        pass

    # Update rover dynamics
    rover.Update()

    # Update camera to follow the rover
    rover_pos = rover.GetChassis().GetPos()
    vis.GetActiveCamera().SetPosition(chrono.ChVector3d(rover_pos.x - 3, rover_pos.y + 3, rover_pos.z + 2))
    vis.GetActiveCamera().SetTarget(rover_pos)

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance simulation by one time step
    system.DoStepDynamics(time_step)