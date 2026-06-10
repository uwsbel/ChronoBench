import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as robot
from pychrono import irrlicht as chronoirr

# --------------------------------------------------------------
# 1. Create Chrono system
# --------------------------------------------------------------
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

# Default collision envelope / margin (still works when called on the class)
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# --------------------------------------------------------------
# 2. Ground body
# --------------------------------------------------------------
ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.5))          # centre of the box is 0.5 below the origin → top surface at z = 0
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# --------------------------------------------------------------
# 3. Long box obstacle for the rover to cross
# --------------------------------------------------------------
obstacle_mat = chrono.ChContactMaterialNSC()
# half‑extents: 2.0 (x) → full length 4, 0.5 (y) → full width 1, 0.1 (z) → full height 0.2
obstacle = chrono.ChBodyEasyBox(2.0, 0.5, 0.1, 1000, True, True, obstacle_mat)
# place it on top of the ground (ground top is at z = 0)
obstacle.SetPos(chrono.ChVector3d(0, 0, 0.1))
obstacle.SetFixed(True)
obstacle.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(obstacle)

# --------------------------------------------------------------
# 4. Curiosity rover
# --------------------------------------------------------------
rover = robot.Curiosity(system)

# --------------------------------------------------------------
# 5. Driver
# --------------------------------------------------------------
driver = robot.CuriosityDCMotorControl()
rover.SetDriver(driver)

# --------------------------------------------------------------
# 6. Initial pose (updated to the required position)
# --------------------------------------------------------------
init_pos = chrono.ChVector3d(-5, 0.0, 0)          # new start position
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)      # identity quaternion
rover.Initialize(chrono.ChFramed(init_pos, init_rot))

# --------------------------------------------------------------
# 7. Irrlicht visualisation
# --------------------------------------------------------------
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
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5),
                       chrono.ChVector3d(0, 0, 0),
                       3, 4, 10, 40, 512)

# Shadows disabled for performance (kept as comment)
# vis.EnableShadows()

# --------------------------------------------------------------
# 8. Simulation parameters
# --------------------------------------------------------------
time_step = 1e-3                # 1 ms timestep
throttle = 0.2                  # constant forward throttle

# --------------------------------------------------------------
# 9. Main loop
# --------------------------------------------------------------
time = 0
while vis.Run():
    time += time_step

    # Zero steering – the rover goes straight ahead
    steering = 0.0
    driver.SetSteering(steering)

    # Constant forward throttle
    driver.SetThrottle(throttle)

    # Update rover dynamics
    rover.Update()

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance the physics
    system.DoStepDynamics(time_step)