import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as robot
from pychrono import irrlicht as chronoirr

# -------------------------------
# 1. Create Chrono system
# -------------------------------
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))  # Corrected method name and vector type

chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# -------------------------------
# 2. Create ground body
# -------------------------------
ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVectorD(0, 0, -0.5))  # Corrected vector type
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# -------------------------------
# 3. Add a long box obstacle
# -------------------------------
# Example: a box 2m long, 0.5m wide, 0.3m tall, centered at (0, 0, 0.15)
obstacle_mat = chrono.ChContactMaterialNSC()
obstacle = chrono.ChBodyEasyBox(2.0, 0.5, 0.3, 1000, True, True, obstacle_mat)
obstacle.SetPos(chrono.ChVectorD(0, 0, 0.15))  # Sits on the ground
obstacle.SetFixed(True)
obstacle.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))
system.Add(obstacle)

# -------------------------------
# 4. Create Curiosity rover
# -------------------------------
rover = robot.Curiosity(system)

# -------------------------------
# 5. Create driver for rover
# -------------------------------
driver = robot.CuriosityDCMotorControl()
rover.SetDriver(driver)

# -------------------------------
# 6. Initialize rover position and orientation
# -------------------------------
init_pos = chrono.ChVectorD(-5, 0.0, 0)  # Changed as per instruction
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)
rover.Initialize(chrono.ChFrameD(init_pos, init_rot))  # Corrected class name

# -------------------------------
# 7. Create the Irrlicht visualization
# -------------------------------
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Curiosity rover - Rigid terrain with obstacle')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 3, 3), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(1.5, -2.5, 5.5), chrono.ChVectorD(0, 0, 0), 3, 4, 10, 40, 512)
# vis.EnableShadows()  # Optional

# -------------------------------
# 8. Set the simulation time step
# -------------------------------
time_step = 1e-3

# -------------------------------
# 9. Simulation loop
# -------------------------------
time = 0
# Set constant throttle for forward motion, zero steering
driver.SetThrottle(0.5)  # 0.5 is moderate forward throttle; adjust as needed
driver.SetSteering(0.0)  # Always zero steering

while vis.Run():
    time += time_step

    # No steering change, always zero
    # driver.SetSteering(0.0)  # Already set above, can be omitted

    # Update rover dynamics
    rover.Update()

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance simulation by one time step
    system.DoStepDynamics(time_step)