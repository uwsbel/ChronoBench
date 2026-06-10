"""
Curiosity Mars rover simulation on rigid terrain using PyChrono.

System type: ChSystemNSC (Non-smooth contact, Bullet collision)
Main bodies:
  - Fixed ground box (rigid terrain with concrete texture)
  - Curiosity rover (built-in robot.Curiosity with CuriosityDCMotorControl driver)

Expected behavior:
  The Curiosity rover navigates forward on the flat rigid terrain while the
  DC-motor driver controls steering via a smooth ramp maneuver. The Irrlicht
  window shows the rover rolling forward with customizable camera, shadows, and
  sky-box lighting.
"""

import os
import math
import pychrono.core as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr

# === Constants ===
TIME_STEP = 1e-3          # physics step size [s]
SIM_END   = 20.0          # simulation duration [s]
RENDER_FPS = 50.0         # review video frame rate [fps]
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# Rover spawn position: z=0 on top of ground surface (ground top at z=0)
ROVER_INIT_POS = chrono.ChVector3d(-5, 0, 0)
ROVER_INIT_ROT = chrono.ChQuaterniond(1, 0, 0, 0)  # identity (w, x, y, z)

# Steering ramp parameters
MAX_STEERING = math.pi / 6   # max steering angle ~30 deg

# === System & gravity ===
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# === Ground (rigid terrain) ===
# Curiosity ground: center at z=-0.5 so the top surface sits at z=0
ground_mat = chrono.ChContactMaterialNSC()
ground_mat.SetFriction(0.8)
ground_mat.SetRestitution(0.0)
ground = chrono.ChBodyEasyBox(40, 40, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.5))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# === Curiosity rover ===
curiosity = robot.Curiosity(system)
driver = robot.CuriosityDCMotorControl()
curiosity.SetDriver(driver)
curiosity.Initialize(chrono.ChFramed(ROVER_INIT_POS, ROVER_INIT_ROT))

chassis_body = curiosity.GetChassis().GetBody()  # cache: fetched once, reused every step

# === Visualization ===
# Irrlicht: Initialize() FIRST, then add scene elements (AFTER)
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Curiosity Rover - Rigid Terrain")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(-8, -6, 3), chrono.ChVector3d(-5, 0, 1))
vis.AddTypicalLights()
vis.AddLightWithShadow(
    chrono.ChVector3d(1.5, -2.5, 5.5),
    chrono.ChVector3d(0, 0, 0.5),
    3, 4, 10, 40, 512,
)

# === Review-only setup ===


# === Main loop ===
frame = 0
try:
    while vis.Run() and system.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            sim_time = system.GetChTime()

            # Smooth steering ramp: straight → turn in → back to straight
            steering = 0.0
            if 2.0 < sim_time < 7.0:
                steering = MAX_STEERING * (sim_time - 2.0) / 5.0   # ramp up
            elif 7.0 < sim_time < 12.0:
                steering = MAX_STEERING * (12.0 - sim_time) / 5.0  # ramp down

            driver.SetSteering(steering)
            curiosity.Update()


            system.DoStepDynamics(TIME_STEP)
            if system.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:  # solver divergence / bad physics state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
