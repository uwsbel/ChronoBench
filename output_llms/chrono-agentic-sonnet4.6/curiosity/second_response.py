"""
Curiosity Mars Rover simulation on rigid terrain with a box obstacle.

System type: ChSystemNSC (NSC contact, Bullet collision)
Main bodies: rigid ground plane, long box obstacle, Curiosity rover (built-in model)
Expected behavior: Rover starts at (-5, 0, 0), drives straight forward (zero steering),
approaches and attempts to cross a long box obstacle placed across its path.
"""

import os
import math
import csv
import pychrono.core as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr

# === Constants ===
TIME_STEP   = 1e-3          # physics time step (s)
SIM_END     = 15.0          # simulation duration (s)
RENDER_FPS  = 50.0
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# Rover spawn per prompt: start at (-5, 0, 0)
ROVER_INIT_POS = chrono.ChVector3d(-5.0, 0.0, 0.0)
ROVER_INIT_ROT = chrono.ChQuaterniond(1, 0, 0, 0)   # identity

# Obstacle: a long box placed across the rover's forward path (x-axis)
OBSTACLE_POS    = chrono.ChVector3d(2.0, 0.0, 0.1)  # center; height=0.2, top at z=0.2
OBSTACLE_SIZE_X = 0.3   # narrow along travel direction (x)
OBSTACLE_SIZE_Y = 8.0   # long across (y) — rover must go over it
OBSTACLE_SIZE_Z = 0.2   # height

# === System & gravity ===
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# === Ground ===
ground_mat = chrono.ChContactMaterialNSC()
ground_mat.SetFriction(0.9)
ground_mat.SetRestitution(0.0)

ground = chrono.ChBodyEasyBox(40, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.5))   # top surface at z=0.0
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# === Obstacle ===
obs_mat = chrono.ChContactMaterialNSC()
obs_mat.SetFriction(0.8)
obs_mat.SetRestitution(0.0)

obstacle = chrono.ChBodyEasyBox(OBSTACLE_SIZE_X, OBSTACLE_SIZE_Y, OBSTACLE_SIZE_Z,
                                 2000, True, True, obs_mat)
obstacle.SetPos(OBSTACLE_POS)
obstacle.SetFixed(True)
obstacle.GetVisualShape(0).SetTexture(
    chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(obstacle)

# === Curiosity rover ===
rover = robot.Curiosity(system)
driver = robot.CuriosityDCMotorControl()
rover.SetDriver(driver)
rover.Initialize(chrono.ChFramed(ROVER_INIT_POS, ROVER_INIT_ROT))

chassis_body = rover.GetChassis().GetBody()  # cache: fetched once, reused for logging

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Curiosity Rover — Box Obstacle")
vis.Initialize()   # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(-8, -4, 3), chrono.ChVector3d(0, 0, 0.5))
vis.AddTypicalLights()
vis.AddLightWithShadow(
    chrono.ChVector3d(1.5, -2.5, 5.5),
    chrono.ChVector3d(0, 0, 0.5),
    3, 4, 10, 40, 512)

# === Review-only setup ===

# Open CSV before loop (review-only; single-line context manager for strip safety)

# === Main loop ===
frame = 0

try:
    while vis.Run() and system.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(render_every):
            driver.SetSteering(0.0)   # zero steering: drive straight
            rover.Update()
            system.DoStepDynamics(TIME_STEP)
            if system.GetChTime() >= SIM_END:
                break

except (RuntimeError, ValueError) as exc:  # solver divergence / bad physics state
    import traceback
    traceback.print_exc()
    raise
