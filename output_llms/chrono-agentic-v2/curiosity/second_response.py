"""
Curiosity Mars rover simulation on rigid terrain with a box obstacle.

System type: ChSystemNSC (NSC contact, Bullet collision).
Main bodies: fixed ground plane (rigid), long box obstacle, Curiosity rover chassis
  + rocker-bogie suspension/wheels (all owned by robot.Curiosity).
Expected behavior: rover spawns at (-5, 0, 0), drives forward with zero steering,
  encounters and crosses the long box obstacle placed in its path.
"""

import math
import os
import pychrono as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr

# === Constants ===
TIME_STEP = 1e-3          # physics step size (s)
SIM_END   = 20.0          # simulation duration (s)
RENDER_FPS = 50.0         # Irrlicht render rate
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# Rover spawn
ROVER_INIT_X = -5.0
ROVER_INIT_Z =  0.0       # ground surface is z=0 for Curiosity

# Obstacle geometry (long box in rover's path)
OBS_LEN   = 5.0   # length along Y (rover drives in +X, obstacle spans Y)
OBS_WIDTH = 0.4   # width along X (obstacle crosses rover path)
OBS_HEIGHT= 0.15  # height above ground
OBS_X     = 0.0   # centered at x=0 (rover drives from x=-5 toward +X)
OBS_Y     = 0.0
OBS_Z     = OBS_HEIGHT / 2.0  # box center so its bottom is at z=0

# Ground
GROUND_SIZE  = 30.0
GROUND_THICK = 1.0

# === System & gravity ===
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# === Bodies ===
# Ground — fixed box, top surface at z=0
ground_mat = chrono.ChContactMaterialNSC()
ground_mat.SetFriction(0.8)
ground_mat.SetRestitution(0.0)
ground = chrono.ChBodyEasyBox(GROUND_SIZE, GROUND_SIZE, GROUND_THICK,
                               1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -GROUND_THICK / 2.0))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# Long box obstacle — fixed, lying across the rover's forward path
obs_mat = chrono.ChContactMaterialNSC()
obs_mat.SetFriction(0.6)
obs_mat.SetRestitution(0.0)
obstacle = chrono.ChBodyEasyBox(OBS_WIDTH, OBS_LEN, OBS_HEIGHT,
                                 800, True, True, obs_mat)
obstacle.SetPos(chrono.ChVector3d(OBS_X, OBS_Y, OBS_Z))
obstacle.SetFixed(True)
obstacle.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
system.Add(obstacle)

# === Rover ===
# Curiosity rover — owns its own bodies; pass system to constructor
rover = robot.Curiosity(system)
driver = robot.CuriosityDCMotorControl()
rover.SetDriver(driver)   # SetDriver BEFORE Initialize
init_pos = chrono.ChVector3d(ROVER_INIT_X, 0.0, ROVER_INIT_Z)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)   # identity (w,x,y,z)
rover.Initialize(chrono.ChFramed(init_pos, init_rot))

chassis_body = rover.GetChassis().GetBody()  # cache: fetched once, reused for logging

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Curiosity Rover - Obstacle Crossing")
vis.Initialize()   # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(-8, -4, 2), chrono.ChVector3d(0, 0, 0.5))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5),
                       chrono.ChVector3d(0, 0, 0.5),
                       3, 4, 10, 40, 512)

# === Review-only setup ===

frame = 0

# === Main loop ===
try:
    while vis.Run() and system.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            # Drive straight — zero steering throughout
            driver.SetSteering(0.0)
            rover.Update()   # propagate steering command to motors each step


            system.DoStepDynamics(TIME_STEP)
            if system.GetChTime() >= SIM_END:
                break

except (RuntimeError, ValueError) as exc:  # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
