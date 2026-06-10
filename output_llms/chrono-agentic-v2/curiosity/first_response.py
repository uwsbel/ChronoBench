"""
Curiosity rover simulation on rigid terrain using PyChrono 9.0.x + Irrlicht.

System type: ChSystemNSC (rover models use NSC with Bullet collision).
Main bodies: fixed ground box, Curiosity rover (rocker-bogie multi-body model
             built internally by robot.Curiosity).
Driver: CuriosityDCMotorControl with smooth steering ramp.
Expected behavior: Curiosity rover rolls forward on rigid terrain and executes
a smooth steering maneuver — ramps steering left, then returns to straight.
Visualized with Irrlicht: sky box, shadows, customizable camera, logo.
"""

import math
import os
import pychrono as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr

# === Named constants ===
TIME_STEP = 1e-3          # physics step size (s)
SIM_END   = 20.0          # total simulation duration (s)
RENDER_FPS = 50.0         # render frame rate (Hz)
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # steps per frame; precomputed once

# Rover spawn position — Curiosity sits with ground box top at z=0.0 (ground z=-0.5)
ROVER_INIT_POS = chrono.ChVector3d(0.0, 0.0, 0.3)
ROVER_INIT_ROT = chrono.ChQuaterniond(1, 0, 0, 0)  # identity

# Steering schedule constants
STEER_START   = 2.0           # time (s) when steering begins
STEER_FULL    = 7.0           # time (s) when steering reaches maximum
STEER_END     = 12.0          # time (s) when steering returns to zero
MAX_STEERING  = math.pi / 6   # maximum steering angle (rad) — about 30 deg

# Camera eye and target (Z-up)
CAM_EYE    = chrono.ChVector3d(0, -5.0, 2.0)
CAM_TARGET = chrono.ChVector3d(0, 0, 0.5)

# === System & gravity ===
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# === Ground (rigid terrain) ===
ground_mat = chrono.ChContactMaterialNSC()
ground_mat.SetFriction(0.8)
ground_mat.SetRestitution(0.0)

# Ground box: 20×20×1 m, top surface at z=0.0
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.5))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# === Curiosity rover ===
curiosity = robot.Curiosity(system)
driver = robot.CuriosityDCMotorControl()
curiosity.SetDriver(driver)
curiosity.Initialize(chrono.ChFramed(ROVER_INIT_POS, ROVER_INIT_ROT))

# Cache chassis body for sensor/logging use
chassis_body = curiosity.GetChassis().GetBody()  # cache: fetched once, reused each step

# === Visualization (Irrlicht) ===
# Initialize FIRST, then add scene elements (Irrlicht order is INVERSE of VSG)
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Curiosity Rover — Rigid Terrain")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(CAM_EYE, CAM_TARGET)
vis.AddTypicalLights()
vis.AddLightWithShadow(
    chrono.ChVector3d(1.5, -2.5, 5.5),
    chrono.ChVector3d(0, 0, 0.5),
    3, 4, 10, 40, 512,
)
vis.EnableShadows()

# === Review-only setup ===

# === Main loop ===
frame = 0
try:
    while vis.Run() and system.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            sim_time = system.GetChTime()  # cache: read once per sub-step

            # Ramp steering smoothly (sudden jumps destabilize the suspension)
            if sim_time < STEER_START:
                steering = 0.0
            elif sim_time < STEER_FULL:
                steering = MAX_STEERING * (sim_time - STEER_START) / (STEER_FULL - STEER_START)
            elif sim_time < STEER_END:
                steering = MAX_STEERING * (STEER_END - sim_time) / (STEER_END - STEER_FULL)
            else:
                steering = 0.0

            driver.SetSteering(steering)
            curiosity.Update()  # propagate steering to DC motors (REQUIRED every step)


            system.DoStepDynamics(TIME_STEP)
            if system.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:  # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
