"""
Viper Rover on Rigid Terrain — PyChrono 9.0.x (Irrlicht)

Models a six-wheel Viper rover driving on a rigid flat ground using the
built-in robot.Viper + robot.ViperDCMotorControl API. The rover starts at
rest, then gradually turns (steering ramped from 0 to max and back) while
the DC-motor drive keeps all wheels rolling forward. System type: ChSystemNSC
with Bullet collision. Expected behavior: rover rolls forward, executes a
smooth left-turn arc, then straightens out over the 20-second simulation.
"""

import math
import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.robot as robot

# === Constants ===
TIME_STEP   = 1e-3          # physics step size (s)
SIM_END     = 20.0          # simulation duration (s)
RENDER_FPS  = 50.0          # frames per second for review capture
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

MAX_STEERING = math.pi / 6  # max steering angle ~0.52 rad (practical Viper limit)
STEER_RAMP_START = 2.0      # time to begin ramping steering (s)
STEER_RAMP_PEAK  = 7.0      # time at peak steering (s)
STEER_RAMP_END   = 12.0     # time to finish ramping back to zero (s)

GROUND_HALF_SIZE = 20.0     # half-extent of ground plane (m)
GROUND_THICK     = 1.0      # ground box thickness (m)
GROUND_Z         = -GROUND_THICK  # ground center Z so top surface is at z=0

ROVER_INIT_POS   = chrono.ChVector3d(0, 0, 0.2)   # spawn above ground surface
ROVER_INIT_ROT   = chrono.ChQuaterniond(1, 0, 0, 0)  # identity quaternion (w,x,y,z)

# === System & Gravity ===
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# === Ground Body ===
ground_mat = chrono.ChContactMaterialNSC()
ground_mat.SetFriction(0.8)
ground_mat.SetRestitution(0.0)

ground = chrono.ChBodyEasyBox(
    2 * GROUND_HALF_SIZE, 2 * GROUND_HALF_SIZE, GROUND_THICK,
    1000, True, True, ground_mat
)
ground.SetPos(chrono.ChVector3d(0, 0, GROUND_Z))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(
    chrono.GetChronoDataFile("textures/concrete.jpg")
)
system.Add(ground)

# === Viper Rover ===
rover  = robot.Viper(system)
driver = robot.ViperDCMotorControl()
rover.SetDriver(driver)                             # SetDriver BEFORE Initialize
rover.Initialize(chrono.ChFramed(ROVER_INIT_POS, ROVER_INIT_ROT))

chassis_body = rover.GetChassis().GetBody()         # cache: fetched once, reused every step

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Viper Rover - Rigid Terrain")
vis.Initialize()                                    # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2.5, 1.5), chrono.ChVector3d(0, 0, 1))  # AFTER Initialize
vis.AddTypicalLights()
vis.AddLightWithShadow(
    chrono.ChVector3d(1.5, -2.5, 5.5),
    chrono.ChVector3d(0, 0, 0.5),
    3, 4, 10, 40, 512
)
vis.AddGrid(
    2.0, 2.0, 20, 20,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    chrono.ChColor(0.4, 0.4, 0.4)
)

# === Review-only setup ===

# === Main Loop ===
try:
    while vis.Run() and system.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(render_every):
            sim_time = system.GetChTime()

            # Ramp steering: 0 → max between t=2..7, max → 0 between t=7..12
            steering = 0.0
            if STEER_RAMP_START < sim_time < STEER_RAMP_PEAK:
                steering = MAX_STEERING * (sim_time - STEER_RAMP_START) / (
                    STEER_RAMP_PEAK - STEER_RAMP_START
                )
            elif STEER_RAMP_PEAK <= sim_time < STEER_RAMP_END:
                steering = MAX_STEERING * (STEER_RAMP_END - sim_time) / (
                    STEER_RAMP_END - STEER_RAMP_PEAK
                )

            driver.SetSteering(steering)
            rover.Update()                          # propagate steering to motors (REQUIRED)


            system.DoStepDynamics(TIME_STEP)
            if system.GetChTime() >= SIM_END:
                break

except (RuntimeError, ValueError) as exc:          # solver divergence / bad rover state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
