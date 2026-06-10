"""
Viper rover simulation on rigid terrain using PyChrono 9.0.x with Irrlicht.

System type: ChSystemNSC with Bullet collision.
Bodies: flat rigid ground + Viper rover (built-in robot.Viper, 6-wheel suspension).
Driver: ViperDCMotorControl — steering ramps gradually from 0 to max and back.
Expected behavior: rover drives forward while steering gradually changes over time,
executing a gentle curve and returning to straight.
"""

import math
import os
import pychrono.core as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr

# === Named constants ===
TIME_STEP   = 1e-3          # physics time step (s)
SIM_END     = 20.0          # total simulation duration (s)
RENDER_FPS  = 50.0          # review-video frame rate
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

MAX_STEERING = math.pi / 6  # maximum steering angle (rad), ~30 deg
STEER_RAMP_START = 2.0      # time at which steering begins to ramp up (s)
STEER_RAMP_PEAK  = 7.0      # time at which steering reaches maximum (s)
STEER_RAMP_END   = 12.0     # time at which steering returns to zero (s)

GROUND_HALF_THICKNESS = 1.0       # half-thickness of ground box (m)
GROUND_Z_CENTER       = -GROUND_HALF_THICKNESS  # box center so top face is at z=0

ROVER_INIT_X, ROVER_INIT_Y, ROVER_INIT_Z = 0.0, 0.0, 0.2  # spawn above ground

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
ground = chrono.ChBodyEasyBox(20, 20, 2 * GROUND_HALF_THICKNESS, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, GROUND_Z_CENTER))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# === Viper rover + driver ===
rover  = robot.Viper(system)
driver = robot.ViperDCMotorControl()
rover.SetDriver(driver)

init_pos = chrono.ChVector3d(ROVER_INIT_X, ROVER_INIT_Y, ROVER_INIT_Z)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)  # identity (w,x,y,z)
rover.Initialize(chrono.ChFramed(init_pos, init_rot))

chassis_body = rover.GetChassis().GetBody()  # cache: fetched once, reused for camera tracking

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Viper Rover - Rigid Terrain")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2.5, 1.5), chrono.ChVector3d(0, 0, 0.2))
vis.AddTypicalLights()
vis.AddLightWithShadow(
    chrono.ChVector3d(1.5, -2.5, 5.5),
    chrono.ChVector3d(0, 0, 0.5),
    3, 4, 10, 40, 512,
)


# === Main loop ===
frame = 0
try:
    while vis.Run() and system.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            time = system.GetChTime()

            # Compute ramped steering angle
            steering = 0.0
            if STEER_RAMP_START < time < STEER_RAMP_PEAK:
                steering = MAX_STEERING * (time - STEER_RAMP_START) / (STEER_RAMP_PEAK - STEER_RAMP_START)
            elif STEER_RAMP_PEAK <= time <= STEER_RAMP_END:
                steering = MAX_STEERING * (STEER_RAMP_END - time) / (STEER_RAMP_END - STEER_RAMP_PEAK)

            driver.SetSteering(steering)
            rover.Update()


            system.DoStepDynamics(TIME_STEP)
            if system.GetChTime() >= SIM_END:
                break

except (RuntimeError, ValueError) as exc:  # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
