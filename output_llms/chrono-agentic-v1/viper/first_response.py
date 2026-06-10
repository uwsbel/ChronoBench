"""
Viper rover on rigid terrain simulation.

System type: ChSystemNSC with Bullet collision.
Bodies: rigid ground box + Viper rover (6-wheel suspension, built-in catalog model).
Driver: ViperDCMotorControl — steering gradually changes over the simulation period,
        ramping from 0 to max_steering and back, while the rover drives forward.
Expected behavior: Viper rover rolls forward on flat concrete terrain while the
                   front wheels steer left then return to straight.
"""

import math
import os
import pychrono.core as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr

# === Constants ===
TIME_STEP = 1e-3          # physics step size (s)
SIM_END = 15.0            # total simulation duration (s)
RENDER_FPS = 50.0         # frames per second for review video
MAX_STEERING = math.pi / 6  # maximum steering angle (rad), ~30 degrees
STEER_START = 2.0         # time to begin steering ramp (s)
STEER_PEAK  = 7.0         # time steering reaches maximum (s)
STEER_END   = 12.0        # time steering returns to zero (s)

# Precomputed render cadence — compute once, reuse in loop
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# === System & gravity ===
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

# Set suggested collision envelope/margin for rover contact geometry
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# === Ground (rigid terrain) ===
# A fixed box so its top surface sits at z=0 under the rover spawn point.
ground_mat = chrono.ChContactMaterialNSC()
ground_mat.SetFriction(0.8)
ground_mat.SetRestitution(0.0)

ground = chrono.ChBodyEasyBox(40, 40, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.5))  # top surface at z=0
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# === Viper rover ===
rover = robot.Viper(system)
driver = robot.ViperDCMotorControl()
rover.SetDriver(driver)   # must be called before Initialize

init_pos = chrono.ChVector3d(0, 0, 0.2)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)  # identity (w,x,y,z)
rover.Initialize(chrono.ChFramed(init_pos, init_rot))

# Cache chassis body — fetched once, reused for any logging or camera tracking
chassis_body = rover.GetChassis().GetBody()  # cache: fetched once, reused every step

# === Visualization ===
# Full Irrlicht block: Initialize() FIRST, then scene elements AFTER.
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Viper Rover - Rigid Terrain")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2.5, 1.5), chrono.ChVector3d(0, 0, 0.3))
vis.AddTypicalLights()
vis.AddLightWithShadow(
    chrono.ChVector3d(1.5, -2.5, 5.5),
    chrono.ChVector3d(0, 0, 0.5),
    3, 4, 10, 40, 512,
)
vis.AddGrid(
    1.0, 1.0, 40, 40,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.001), chrono.QUNIT),
    chrono.ChColor(0.4, 0.4, 0.4),
)

# === Review-only setup ===


# === Main loop ===
frame = 0
try:
    while vis.Run() and system.GetChTime() < SIM_END:
        sim_time = system.GetChTime()  # cache: current sim time for this frame

        # Compute steering ramp — gradual change over specified time period
        if sim_time < STEER_START:
            steering = 0.0
        elif sim_time < STEER_PEAK:
            steering = MAX_STEERING * (sim_time - STEER_START) / (STEER_PEAK - STEER_START)
        elif sim_time < STEER_END:
            steering = MAX_STEERING * (STEER_END - sim_time) / (STEER_END - STEER_PEAK)
        else:
            steering = 0.0

        driver.SetSteering(steering)
        rover.Update()  # propagate steering command into motors — REQUIRED each step

        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(render_every):
            system.DoStepDynamics(TIME_STEP)
            if system.GetChTime() >= SIM_END:
                break

except (RuntimeError, ValueError) as exc:  # solver divergence / bad simulation state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
