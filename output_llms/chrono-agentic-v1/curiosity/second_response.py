"""
Curiosity Mars Rover Simulation — Obstacle Crossing on Rigid Terrain.

System type: ChSystemNSC with Bullet collision.
Main bodies:
  - Rigid ground plane (flat terrain)
  - Long box obstacle placed in the rover's path
  - Curiosity rover (built-in robot.Curiosity with CuriosityDCMotorControl)

Expected behavior:
  The Curiosity rover starts at (-5, 0, 0) and drives forward (zero steering)
  at constant DC-motor drive. It encounters a long box obstacle placed across
  its path and climbs over it, demonstrating the rocker-bogie suspension
  traversal capability.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr

# === Named constants ===
TIME_STEP   = 1e-3          # physics time step (s)
SIM_END     = 20.0          # simulation end time (s)
RENDER_FPS  = 50.0          # review video frame rate
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))        # precomputed once

# Rover spawn
ROVER_X0, ROVER_Y0, ROVER_Z0 = -5.0, 0.0, 0.0   # initial position per prompt

# Ground geometry
GROUND_LEN   = 40.0
GROUND_WIDTH = 10.0
GROUND_THICK = 1.0
GROUND_Z     = -0.5         # Curiosity sits lower; top at z=0

# Obstacle geometry — a long box across the rover's path
OBS_LEN   = 0.2             # depth along X (rover travel direction)
OBS_WIDTH = 4.0             # full width across Y
OBS_HEIGHT = 0.15           # height above ground (z)
OBS_X     = 0.0             # placed at x=0 (rover drives from -5 toward +x)
OBS_Y     = 0.0
OBS_Z     = OBS_HEIGHT / 2.0   # center-height: box bottom at z=0, top at OBS_HEIGHT

# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

# Reduce collision envelope for fine rover contact geometry
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# === Contact material (NSC — matches ChSystemNSC) ===
ground_mat = chrono.ChContactMaterialNSC()
ground_mat.SetFriction(0.8)
ground_mat.SetRestitution(0.01)

# === Ground (rigid flat terrain) ===
ground = chrono.ChBodyEasyBox(
    GROUND_LEN, GROUND_WIDTH, GROUND_THICK,
    1000.0, True, True, ground_mat
)
ground.SetPos(chrono.ChVector3d(0.0, 0.0, GROUND_Z))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(
    chrono.GetChronoDataFile("textures/concrete.jpg")
)
sys.Add(ground)

# === Obstacle — long box across the rover path ===
obs_mat = chrono.ChContactMaterialNSC()
obs_mat.SetFriction(0.7)
obs_mat.SetRestitution(0.01)

obstacle = chrono.ChBodyEasyBox(
    OBS_LEN, OBS_WIDTH, OBS_HEIGHT,
    2000.0, True, True, obs_mat
)
obstacle.SetPos(chrono.ChVector3d(OBS_X, OBS_Y, OBS_Z))
obstacle.SetFixed(True)
obstacle.GetVisualShape(0).SetTexture(
    chrono.GetChronoDataFile("textures/blue.png")
)
sys.Add(obstacle)

# === Curiosity rover ===
curiosity = robot.Curiosity(sys)
driver    = robot.CuriosityDCMotorControl()
curiosity.SetDriver(driver)    # must be before Initialize

init_pos = chrono.ChVector3d(ROVER_X0, ROVER_Y0, ROVER_Z0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)    # identity (w,x,y,z)
curiosity.Initialize(chrono.ChFramed(init_pos, init_rot))

# Cache chassis body for sensor / logging
chassis_body = curiosity.GetChassis().GetBody()   # cache: fetched once, reused every step

# === Visualization (Irrlicht) ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Curiosity Rover — Obstacle Crossing")
vis.Initialize()                # FIRST, then add scene elements
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(
    chrono.ChVector3d(-8.0, -5.0, 3.5),   # eye: behind and to the side
    chrono.ChVector3d(0.0, 0.0, 0.5),     # look-at: near obstacle height
)
vis.AddTypicalLights()
vis.AddLightWithShadow(
    chrono.ChVector3d(2.0, -4.0, 6.0),    # pos
    chrono.ChVector3d(0.0, 0.0, 0.5),     # aim
    3, 4, 14, 40, 512,                    # radius, near, far, angle, resolution
)
vis.AddGrid(
    1.0, 1.0, 40, 20,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    chrono.ChColor(0.4, 0.4, 0.4),
)

# === Review-only setup ===

# CSV writer (review-only)

# === Main loop ===
frame = 0
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(RENDER_EVERY):
            t = sys.GetChTime()

            # Zero steering: drive straight forward at all times
            driver.SetSteering(0.0)
            curiosity.Update()          # propagate steering command into motors


            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break

except (RuntimeError, ValueError) as exc:   # solver divergence / bad physics state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
