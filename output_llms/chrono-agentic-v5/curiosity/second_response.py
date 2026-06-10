"""Curiosity Mars rover crossing a long box obstacle on rigid terrain.

System type: ChSystemNSC (non-smooth, Bullet collision) — the tuning the
built-in rocker-bogie Curiosity rover expects.

Main bodies:
  - Curiosity rover (chassis + rocker-bogie suspension + six driven wheels),
    owned and built internally by robot.Curiosity.
  - A fixed rigid ground box (concrete texture).
  - A long, low, fixed box obstacle laid across the rover's forward path.

Expected behavior: the rover spawns at (-5, 0, 0) with identity orientation and
drives straight forward (zero steering) under its DC-motor drive, climbing over
and crossing the long box obstacle without falling through the terrain.
"""

import os
import math

import pychrono as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr


# === Parameters === geometry / physics constants (no bare literals downstream)
time_step = 1e-3                       # rover integration step (Curiosity uses 1e-3)
sim_end = 14.0                         # long enough to roll from -5 over the obstacle

rover_start = chrono.ChVector3d(-5.0, 0.0, 0.0)   # new spawn position
ground_z = -0.5                                    # Curiosity ground top sits at z=0

obstacle_len = 4.0                     # long box obstacle: spans across the path (X)
obstacle_wid = 1.5                     # wide enough to stay under both wheel tracks
obstacle_hgt = 0.12                    # low ridge the rover climbs over
obstacle_pos = chrono.ChVector3d(0.0, 0.0, ground_z + 0.5 + obstacle_hgt / 2.0)


# === System & gravity === NSC + Bullet collision (wheel/terrain/obstacle contact)
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# === Ground & obstacle === fixed rigid terrain plus a long box to cross
ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(30, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, ground_z))   # 1 m thick box -> top at z=0
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

obstacle_mat = chrono.ChContactMaterialNSC()
obstacle = chrono.ChBodyEasyBox(obstacle_len, obstacle_wid, obstacle_hgt,
                                1000, True, True, obstacle_mat)
obstacle.SetPos(obstacle_pos)
obstacle.SetFixed(True)
obstacle.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
system.Add(obstacle)

# === Rover === built-in Curiosity, system-owned bodies, DC-motor steering driver
rover = robot.Curiosity(system)
driver = robot.CuriosityDCMotorControl()
rover.SetDriver(driver)                  # SetDriver BEFORE Initialize
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
rover.Initialize(chrono.ChFramed(rover_start, init_rot))
chassis_body = rover.GetChassis().GetBody()   # cache: chassis handle, reused for logging

# === Visualization === full Irrlicht scene (Z-up): window + sky + camera + lights
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Curiosity rover - crossing a box obstacle")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(-5, 3.5, 2.0), chrono.ChVector3d(0, 0, 0.5))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5),
                       3, 4, 10, 40, 512)

# === Main loop === drive straight (zero steering) and cross the obstacle

frame = 0
os.makedirs("cam", exist_ok=True)                                      # guard output dir
try:
    while vis.Run() and system.GetChTime() < sim_end:
        # straight-ahead motion: zero steering, DC drive is always on
        driver.SetSteering(0.0)
        rover.Update()                # REQUIRED: propagate command into the motors

        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(time_step)
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid state
    import traceback
    traceback.print_exc()
    raise

# === Post-processing === assemble review video + timeseries (record run only)
