"""Curiosity rover on rigid NSC terrain with a fixed box obstacle.

The scene uses PyChrono's built-in Curiosity rover, a non-smooth contact system,
a textured rigid ground plane, and one long fixed obstacle placed in front of the
rover.  The rover starts at (-5, 0, 0), drives forward with zero steering, and is
expected to climb over or traverse the obstacle.
"""

import traceback

import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.robot as robot


# === Constants === define reusable geometry and run parameters once
TIME_STEP = 1e-3
SIM_END = 12.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once
GROUND_SIZE_X = 18.0
GROUND_SIZE_Y = 8.0
GROUND_THICKNESS = 1.0
GROUND_TOP_Z = 0.0
GROUND_CENTER_Z = GROUND_TOP_Z - GROUND_THICKNESS / 2.0
OBSTACLE_LENGTH = 2.6
OBSTACLE_WIDTH = 0.75
OBSTACLE_HEIGHT = 0.35
OBSTACLE_X = -0.7
OBSTACLE_Y = 0.0
OBSTACLE_Z = GROUND_TOP_Z + OBSTACLE_HEIGHT / 2.0
START_POS = chrono.ChVector3d(-5.0, 0.0, 0.0)
START_ROT = chrono.ChQuaterniond(1, 0, 0, 0)


# === System & gravity === NSC rover contact with Bullet collision
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)


# === Terrain and obstacle === fixed contact bodies support and challenge the rover
ground_mat = chrono.ChContactMaterialNSC()
ground_mat.SetFriction(0.8)
ground_mat.SetRestitution(0.0)

ground = chrono.ChBodyEasyBox(GROUND_SIZE_X, GROUND_SIZE_Y, GROUND_THICKNESS, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, GROUND_CENTER_Z))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

obstacle_mat = chrono.ChContactMaterialNSC()
obstacle_mat.SetFriction(0.9)
obstacle_mat.SetRestitution(0.0)

obstacle = chrono.ChBodyEasyBox(OBSTACLE_LENGTH, OBSTACLE_WIDTH, OBSTACLE_HEIGHT, 1000, True, True, obstacle_mat)
obstacle.SetPos(chrono.ChVector3d(OBSTACLE_X, OBSTACLE_Y, OBSTACLE_Z))
obstacle.SetFixed(True)
obstacle.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))
system.Add(obstacle)


# === Rover === built-in Curiosity with DC motor steering driver
rover = robot.Curiosity(system)
driver = robot.CuriosityDCMotorControl()
rover.SetDriver(driver)
rover.Initialize(chrono.ChFramed(START_POS, START_ROT))
chassis_body = rover.GetChassis().GetBody()  # cache: fetched once, reused for logging


# === Visualization === Irrlicht initialized before scene elements are added
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Curiosity rover obstacle crossing")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(-7.0, -5.0, 3.0), chrono.ChVector3d(-1.0, 0.0, 0.5))
vis.AddTypicalLights()
vis.AddLightWithShadow(
    chrono.ChVector3d(-2.0, -4.0, 6.0),
    chrono.ChVector3d(-1.0, 0.0, 0.4),
    3,
    4,
    12,
    40,
    512,
)


# === Main loop === render, command straight steering, update rover, and step dynamics
try:
    while vis.Run() and system.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            driver.SetSteering(0.0)
            rover.Update()
            system.DoStepDynamics(TIME_STEP)
            if system.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:  # solver divergence / invalid state
    traceback.print_exc()
    raise
except (OSError, IOError) as exc:  # disk or frame-write failure
    traceback.print_exc()
    raise
finally:
    pass
