"""Curiosity rover on rigid terrain crossing a fixed long box obstacle.

This PyChrono NSC simulation uses the built-in Curiosity rover model with Bullet
collision, a textured rigid ground plane, and a low transverse rectangular
obstacle. The rover starts at (-5, 0, 0), keeps zero steering input, and drives
forward over the obstacle under its built-in DC motor control.
"""

import math

import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.robot as robot


# === Constants: geometry and timing ===
# Named values make the rover start, obstacle placement, and timing explicit.
TIME_STEP = 1.0e-3
SIM_END = 9.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

GROUND_SIZE_X = 24.0
GROUND_SIZE_Y = 10.0
GROUND_THICKNESS = 1.0
GROUND_CENTER_Z = -0.5

OBSTACLE_SIZE_X = 0.80
OBSTACLE_SIZE_Y = 5.00
OBSTACLE_SIZE_Z = 0.08
OBSTACLE_CENTER_X = -2.4
OBSTACLE_CENTER_Y = 0.0
OBSTACLE_CENTER_Z = OBSTACLE_SIZE_Z / 2.0

ROVER_START = chrono.ChVector3d(-5.0, 0.0, 0.0)
ROVER_ROTATION = chrono.QUNIT
STEERING_INPUT = 0.0
MAX_SOLVER_ITERATIONS = 80


# === System and collision: NSC rover world with Bullet contact ===
# Curiosity uses a system-owned NSC setup; Bullet collision is required for ground contact.
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
system.SetSolverType(chrono.ChSolver.Type_PSOR)
system.GetSolver().AsIterative().SetMaxIterations(MAX_SOLVER_ITERATIONS)
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)


# === Terrain and obstacle: fixed collision bodies for rover traversal ===
# The ground top is at z=0 and the long obstacle is a low box across the driving path.
contact_mat = chrono.ChContactMaterialNSC()
contact_mat.SetFriction(0.85)
contact_mat.SetRestitution(0.0)

ground = chrono.ChBodyEasyBox(
    GROUND_SIZE_X,
    GROUND_SIZE_Y,
    GROUND_THICKNESS,
    1000.0,
    True,
    True,
    contact_mat,
)
ground.SetName("rigid_ground")
ground.SetFixed(True)
ground.SetPos(chrono.ChVector3d(0.0, 0.0, GROUND_CENTER_Z))
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

obstacle = chrono.ChBodyEasyBox(
    OBSTACLE_SIZE_X,
    OBSTACLE_SIZE_Y,
    OBSTACLE_SIZE_Z,
    1000.0,
    True,
    True,
    contact_mat,
)
obstacle.SetName("long_box_obstacle")
obstacle.SetFixed(True)
obstacle.SetPos(chrono.ChVector3d(OBSTACLE_CENTER_X, OBSTACLE_CENTER_Y, OBSTACLE_CENTER_Z))
obstacle.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))
system.Add(obstacle)


# === Curiosity rover: built-in robot model with straight steering ===
# The rover driver owns DC motor steering; zero steering keeps the rover moving forward.
rover = robot.Curiosity(system)
driver = robot.CuriosityDCMotorControl()
rover.SetDriver(driver)
rover.Initialize(chrono.ChFramed(ROVER_START, ROVER_ROTATION))

chassis_body = rover.GetChassis().GetBody()  # cache: reused for camera tracking and logging


# === Visualization: Irrlicht window configured after initialization ===
# The camera views the whole crossing corridor with Z-up controls and standard lighting.
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Curiosity rover crossing a long box obstacle")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(-5.0, -5.5, 2.4), chrono.ChVector3d(-2.6, 0.0, 0.5))
vis.AddTypicalLights()
vis.AddLightWithShadow(
    chrono.ChVector3d(1.5, -2.5, 5.5),
    chrono.ChVector3d(0.0, 0.0, 0.5),
    3,
    4,
    10,
    40,
    512,
)
vis.AddGrid(
    1.0,
    1.0,
    24,
    10,
    chrono.ChCoordsysd(chrono.ChVector3d(0.0, 0.0, 0.003), chrono.QUNIT),
    chrono.ChColor(0.35, 0.35, 0.35),
)


# === Main loop: straight rover command, render, update, and step ===
# The inner physics batch keeps the rover simulation efficient while remaining self-contained.
try:
    while vis.Run() and system.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(RENDER_EVERY):
            driver.SetSteering(STEERING_INPUT)
            rover.Update()
            system.DoStepDynamics(TIME_STEP)
            if system.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:
    print(f"Simulation failed during rover stepping: {exc}")
    raise
finally:
    pass


# === Post-processing: assemble review artifacts only ===
