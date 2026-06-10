"""
TurtleBot differential-drive simulation on rigid terrain.

System type: ChSystemNSC with Bullet collision.
Bodies: fixed ground box (rigid terrain), TurtleBot robot (built-in catalog model).
Expected behavior: TurtleBot drives forward, then executes a left turn at t=1 s
(stop left wheel, keep right wheel spinning) and a right turn at t=2 s
(keep left wheel spinning, stop right wheel), illustrating differential steering.
"""

import math
import os
import pychrono.core as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr

# === Named constants ===
TIME_STEP = 2e-3          # TurtleBot canonical timestep (seconds)
SIM_END   = 10.0          # total simulation duration (seconds)
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# Robot spawn position (above ground surface at z=0)
ROBOT_INIT_POS = chrono.ChVector3d(0, 0.2, 0)
ROBOT_INIT_ROT = chrono.ChQuaterniond(1, 0, 0, 0)   # identity (w, x, y, z)

# Wheel IDs for TurtleBot differential drive
LEFT_DRIVE_WHEEL  = 0
RIGHT_DRIVE_WHEEL = 1

# Motor speeds (rad/s) — negative = forward for this model
SPEED_FORWARD = -math.pi
SPEED_STOP    = 0.0

# === System & gravity ===
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# === Ground body (rigid terrain) ===
ground_mat = chrono.ChContactMaterialNSC()
ground_mat.SetFriction(0.8)
ground_mat.SetRestitution(0.0)
# 20×20×1 box, top surface at z=0; centre placed at z=-1 so top face is flush with world z=0
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -1))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# === TurtleBot robot ===
# Pose is passed to the CONSTRUCTOR; Initialize() takes no arguments.
robot_tb = robot.TurtleBot(system, ROBOT_INIT_POS, ROBOT_INIT_ROT)
robot_tb.Initialize()

# Start both wheels spinning forward from t=0
robot_tb.SetMotorSpeed(SPEED_FORWARD, LEFT_DRIVE_WHEEL)
robot_tb.SetMotorSpeed(SPEED_FORWARD, RIGHT_DRIVE_WHEEL)

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("TurtleBot - Differential Drive on Rigid Terrain")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, -2.5, 1.5), chrono.ChVector3d(0, 0, 0))
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
chassis_body = robot_tb.GetChassis().GetBody()  # cache: fetched once, reused every step

try:
    while vis.Run() and system.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            sim_time = system.GetChTime()

            # --- Motor control: differential turning at specified times ---
            if abs(sim_time - 1.0) < TIME_STEP * 0.5:
                # Left turn: stop left wheel, keep right wheel spinning
                robot_tb.SetMotorSpeed(SPEED_STOP,    LEFT_DRIVE_WHEEL)
                robot_tb.SetMotorSpeed(SPEED_FORWARD, RIGHT_DRIVE_WHEEL)
            if abs(sim_time - 2.0) < TIME_STEP * 0.5:
                # Right turn: keep left wheel spinning, stop right wheel
                robot_tb.SetMotorSpeed(SPEED_FORWARD, LEFT_DRIVE_WHEEL)
                robot_tb.SetMotorSpeed(SPEED_STOP,    RIGHT_DRIVE_WHEEL)

            # review-only CSV logging

            system.DoStepDynamics(TIME_STEP)
            if system.GetChTime() >= SIM_END:
                break

except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
