"""
TurtleBot differential-drive simulation on rigid terrain.

System type : ChSystemNSC (NSC contact, Bullet collision).
Robot       : robot.TurtleBot — 2-wheel differential-drive built-in model.
Ground      : fixed ChBodyEasyBox terrain (Z-up, top surface at z=0).
Behaviour   : robot moves forward; at t=1 s pivots left (left wheel stopped,
              right wheel spinning); at t=2 s pivots right (left wheel spinning,
              right wheel stopped); continues for the remainder of the run.

No URDF, no RL policy — this is the built-in catalog TurtleBot.
"""

# === Imports ===
import math
import os
import pychrono.core as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr

# === Constants ===
TIME_STEP   = 2e-3          # TurtleBot nominal step (s)
SIM_END     = 10.0          # total simulation duration (s)
RENDER_FPS  = 50.0          # frames per second for review video
# precomputed once
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))

# TurtleBot wheel IDs (per robot.TurtleBot API)
LEFT_WHEEL  = 0
RIGHT_WHEEL = 1

# Motor speed magnitudes (rad/s) — negative = forward in TurtleBot convention
FWD_SPEED   = -math.pi    # forward drive speed
STOP_SPEED  = 0.0         # stopped wheel

# Spawn pose (Z-up; TurtleBot chassis sits above z=0 ground surface)
SPAWN_POS = chrono.ChVector3d(0.0, 0.0, 0.2)
SPAWN_ROT = chrono.ChQuaterniond(1, 0, 0, 0)   # identity (w, x, y, z)

# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))   # Z-down

# Fine collision envelope/margin recommended for small rover bodies
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# === Ground (rigid terrain) ===
ground_mat = chrono.ChContactMaterialNSC()
ground_mat.SetFriction(0.8)
ground_mat.SetRestitution(0.01)

# 20 x 20 m box, 1 m thick; top surface at z = 0 with bottom at z = -1
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.5))   # top at z = 0
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(
    chrono.GetChronoDataFile("textures/concrete.jpg")
)
sys.Add(ground)

# === TurtleBot robot ===
# Pose passed to the constructor (NOT to Initialize).
# Initialize() takes no argument for TurtleBot.
robot_tb = robot.TurtleBot(sys, SPAWN_POS, SPAWN_ROT)
robot_tb.Initialize()

# cache: chassis body fetched once by name from system; used for CSV logging
chassis_body = sys.SearchBody("chassis_body")   # cache: fetched once, reused every step

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("TurtleBot — Differential Drive on Rigid Terrain")
vis.Initialize()                          # Initialize FIRST (Irrlicht call order)
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, -3.0, 1.5), chrono.ChVector3d(0, 0, 0.2))
vis.AddTypicalLights()
vis.AddLightWithShadow(
    chrono.ChVector3d(1.5, -2.5, 5.5),   # light position
    chrono.ChVector3d(0, 0, 0.5),         # aim point
    3, 4, 10, 40, 512,                    # radius, near, far, angle, resolution
)
vis.AddGrid(
    0.5, 0.5, 40, 40,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    chrono.ChColor(0.4, 0.4, 0.4),
)

# === Review-only setup ===


# Track current motor commands (for CSV logging)
left_cmd  = STOP_SPEED   # motors start stopped; first forward command applied below
right_cmd = STOP_SPEED

# === Main loop ===
# TurtleBot: command motors with SetMotorSpeed(rad_s, wheel_id); no Update() needed.
# Motor commands change at t=0 (forward), t=1 s (left turn), t=2 s (right turn).
frame = 0

try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(RENDER_EVERY):
            t = sys.GetChTime()

            # --- Motor command schedule ---
            # t < 1 s  : drive forward (both wheels at FWD_SPEED)
            # 1 ≤ t < 2: left turn (left wheel stopped, right spins)
            # t ≥ 2 s  : right turn (right wheel stopped, left spins)
            if t < 1.0:
                new_left  = FWD_SPEED
                new_right = FWD_SPEED
            elif t < 2.0:
                new_left  = STOP_SPEED
                new_right = FWD_SPEED
            else:
                new_left  = FWD_SPEED
                new_right = STOP_SPEED

            # Apply motor commands when they change (avoids redundant API calls)
            if new_left != left_cmd:
                robot_tb.SetMotorSpeed(new_left, LEFT_WHEEL)
                left_cmd = new_left
            if new_right != right_cmd:
                robot_tb.SetMotorSpeed(new_right, RIGHT_WHEEL)
                right_cmd = new_right


            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break

except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise

finally:
    pass

# === Post-processing ===
