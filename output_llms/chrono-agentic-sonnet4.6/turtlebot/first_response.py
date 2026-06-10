"""
TurtleBot differential-drive robot on rigid terrain.

System type: ChSystemNSC with Bullet collision.
Main bodies: flat rigid ground (20x20x1 m box, top surface at z=-0.5),
TurtleBot chassis + two drive wheels (built by robot.TurtleBot).
Expected behavior: robot starts stationary, at t=1s begins a left turn (left
wheel stops, right wheel drives at -pi rad/s), at t=2s switches to a right
turn (left wheel drives, right wheel stops), producing visible differential-
drive turning maneuvers on flat concrete terrain.
"""

import math
import os
import csv
import pychrono.core as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr

# === Constants ===
TIME_STEP    = 2e-3    # TurtleBot recommended timestep
SIM_END      = 10.0
RENDER_FPS   = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

LEFT_WHEEL  = 0   # WheelID: 0 = LEFT drive wheel
RIGHT_WHEEL = 1   # WheelID: 1 = RIGHT drive wheel

WHEEL_SPEED_FWD  = -math.pi   # rad/s forward (negative convention)
WHEEL_SPEED_STOP = 0.0

# Robot spawn position — z=-0.45 places wheels just above ground top (ground top at z=-0.5)
INIT_POS = chrono.ChVector3d(0, 0, -0.45)
INIT_ROT = chrono.ChQuaterniond(1, 0, 0, 0)   # identity (w, x, y, z)

CSV_PATH = "simulation_data.csv"

# === System & gravity ===
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# === Ground body ===
ground_mat = chrono.ChContactMaterialNSC()
ground_mat.SetFriction(0.8)
ground_mat.SetRestitution(0.0)
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -1))   # 1 m thick box, center at z=-1, top at z=-0.5
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# === TurtleBot robot ===
# Pose passed to constructor; Initialize() takes no argument; no driver; no Update()
robot_tb = robot.TurtleBot(system, INIT_POS, INIT_ROT)
robot_tb.Initialize()

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("TurtleBot Robot - Rigid Terrain")
vis.Initialize()                          # Initialize FIRST (Irrlicht call order)
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1.5, 0.2), chrono.ChVector3d(0, 0, 0.2))
vis.AddTypicalLights()
vis.AddLightWithShadow(
    chrono.ChVector3d(1.5, -2.5, 5.5),
    chrono.ChVector3d(0, 0, 0.5),
    3, 4, 10, 40, 512,
)

# === Review-only recording setup ===

# === Main loop ===
frame = 0
time = 0.0

try:

    while vis.Run() and system.GetChTime() < SIM_END:
        # Motor commands triggered at specific time instants (matches reference demo)
        if abs(time - 1.0) < 1e-4:
            robot_tb.SetMotorSpeed(WHEEL_SPEED_STOP, LEFT_WHEEL)   # pivot left turn
            robot_tb.SetMotorSpeed(WHEEL_SPEED_FWD,  RIGHT_WHEEL)
        if abs(time - 2.0) < 1e-4:
            robot_tb.SetMotorSpeed(WHEEL_SPEED_FWD,  LEFT_WHEEL)   # pivot right turn
            robot_tb.SetMotorSpeed(WHEEL_SPEED_STOP, RIGHT_WHEEL)

        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            time += TIME_STEP
            system.DoStepDynamics(TIME_STEP)
            if system.GetChTime() >= SIM_END:
                break

except (RuntimeError, ValueError) as exc:   # solver divergence / bad physics state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass   # ensure finally block is non-empty after strip
