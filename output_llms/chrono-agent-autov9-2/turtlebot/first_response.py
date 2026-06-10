"""TurtleBot differential-drive robot on a rigid ground patch (PyChrono 9.0.1, Irrlicht).

Models a Chrono `pychrono.robot.TurtleBot` two-wheel differential-drive robot
standing on a fixed rigid ground body. The system is a ChSystemNSC with Z-up
gravity and a Bullet collision system so the robot's wheels make frictional
contact with the ground. The robot's left and right drive motors are commanded
with a timed sequence of wheel speeds so the robot first drives forward, then
executes a left turn (left wheel slowed) and a right turn (right wheel slowed),
demonstrating differential-drive steering.

Expected behavior: the robot's base translates across the ground and changes
heading during the turn phases; it never tips over or sinks through the floor.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.robot as robot

# === Named constants === geometry / physics / timed motor schedule
TIME_STEP = 1.0e-3            # solver step (s)
SIM_END = 12.0               # total simulated time (s)
RENDER_FPS = 50.0            # review render cadence (frames/s)

GROUND_SIZE = 20.0           # square ground patch full edge length (m)
GROUND_THICK = 1.0           # ground slab thickness (m)
GROUND_FRICTION = 0.8        # tire/ground friction coefficient
GROUND_TOP_Z = 0.0           # ground top surface height (m)

# TurtleBot spawn: drop the base just above the ground so the wheels settle on it.
SPAWN_Z = GROUND_TOP_Z + 0.0          # robot frame origin at ground level; wheels rest on top
INIT_POS = chrono.ChVector3d(0.0, 0.0, SPAWN_Z)
INIT_ROT = chrono.ChQuaterniond(1, 0, 0, 0)   # identity orientation

# Differential-drive wheel ids (bare ints): 0 = left drive wheel, 1 = right drive wheel.
LEFT_WHEEL = 0
RIGHT_WHEEL = 1

DRIVE_SPEED = 6.0            # nominal wheel angular speed (rad/s)
TURN_SLOW = 1.0             # slowed-wheel angular speed during a turn (rad/s)

# Timed schedule (start, end) seconds for each maneuver phase.
T_FORWARD_END = 4.0          # 0..4 s: drive straight
T_LEFT_END = 8.0             # 4..8 s: left turn (left wheel slowed)
# 8..SIM_END: right turn (right wheel slowed)

# === System & gravity === NSC system with Bullet collision (robot contacts ground)
sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

# === Ground body === fixed rigid patch with a contact material + collision shape
ground_mat = chrono.ChContactMaterialNSC()   # NSC material to match ChSystemNSC
ground_mat.SetFriction(GROUND_FRICTION)
ground_mat.SetRestitution(0.0)

ground = chrono.ChBodyEasyBox(
    GROUND_SIZE, GROUND_SIZE, GROUND_THICK,
    1000.0,            # density (unused for a fixed body, kept explicit)
    True,              # visualization shape
    True,              # collision shape
    ground_mat,
)
ground.SetPos(chrono.ChVector3d(0, 0, GROUND_TOP_Z - 0.5 * GROUND_THICK))  # top face at GROUND_TOP_Z
ground.SetFixed(True)
ground.GetVisualShape(0).SetColor(chrono.ChColor(0.55, 0.55, 0.55))
sys.Add(ground)

# === Robot === TurtleBot differential-drive robot built on the shared NSC system
turtlebot = robot.TurtleBot(sys, INIT_POS, INIT_ROT)
turtlebot.Initialize()

# Resolve the base body from the system by name (no public chassis getter).
base_body = None
for body in sys.GetBodies():                  # cache: scanned once before the loop
    if body.GetName() == "chassis_body":
        base_body = body
        break
assert base_body is not None, "TurtleBot base 'chassis_body' not found in system"

# Start driving straight: both wheels at nominal speed.
turtlebot.SetMotorSpeed(DRIVE_SPEED, LEFT_WHEEL)
turtlebot.SetMotorSpeed(DRIVE_SPEED, RIGHT_WHEEL)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)   # Z-up world
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("TurtleBot on rigid ground")
vis.Initialize()                                     # Initialize FIRST (Irrlicht order)
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(2.5, -2.5, 1.8), chrono.ChVector3d(0, 0, 0.2))
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 40, 40,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, GROUND_TOP_Z + 0.001), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))           # ground reference grid

# === Main loop === render-cadence outer loop, physics batch inner; timed motor schedule
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once

phase = "forward"   # tracks the current maneuver to avoid redundant motor commands


frame = 0
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            t = sys.GetChTime()
            # Timed differential-drive schedule: forward, then left turn, then right turn.
            if t < T_FORWARD_END:
                if phase != "forward":
                    turtlebot.SetMotorSpeed(DRIVE_SPEED, LEFT_WHEEL)
                    turtlebot.SetMotorSpeed(DRIVE_SPEED, RIGHT_WHEEL)
                    phase = "forward"
            elif t < T_LEFT_END:
                if phase != "left":
                    turtlebot.SetMotorSpeed(TURN_SLOW, LEFT_WHEEL)    # slow left wheel -> turn left
                    turtlebot.SetMotorSpeed(DRIVE_SPEED, RIGHT_WHEEL)
                    phase = "left"
            else:
                if phase != "right":
                    turtlebot.SetMotorSpeed(DRIVE_SPEED, LEFT_WHEEL)
                    turtlebot.SetMotorSpeed(TURN_SLOW, RIGHT_WHEEL)   # slow right wheel -> turn right
                    phase = "right"
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
