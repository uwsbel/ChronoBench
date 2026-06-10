"""TurtleBot differential-drive maneuver on a flat rigid ground.

Models the Chrono TurtleBot mobile robot (pychrono.robot.TurtleBot) driving on a
fixed ground plate. System type: NSC (non-smooth contact) with a Bullet collision
system, since the robot wheels make rolling contact with the ground.

Main bodies:
- A fixed ground box (top surface at z = -0.6) with a contact material.
- The TurtleBot robot, which internally builds its chassis, plates, rods, two
  active drive wheels, and a passive caster wheel.

Behavior / objective: the robot performs a scripted differential-drive timeline.
Both drive wheels spin forward for the first 5 s (drive STRAIGHT), then the robot
turns LEFT for the next 5 s, then turns RIGHT thereafter. Left/right turns are
in-place skid-steer pivots produced by counter-rotating the two wheels. The
expected result: the robot translates forward, then yaws left in place, then yaws
right in place, with clearly visible heading changes.
"""

import math
import os

import pychrono.core as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr

# === Constants === geometry / physics / timeline (no bare literals downstream)
time_step = 1e-3            # NSC integration step (s)
sim_end = 15.0             # total sim time: 5 s straight + 5 s left + 5 s right
render_fps = 50.0          # review render cadence

ground_top_z = -0.6        # ground top surface height (robot wheels rest on this)
ground_size = 20.0         # ground plate side length (m)
ground_thickness = 0.5     # ground plate thickness (m)
ground_center_z = ground_top_z - 0.5 * ground_thickness  # box center so top = ground_top_z

robot_z = 0.0              # robot spawn height above the ground top
drive_speed = 2.0 * math.pi  # forward wheel angular speed magnitude (rad/s)
pivot_speed = 2.0 * math.pi  # counter-rotation speed for in-place turns (rad/s)

phase_straight_end = 5.0   # straight until t = 5 s
phase_left_end = 10.0      # left turn until t = 10 s; right turn afterwards

# render_every: physics steps between rendered frames (precomputed once)
render_every = max(1, round(1.0 / (render_fps * time_step)))

# === System & gravity === NSC system; Bullet collision for wheel/ground contact
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Contact material === shared NSC material for ground (and robot wheels)
ground_mat = chrono.ChContactMaterialNSC()
ground_mat.SetFriction(0.8)        # high friction so wheels grip and drive
ground_mat.SetRestitution(0.0)     # no bounce on contact

# === Bodies === fixed ground plate with collision + a TurtleBot robot
ground = chrono.ChBodyEasyBox(ground_size, ground_size, ground_thickness,
                              1000.0, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, ground_center_z))
ground.SetFixed(True)
sys.Add(ground)

# TurtleBot: builds chassis/plates/rods/wheels internally; spawn just above ground.
robot_pos = chrono.ChVector3d(0, 0, robot_z)
robot_rot = chrono.ChQuaterniond(1, 0, 0, 0)   # identity orientation (facing +X)
turtlebot = robot.TurtleBot(sys, robot_pos, robot_rot)
turtlebot.Initialize()

# Resolve the chassis body by name from the system (no public chassis getter).
# cache: fetched once before the loop, reused for logging every step
base_body = None
for body in sys.GetBodies():
    if "chassis" in body.GetName().lower():
        base_body = body
        break
if base_body is None:
    base_body = ground   # fallback so logging never dereferences None

# Wheel IDs: 0 = left drive wheel, 1 = right drive wheel.
LEFT_WHEEL = robot.LD
RIGHT_WHEEL = robot.RD


# === Differential-drive controller ===
# Set the two active wheel speeds for a given maneuver mode (skid-steer).
def move(mode):
    """Drive STRAIGHT / LEFT / RIGHT by setting the two wheel motor speeds.

    'straight' : both wheels forward at drive_speed.
    'left'     : left wheel reversed, right wheel forward -> yaw left in place.
    'right'    : right wheel reversed, left wheel forward -> yaw right in place.
    Raises ValueError on an unknown mode.
    """
    if mode == "straight":
        turtlebot.SetMotorSpeed(-drive_speed, LEFT_WHEEL)
        turtlebot.SetMotorSpeed(-drive_speed, RIGHT_WHEEL)
    elif mode == "left":
        turtlebot.SetMotorSpeed(pivot_speed, LEFT_WHEEL)
        turtlebot.SetMotorSpeed(-pivot_speed, RIGHT_WHEEL)
    elif mode == "right":
        turtlebot.SetMotorSpeed(-pivot_speed, LEFT_WHEEL)
        turtlebot.SetMotorSpeed(pivot_speed, RIGHT_WHEEL)
    else:
        raise ValueError(f"invalid move mode: {mode!r}")


# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("TurtleBot differential-drive maneuver")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(-0.9, -1.4, 0.45),
              chrono.ChVector3d(0.35, 0.0, ground_top_z + 0.1))
vis.AddTypicalLights()
vis.AddGrid(0.25, 0.25, 32, 32,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, ground_top_z), chrono.QUNIT),
            chrono.ChColor(0.2, 0.2, 0.2))

# === Main loop === scripted straight -> left -> right differential drive
frame = 0
current_mode = None
try:
    move("straight")
    current_mode = "straight"
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            t = sys.GetChTime()
            # Select maneuver for the current phase of the timeline.
            if t < phase_straight_end:
                mode = "straight"
            elif t < phase_left_end:
                mode = "left"
            else:
                mode = "right"
            if mode != current_mode:
                move(mode)
                current_mode = mode
                print(f"t={t:.2f}s: robot now driving {mode}")
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad maneuver mode
    import traceback
    traceback.print_exc()
    raise
