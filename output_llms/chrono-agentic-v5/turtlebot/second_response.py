"""TurtleBot differential-drive rover on rigid terrain (PyChrono 9.0.0, NSC).

Models PyChrono's built-in two-wheel differential-drive ``robot.TurtleBot`` on a
fixed rigid ground box whose top surface sits just below the robot spawn. The
bot is system-owned (the class builds its own chassis, wheels, casters and motors
when passed the system); we command each drive wheel's motor speed directly.

A ``move(mode)`` helper sets the two wheel speeds for three motion modes:
  - ``"straight"`` : both wheels forward (drive ahead)
  - ``"left"``     : left wheel slow, right wheel fast (curve left)
  - ``"right"``    : left wheel fast, right wheel slow (curve right)
It raises ``ValueError`` for any unknown mode.

Expected behavior: the bot drives straight for the first 5 s, curves left for the
next 5 s, then curves right for the remainder, printing its current action on each
mode change. System type: ChSystemNSC with Bullet collision (wheel-ground contact).
"""

import os
import math
import pychrono as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr

# === Named constants === geometry / timing / control (no bare literals downstream)
time_step = 2e-3                 # TurtleBot uses 2e-3 (rover skill)
sim_end = 15.0                   # 5 s straight + 5 s left + 5 s right
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))            # precomputed once

ground_z = -0.6                  # ground body center; 1 m box -> top at -0.1
straight_speed = -math.pi        # rad/s drive-forward wheel speed
turn_fast = -math.pi             # outer wheel during a turn
turn_slow = -math.pi / 3.0       # inner wheel during a turn

LEFT_DRIVE_WHEEL = 0             # WheelID 0 = LEFT
RIGHT_DRIVE_WHEEL = 1            # WheelID 1 = RIGHT

t_straight_end = 5.0             # drive straight until here
t_left_end = 10.0                # curve left until here, then curve right


# === System & gravity === NSC + Bullet collision (wheel <-> ground contact)
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))     # Z-up world
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# === Ground === fixed rigid box, top surface just under the bot spawn
ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, ground_z))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# === Robot === built-in TurtleBot (pose in constructor; Initialize() no-arg)
init_pos = chrono.ChVector3d(0, 0.2, 0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                             # identity (w,x,y,z)
robot_tb = robot.TurtleBot(system, init_pos, init_rot)
robot_tb.Initialize()


# === Control === differential-drive mode -> per-wheel motor speeds
def move(mode):
    """Set the two drive-wheel motor speeds for the requested motion mode."""
    if mode == "straight":
        robot_tb.SetMotorSpeed(straight_speed, LEFT_DRIVE_WHEEL)
        robot_tb.SetMotorSpeed(straight_speed, RIGHT_DRIVE_WHEEL)
    elif mode == "left":
        robot_tb.SetMotorSpeed(turn_slow, LEFT_DRIVE_WHEEL)
        robot_tb.SetMotorSpeed(turn_fast, RIGHT_DRIVE_WHEEL)
    elif mode == "right":
        robot_tb.SetMotorSpeed(turn_fast, LEFT_DRIVE_WHEEL)
        robot_tb.SetMotorSpeed(turn_slow, RIGHT_DRIVE_WHEEL)
    else:
        raise ValueError("invalid move mode: %r" % mode)


# === Visualization === full Irrlicht scene: window + sky + camera + lights + shadow
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("TurtleBot - rigid terrain")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2.5, 1.5), chrono.ChVector3d(0, 0, 1))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5),
                       3, 4, 10, 40, 512)

# === Main loop === drive straight, then curve left, then curve right

current_mode = None
frame = 0
try:
    while vis.Run() and system.GetChTime() < sim_end:
        t = system.GetChTime()

        # pick the motion mode for this phase and (re)issue speeds on change
        if t < t_straight_end:
            mode = "straight"
        elif t < t_left_end:
            mode = "left"
        else:
            mode = "right"
        if mode != current_mode:
            print("TurtleBot action:", mode)
            move(mode)
            current_mode = mode

        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            system.DoStepDynamics(time_step)
            if system.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:      # solver divergence / bad control state
    import traceback
    traceback.print_exc()
    raise
