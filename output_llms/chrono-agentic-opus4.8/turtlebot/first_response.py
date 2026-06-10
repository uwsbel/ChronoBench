"""TurtleBot differential-drive robot on rigid terrain (PyChrono 9.0.0, NSC).

Models PyChrono's built-in two-wheel differential-drive TurtleBot
(`pychrono.robot.TurtleBot`) standing on a fixed rigid ground box. The robot
owns its chassis / drive wheels / caster / motors; the script commands each
drive wheel's angular speed directly to make the robot pivot left, then right,
at scheduled times. A non-smooth (NSC) system with Bullet collision handles the
wheel-ground contact. Visualization is a real-time Irrlicht window (Z-up).

Expected behavior: the TurtleBot rolls/turns in place — first a left pivot
(right wheel driven), then a right pivot (left wheel driven) — staying on the
ground plane with no fall-through.
"""

import os
import math

import pychrono.core as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr

# === Constants === geometry / timing / control schedule
time_step = 2e-3            # TurtleBot integrates well at 2 ms
sim_end = 6.0              # total simulated seconds
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))           # precomputed once

GROUND_SIZE = 20.0         # square ground extent (m)
GROUND_THICK = 1.0         # ground box thickness (m); top surface at z = -0.5
TURN_SPEED = math.pi       # wheel angular speed magnitude (rad/s)
LEFT_TURN_TIME = 1.0       # start a left pivot at t = 1 s
RIGHT_TURN_TIME = 3.0      # start a right pivot at t = 3 s

LEFT_DRIVE_WHEEL = 0       # WheelID: 0 = LEFT
RIGHT_DRIVE_WHEEL = 1      # WheelID: 1 = RIGHT

init_pos = chrono.ChVector3d(0, 0.2, 0)                 # spawn just above ground top
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)            # identity orientation (w,x,y,z)


# === System & gravity === NSC system + Bullet collision (wheel-ground contact)
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# === Ground === fixed rigid box; top surface at z = -0.5 under the spawn
ground_mat = chrono.ChContactMaterialNSC()
ground_mat.SetFriction(0.9)          # traction so the driven wheels turn the robot, not slip
ground = chrono.ChBodyEasyBox(GROUND_SIZE, GROUND_SIZE, GROUND_THICK, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -GROUND_THICK / 2))   # top at z = -0.5
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# === Robot === built-in TurtleBot (owns its bodies); pose in constructor
robot_tb = robot.TurtleBot(system, init_pos, init_rot)
robot_tb.Initialize()                                   # no-arg for TurtleBot

# === Visualization === full Irrlicht scene: window + sky + camera + lights
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("TurtleBot - Rigid terrain")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1.5, 0.8), chrono.ChVector3d(0, 0, 0.1))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5),
                       3, 4, 10, 40, 512)

# === Main loop === drive wheels on a left- then right-pivot schedule
left_cmd = False
right_cmd = False

try:

    frame = 0
    while vis.Run() and system.GetChTime() < sim_end:
        t = system.GetChTime()

        # schedule: left pivot at t=1s (drive right wheel), right pivot at t=3s (drive left wheel)
        if not left_cmd and t >= LEFT_TURN_TIME:
            robot_tb.SetMotorSpeed(0.0, LEFT_DRIVE_WHEEL)
            robot_tb.SetMotorSpeed(-TURN_SPEED, RIGHT_DRIVE_WHEEL)
            left_cmd = True
        if not right_cmd and t >= RIGHT_TURN_TIME:
            robot_tb.SetMotorSpeed(-TURN_SPEED, LEFT_DRIVE_WHEEL)
            robot_tb.SetMotorSpeed(0.0, RIGHT_DRIVE_WHEEL)
            right_cmd = True

        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(render_every):
            system.DoStepDynamics(time_step)
            if system.GetChTime() >= sim_end:
                break

except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass

# === Post-processing === assemble review video + timeseries, drop frames
