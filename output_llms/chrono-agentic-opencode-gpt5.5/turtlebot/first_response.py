"""TurtleBot differential-drive robot on rigid terrain.

This self-contained PyChrono NSC simulation builds a Bullet-contact system with
gravity, a fixed rigid ground body, and the catalog TurtleBot robot. The robot
starts from a specified position and orientation, then its wheel motors command
left and right turns at fixed times while Irrlicht renders the real-time scene.
"""

import math
import traceback
from contextlib import nullcontext

import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.robot as robot


# === Constants === keep simulation parameters explicit and reused consistently
TIME_STEP = 2e-3
SIM_END = 6.0
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once
LEFT_DRIVE_WHEEL = 0
RIGHT_DRIVE_WHEEL = 1
STOP_SPEED = 0.0
TURN_SPEED = -20.0
GROUND_SIZE_X = 20.0
GROUND_SIZE_Y = 20.0
GROUND_THICKNESS = 1.0
GROUND_DENSITY = 1000.0
GROUND_Z = -1.0
INIT_POS = chrono.ChVector3d(0.0, 0.2, 0.0)
INIT_ROT = chrono.ChQuaterniond(1.0, 0.0, 0.0, 0.0)
CAMERA_POS = chrono.ChVector3d(0.0, 2.5, 1.5)
CAMERA_TARGET = chrono.ChVector3d(0.0, 0.0, 0.2)


def command_wheels(robot_tb, sim_time):
    """Set TurtleBot wheel speeds for visible left and right pivot turns."""
    left_speed = STOP_SPEED
    right_speed = STOP_SPEED
    if 0.5 <= sim_time < 2.5:
        left_speed = STOP_SPEED
        right_speed = TURN_SPEED
    elif 3.0 <= sim_time < 5.0:
        left_speed = TURN_SPEED
        right_speed = STOP_SPEED
    robot_tb.SetMotorSpeed(left_speed, LEFT_DRIVE_WHEEL)
    robot_tb.SetMotorSpeed(right_speed, RIGHT_DRIVE_WHEEL)
    return left_speed, right_speed


# === System & Ground === NSC system with Bullet collision for wheel-ground contact
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

ground_mat = chrono.ChContactMaterialNSC()
ground_mat.SetFriction(0.8)
ground_mat.SetRestitution(0.0)
ground = chrono.ChBodyEasyBox(
    GROUND_SIZE_X,
    GROUND_SIZE_Y,
    GROUND_THICKNESS,
    GROUND_DENSITY,
    True,
    True,
    ground_mat,
)
ground.SetPos(chrono.ChVector3d(0.0, 0.0, GROUND_Z))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)


# === Robot === TurtleBot owns its parts and is driven by per-wheel motor speeds
robot_tb = robot.TurtleBot(system, INIT_POS, INIT_ROT)
robot_tb.Initialize()
left_wheel_id = LEFT_DRIVE_WHEEL  # cache: reused for commands and active-speed logging
right_wheel_id = RIGHT_DRIVE_WHEEL  # cache: reused for commands and active-speed logging


# === Visualization === Irrlicht must initialize before adding scene elements
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("TurtleBot robot - rigid terrain")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(CAMERA_POS, CAMERA_TARGET)
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


# === Main Loop === render in real time while motors change at scheduled times
data_context = nullcontext()

frame = 0
try:
    with data_context as data_file:
        while vis.Run() and system.GetChTime() < SIM_END:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            for _ in range(RENDER_EVERY):
                sim_time = system.GetChTime()
                left_speed, right_speed = command_wheels(robot_tb, sim_time)
                system.DoStepDynamics(TIME_STEP)
                if system.GetChTime() >= SIM_END:
                    break
except (RuntimeError, ValueError) as exc:  # guard: Chrono solver or invalid-state failure
    traceback.print_exc()
    raise
except (OSError, IOError) as exc:  # guard: review output file or frame path failure
    traceback.print_exc()
    raise
finally:
    pass


# === Post-processing === assemble review artifacts only when recording is requested
