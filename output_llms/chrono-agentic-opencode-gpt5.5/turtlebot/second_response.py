"""TurtleBot differential-drive simulation on rigid NSC terrain.

The scene uses a PyChrono robot.TurtleBot on a fixed concrete ground box with
Bullet collision enabled. The robot drives straight for 5 seconds, pivots left
for 5 seconds, then pivots right for the remainder of the bounded run.
"""

import math
import traceback

import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.robot as robot


# === Constants === named values keep robot setup and timing explicit
TIME_STEP = 2e-3
SIM_END = 12.0
GROUND_X = 20.0
GROUND_Y = 20.0
GROUND_Z = 1.0
GROUND_DENSITY = 1000.0
GROUND_POSITION = chrono.ChVector3d(0.0, 0.0, -0.6)
INIT_POSITION = chrono.ChVector3d(0.0, 0.2, -0.1)
INIT_ROTATION = chrono.ChQuaterniond(1.0, 0.0, 0.0, 0.0)
LEFT_DRIVE_WHEEL = 0
RIGHT_DRIVE_WHEEL = 1
STRAIGHT_SPEED = -12.0
STOP_SPEED = 0.0


# === System & gravity === NSC rover contact on Bullet collision
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)


# === Bodies === fixed ground plus catalog TurtleBot bodies created by the wrapper
ground_mat = chrono.ChContactMaterialNSC()
ground_mat.SetFriction(0.8)
ground_mat.SetRestitution(0.0)
ground = chrono.ChBodyEasyBox(GROUND_X, GROUND_Y, GROUND_Z, GROUND_DENSITY, True, True, ground_mat)
ground.SetPos(GROUND_POSITION)
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

robot_tb = robot.TurtleBot(system, INIT_POSITION, INIT_ROTATION)
robot_tb.Initialize()
get_time = system.GetChTime  # cache: method handle reused in the hot loop


# === Controls === differential-drive commands requested by the task
current_mode = None


def move(mode):
    if mode == "straight":
        robot_tb.SetMotorSpeed(STRAIGHT_SPEED, LEFT_DRIVE_WHEEL)
        robot_tb.SetMotorSpeed(STRAIGHT_SPEED, RIGHT_DRIVE_WHEEL)
    elif mode == "left":
        robot_tb.SetMotorSpeed(STOP_SPEED, LEFT_DRIVE_WHEEL)
        robot_tb.SetMotorSpeed(STRAIGHT_SPEED, RIGHT_DRIVE_WHEEL)
    elif mode == "right":
        robot_tb.SetMotorSpeed(STRAIGHT_SPEED, LEFT_DRIVE_WHEEL)
        robot_tb.SetMotorSpeed(STOP_SPEED, RIGHT_DRIVE_WHEEL)
    else:
        raise ValueError(f"invalid TurtleBot movement mode: {mode}")


# === Visualization === Irrlicht window initialized before scene elements
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("TurtleBot differential drive")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(2.5, -4.0, 2.0), chrono.ChVector3d(0.0, 0.0, 0.2))
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


# === Main loop === render the robot and apply mode changes by simulation time
try:

    while vis.Run() and get_time() < SIM_END:
        sim_time = get_time()
        if sim_time < 5.0:
            requested_mode = "straight"
        elif sim_time < 10.0:
            requested_mode = "left"
        else:
            requested_mode = "right"

        if requested_mode != current_mode:
            current_mode = requested_mode
            print(f"TurtleBot action: {current_mode}")
        move(current_mode)

        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        system.DoStepDynamics(TIME_STEP)
except (RuntimeError, ValueError, OSError, IOError) as exc:
    traceback.print_exc()
    raise
finally:
    pass
