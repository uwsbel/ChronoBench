"""TurtleBot differential-drive simulation on a rigid NSC contact floor.

The script builds a PyChrono TurtleBot catalog robot, a textured rigid ground
body, and an Irrlicht visualization. The robot drives straight, then pivots
left, then pivots right by assigning wheel motor speeds directly.
"""

import math
import traceback

import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.robot as robot


# === Constants ===
TIME_STEP = 2.0e-3
SIM_END = 12.0
RENDER_FPS = 30.0  # precomputed once for review capture cadence
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

GROUND_SIZE_X = 20.0
GROUND_SIZE_Y = 20.0
GROUND_THICKNESS = 1.0
GROUND_DENSITY = 1000.0
GROUND_Z = -0.6
ROBOT_INIT_POS = chrono.ChVector3d(0.0, 0.2, 0.0)
ROBOT_INIT_ROT = chrono.ChQuaterniond(1.0, 0.0, 0.0, 0.0)

LEFT_DRIVE_WHEEL = 0
RIGHT_DRIVE_WHEEL = 1
DRIVE_SPEED = -10.0


# === System and contact ===
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)


# === Ground body ===
ground_mat = chrono.ChContactMaterialNSC()
ground_mat.SetFriction(0.8)
ground_mat.SetRestitution(0.02)
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


# === TurtleBot robot ===
robot_tb = robot.TurtleBot(system, ROBOT_INIT_POS, ROBOT_INIT_ROT)
robot_tb.Initialize()


def move(mode):
    """Set TurtleBot wheel speeds for the requested differential-drive mode."""
    if mode == "straight":
        robot_tb.SetMotorSpeed(DRIVE_SPEED, LEFT_DRIVE_WHEEL)
        robot_tb.SetMotorSpeed(DRIVE_SPEED, RIGHT_DRIVE_WHEEL)
    elif mode == "left":
        robot_tb.SetMotorSpeed(0.0, LEFT_DRIVE_WHEEL)
        robot_tb.SetMotorSpeed(DRIVE_SPEED, RIGHT_DRIVE_WHEEL)
    elif mode == "right":
        robot_tb.SetMotorSpeed(DRIVE_SPEED, LEFT_DRIVE_WHEEL)
        robot_tb.SetMotorSpeed(0.0, RIGHT_DRIVE_WHEEL)
    else:
        raise ValueError(f"invalid TurtleBot movement mode: {mode}")


# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("TurtleBot differential-drive sequence")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(4.5, -5.0, 2.8), chrono.ChVector3d(0.8, 0.0, 0.4))
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
    20,
    20,
    chrono.ChCoordsysd(chrono.ChVector3d(0.0, 0.0, 0.0), chrono.QUNIT),
    chrono.ChColor(0.4, 0.4, 0.4),
)


# === Cached state ===
chassis_body = next(
    body for body in system.GetBodies() if body.GetName() == "chassis_body"
)  # cache: TurtleBot chassis body found once, reused in loop logs
last_mode = None
frame = 0


# === Main loop ===
try:

    while vis.Run() and system.GetChTime() < SIM_END:
        robot_pos = chassis_body.GetPos()
        vis.UpdateCamera(
            robot_pos + chrono.ChVector3d(2.0, -3.0, 1.4),
            robot_pos + chrono.ChVector3d(0.2, 0.0, 0.2),
        )
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(RENDER_EVERY):
            sim_time = system.GetChTime()
            if sim_time < 5.0:
                mode = "straight"
            elif sim_time < 10.0:
                mode = "left"
            else:
                mode = "right"

            if mode != last_mode:
                print(f"TurtleBot action: {mode}")
                last_mode = mode

            move(mode)

            pos = chassis_body.GetPos()
            euler = chassis_body.GetRot().GetCardanAnglesXYZ()

            system.DoStepDynamics(TIME_STEP)
            if system.GetChTime() >= SIM_END:
                break

except (RuntimeError, ValueError) as exc:  # solver divergence / invalid mode guard
    traceback.print_exc()
    raise
except (OSError, IOError) as exc:  # review-output file or frame path failure guard
    traceback.print_exc()
    raise
finally:
    pass
