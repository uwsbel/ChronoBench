"""TurtleBot differential-drive simulation on rigid terrain.

This PyChrono 9.0.0 script builds an NSC system with Bullet collision, a fixed
rigid ground body, and the catalog TurtleBot robot. The robot starts from a
specified pose, drives under timed left/right wheel-speed commands, and is shown
through a real-time Irrlicht visualization with camera, sky, and lights.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.robot as robot


# === Constants: explicit simulation timing and robot commands ===
TIME_STEP = 2.0e-3
SIM_END = 6.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

LEFT_DRIVE_WHEEL = 0
RIGHT_DRIVE_WHEEL = 1
FORWARD_SPEED = 10.0
TURN_SPEED = 10.0

INIT_POS = chrono.ChVector3d(0.0, 0.2, 0.0)
INIT_ROT = chrono.ChQuaterniond(1.0, 0.0, 0.0, 0.0)

GROUND_SIZE_X = 20.0
GROUND_SIZE_Y = 20.0
GROUND_SIZE_Z = 1.0
GROUND_DENSITY = 1000.0
GROUND_POS = chrono.ChVector3d(0.0, 0.0, -1.0)


def command_wheels(robot_tb, time):
    """Return and apply the timed differential-drive motor command."""
    if time < 1.0:
        left_speed = FORWARD_SPEED
        right_speed = FORWARD_SPEED
    elif time < 2.0:
        left_speed = 0.0
        right_speed = TURN_SPEED
    elif time < 3.0:
        left_speed = FORWARD_SPEED
        right_speed = FORWARD_SPEED
    elif time < 4.0:
        left_speed = TURN_SPEED
        right_speed = 0.0
    else:
        left_speed = FORWARD_SPEED
        right_speed = FORWARD_SPEED

    robot_tb.SetMotorSpeed(left_speed, LEFT_DRIVE_WHEEL)
    robot_tb.SetMotorSpeed(right_speed, RIGHT_DRIVE_WHEEL)
    return left_speed, right_speed


def main():
    # === System & collision: NSC rover world with rigid contact ===
    system = chrono.ChSystemNSC()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    system.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, -9.81))
    system.SetSolverType(chrono.ChSolver.Type_PSOR)
    system.GetSolver().AsIterative().SetMaxIterations(80)
    chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
    chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

    # === Rigid terrain: fixed textured box under the robot ===
    ground_mat = chrono.ChContactMaterialNSC()
    ground_mat.SetFriction(0.8)
    ground_mat.SetRestitution(0.05)
    ground = chrono.ChBodyEasyBox(
        GROUND_SIZE_X,
        GROUND_SIZE_Y,
        GROUND_SIZE_Z,
        GROUND_DENSITY,
        True,
        True,
        ground_mat,
    )
    ground.SetPos(GROUND_POS)
    ground.SetFixed(True)
    ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
    system.Add(ground)

    # === TurtleBot: catalog differential-drive robot with pose in constructor ===
    robot_tb = robot.TurtleBot(system, INIT_POS, INIT_ROT)
    robot_tb.Initialize()
    turtlebot = robot_tb  # cache: command object reused every step

    # === Visualization: Irrlicht initialized before scene elements ===
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("TurtleBot rigid terrain")
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(2.0, -3.0, 1.4), chrono.ChVector3d(0.0, 0.0, 0.1))
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
        chrono.ChCoordsysd(chrono.ChVector3d(0.0, 0.0, -0.495), chrono.QUNIT),
        chrono.ChColor(0.35, 0.35, 0.35),
    )


    frame = 0
    try:
        # === Main loop: render once per frame and advance fixed-size physics steps ===
        while vis.Run() and system.GetChTime() < SIM_END:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            frame += 1

            for _ in range(RENDER_EVERY):
                time = system.GetChTime()
                left_speed, right_speed = command_wheels(turtlebot, time)
                system.DoStepDynamics(TIME_STEP)
                if system.GetChTime() >= SIM_END:
                    break
    except (RuntimeError, ValueError) as exc:  # guard solver/runtime state failures
        traceback.print_exc()
        raise
    except (OSError, IOError) as exc:  # guard recording path and file I/O failures
        traceback.print_exc()
        raise
    finally:
        pass


if __name__ == "__main__":
    main()
