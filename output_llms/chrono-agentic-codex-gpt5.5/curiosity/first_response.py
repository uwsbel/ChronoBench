"""Curiosity rover on rigid NSC terrain with Irrlicht visualization.

The model builds a Bullet-enabled non-smooth contact system, a fixed textured
ground body, and Chrono's built-in Curiosity rover with its DC motor steering
driver. The rover rolls forward on the rigid terrain while steering ramps
smoothly for a visible real-time maneuver under lights, shadows, and a camera.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.robot as robot


# === Constants ===
TIME_STEP = 1.0e-3
SIM_END = 8.0
GROUND_LENGTH = 20.0
GROUND_WIDTH = 20.0
GROUND_THICKNESS = 1.0
GROUND_DENSITY = 1000.0
GROUND_Z = -0.5
ROVER_START = chrono.ChVector3d(-3.0, 0.0, 0.0)
ROVER_ROTATION = chrono.ChQuaterniond(1.0, 0.0, 0.0, 0.0)
MAX_STEERING = math.pi / 6.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once


# === System & Ground ===
def build_system():
    system = chrono.ChSystemNSC()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    system.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, -9.81))
    system.SetSolverType(chrono.ChSolver.Type_PSOR)
    system.GetSolver().AsIterative().SetMaxIterations(80)
    chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
    chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

    ground_mat = chrono.ChContactMaterialNSC()
    ground_mat.SetFriction(0.9)
    ground_mat.SetRestitution(0.01)
    ground = chrono.ChBodyEasyBox(
        GROUND_LENGTH,
        GROUND_WIDTH,
        GROUND_THICKNESS,
        GROUND_DENSITY,
        True,
        True,
        ground_mat,
    )
    ground.SetName("rigid_terrain")
    ground.SetPos(chrono.ChVector3d(0.0, 0.0, GROUND_Z))
    ground.SetFixed(True)
    ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
    system.Add(ground)
    return system


# === Rover & Driver ===
def build_rover(system):
    rover = robot.Curiosity(system)
    driver = robot.CuriosityDCMotorControl()
    rover.SetDriver(driver)
    rover.Initialize(chrono.ChFramed(ROVER_START, ROVER_ROTATION))
    chassis = rover.GetChassis().GetBody()  # cache: fetched once for logging and camera tracking
    return rover, driver, chassis


def steering_command(time):
    if 1.0 < time < 4.0:
        return MAX_STEERING * (time - 1.0) / 3.0
    if 4.0 <= time < 7.0:
        return MAX_STEERING * (7.0 - time) / 3.0
    return 0.0


# === Visualization ===
def build_visualization(system):
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("Curiosity rover - rigid terrain")
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(4.0, -7.0, 3.0), chrono.ChVector3d(0.0, 0.0, 0.5))
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
        chrono.ChCoordsysd(),
        chrono.ChColor(0.35, 0.35, 0.35),
    )
    return vis


# === Main Loop ===
def run():
    system = build_system()
    rover, driver, chassis = build_rover(system)
    vis = build_visualization(system)


    try:
        while vis.Run() and system.GetChTime() < SIM_END:
            time = system.GetChTime()
            camera_target = chassis.GetPos()
            vis.UpdateCamera(
                camera_target + chrono.ChVector3d(4.0, -7.0, 3.0),
                camera_target + chrono.ChVector3d(0.5, 0.0, 0.4),
            )
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

            for _ in range(RENDER_EVERY):
                step_time = system.GetChTime()
                driver.SetSteering(steering_command(step_time))
                rover.Update()
                system.DoStepDynamics(TIME_STEP)
                if system.GetChTime() >= SIM_END:
                    break
    except (RuntimeError, ValueError) as exc:
        traceback.print_exc()
        raise exc
    finally:
        pass


# === Entry Point ===
if __name__ == "__main__":
    try:
        run()
    except (OSError, IOError) as exc:
        traceback.print_exc()
        raise exc
    except (RuntimeError, ValueError) as exc:
        traceback.print_exc()
        raise exc
    finally:
        pass
