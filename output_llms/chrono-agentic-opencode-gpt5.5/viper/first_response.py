"""Viper rover on rigid terrain using a PyChrono NSC system.

The script builds a Z-up Chrono system with Bullet collision, a fixed textured
ground body, the built-in Viper rover with its DC motor steering driver, and an
Irrlicht real-time visualization. The rover drives forward while steering is
smoothly ramped up and down over the run.
"""

import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.robot as robot


# === Constants === named parameters keep the rover setup and control explicit
TIME_STEP = 1.0e-3
SIM_END = 6.0
GROUND_LENGTH = 20.0
GROUND_WIDTH = 20.0
GROUND_THICKNESS = 1.0
GROUND_DENSITY = 1000.0
GROUND_Z = -1.0
INIT_POS = chrono.ChVector3d(0.0, 0.2, 0.0)
INIT_ROT = chrono.ChQuaterniond(1.0, 0.0, 0.0, 0.0)
MAX_STEERING = math.pi / 6.0
STEERING_START = 1.0
STEERING_END = 5.0


def steering_command(time):
    """Return a smooth Viper steering command for the current simulation time."""
    if time <= STEERING_START:
        return 0.0
    if time <= STEERING_END:
        return MAX_STEERING * (time - STEERING_START) / (STEERING_END - STEERING_START)
    return MAX_STEERING * max(0.0, (SIM_END - time) / (SIM_END - STEERING_END))


def main():
    # === System & gravity === NSC rover contacts use Bullet in a Z-up world
    system = chrono.ChSystemNSC()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    system.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, -9.81))
    chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
    chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

    # === Ground body === fixed rigid support with NSC material for wheel contact
    ground_mat = chrono.ChContactMaterialNSC()
    ground_mat.SetFriction(0.8)
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
    ground.SetPos(chrono.ChVector3d(0.0, 0.0, GROUND_Z))
    ground.SetFixed(True)
    ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
    system.Add(ground)

    # === Rover & driver === built-in Viper topology owns chassis, wheels, joints, and motors
    rover = robot.Viper(system)
    driver = robot.ViperDCMotorControl()
    rover.SetDriver(driver)
    rover.Initialize(chrono.ChFramed(INIT_POS, INIT_ROT))
    chassis_body = rover.GetChassis().GetBody()  # cache: rover chassis reused for logging

    # === Visualization === Irrlicht initializes before sky, camera, lights, and grid
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("Viper rover - rigid terrain")
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.0, 2.5, 1.5), chrono.ChVector3d(0.0, 0.0, 1.0))
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


    try:

        # === Main loop === render, ramp steering, update rover, then step dynamics
        while vis.Run() and system.GetChTime() < SIM_END:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()


            time = system.GetChTime()
            steering = steering_command(time)
            driver.SetSteering(steering)
            rover.Update()


            system.DoStepDynamics(TIME_STEP)

    except (RuntimeError, ValueError) as exc:  # solver divergence or invalid rover state
        raise
    except (OSError, IOError) as exc:  # output path or recording file failure
        raise
    finally:
        pass


if __name__ == "__main__":
    main()
