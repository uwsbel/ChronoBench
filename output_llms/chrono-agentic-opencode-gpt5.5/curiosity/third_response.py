"""Curiosity rover with a chassis-mounted lidar sensor on rigid terrain.

This standalone PyChrono NSC simulation builds the built-in Curiosity rover,
drives it forward on a fixed textured ground body, and updates a lidar sensor
attached to the rover chassis. The rover should roll steadily while the lidar
produces depth and point-cloud data through its sensor-manager filter chain.
"""

import math
import traceback


import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.robot as robot
import pychrono.sensor as sens


# === Constants === named parameters keep rover, sensor, and recording behavior explicit
TIME_STEP = 1.0e-3
SIM_END = 8.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

GROUND_SIZE_X = 20.0
GROUND_SIZE_Y = 20.0
GROUND_THICKNESS = 1.0
GROUND_Z = -0.5

INIT_POS = chrono.ChVector3d(0.0, 0.2, 0.0)
INIT_ROT = chrono.ChQuaterniond(1.0, 0.0, 0.0, 0.0)
MAX_STEERING = math.pi / 6.0

LIDAR_RATE = 5.0
LIDAR_HORIZONTAL_SAMPLES = 800
LIDAR_VERTICAL_SAMPLES = 1
LIDAR_HORIZONTAL_FOV = 2.0 * chrono.CH_PI
LIDAR_MAX_VERTICAL_ANGLE = 0.0
LIDAR_MIN_VERTICAL_ANGLE = 0.0
LIDAR_MAX_RANGE = 40.0
LIDAR_OFFSET = chrono.ChVector3d(0.8, 0.0, 1.2)
LIDAR_ROT = chrono.QuatFromAngleAxis(0.0, chrono.ChVector3d(0.0, 1.0, 0.0))


def steering_command(time):
    """Return a smooth Curiosity steering command in radians."""
    if 2.0 < time < 5.0:
        return MAX_STEERING * (time - 2.0) / 3.0
    if 5.0 <= time < 8.0:
        return MAX_STEERING * (8.0 - time) / 3.0
    return 0.0


def main():
    # === System & terrain === NSC rover contact uses Bullet collision on a fixed ground body
    system = chrono.ChSystemNSC()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    system.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, -9.81))
    chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
    chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

    ground_mat = chrono.ChContactMaterialNSC()
    ground_mat.SetFriction(0.8)
    ground_mat.SetRestitution(0.0)

    ground = chrono.ChBodyEasyBox(
        GROUND_SIZE_X, GROUND_SIZE_Y, GROUND_THICKNESS, 1000.0, True, True, ground_mat
    )
    ground.SetPos(chrono.ChVector3d(0.0, 0.0, GROUND_Z))
    ground.SetFixed(True)
    ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
    system.Add(ground)

    # === Rover === built-in Curiosity topology owns chassis, wheels, suspension, and motors
    rover = robot.Curiosity(system)
    driver = robot.CuriosityDCMotorControl()
    for wheel_id in range(6):
        driver.SetMotorNoLoadSpeed(8.0, wheel_id)
        driver.SetMotorStallTorque(120.0, wheel_id)
    rover.SetDriver(driver)
    rover.Initialize(chrono.ChFramed(INIT_POS, INIT_ROT))
    chassis_body = rover.GetChassis().GetBody()  # cache: chassis body reused for sensor and logging
    ground_body = ground  # cache: named fixed support body for review logging

    # === Sensor manager & lidar === chassis-mounted lidar generates depth and point-cloud streams
    manager = sens.ChSensorManager(system)
    lidar_pose = chrono.ChFramed(LIDAR_OFFSET, LIDAR_ROT)
    lidar = sens.ChLidarSensor(
        chassis_body,
        LIDAR_RATE,
        lidar_pose,
        LIDAR_HORIZONTAL_SAMPLES,
        LIDAR_VERTICAL_SAMPLES,
        LIDAR_HORIZONTAL_FOV,
        LIDAR_MAX_VERTICAL_ANGLE,
        LIDAR_MIN_VERTICAL_ANGLE,
        LIDAR_MAX_RANGE,
        sens.LidarBeamShape_RECTANGULAR,
        2,
        0.003,
        0.003,
        sens.LidarReturnMode_STRONGEST_RETURN,
    )
    lidar.SetName("Curiosity Chassis Lidar")
    lidar.SetLag(0.0)
    lidar.SetCollectionWindow(1.0 / LIDAR_RATE)
    lidar.PushFilter(sens.ChFilterVisualize(LIDAR_HORIZONTAL_SAMPLES, LIDAR_VERTICAL_SAMPLES, "Raw Lidar Depth"))
    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
    lidar.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar)

    # === Visualization === Irrlicht window is built unconditionally for the review view
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("Curiosity rover with lidar")
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(-5.0, -4.0, 3.0), chrono.ChVector3d(0.5, 0.0, 0.6))
    vis.AddTypicalLights()
    vis.AddLightWithShadow(
        chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0.0, 0.0, 0.5), 3, 4, 10, 40, 512
    )
    vis.AddGrid(1.0, 1.0, 20, 20, chrono.ChCoordsysd(), chrono.ChColor(0.4, 0.4, 0.4))
    vis.BindAll()


    # === Main loop === render frames, update rover controls and lidar, then advance dynamics
    frame = 0
    try:
        while vis.Run() and system.GetChTime() < SIM_END:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

            for _ in range(RENDER_EVERY):
                sim_time = system.GetChTime()
                steering = steering_command(sim_time)
                driver.SetSteering(steering)
                rover.Update()
                manager.Update()


                system.DoStepDynamics(TIME_STEP)
                if system.GetChTime() >= SIM_END:
                    break
    except (RuntimeError, ValueError) as exc:  # solver divergence or invalid state
        traceback.print_exc()
        raise
    except (OSError, IOError) as exc:  # file output or renderer path failures
        traceback.print_exc()
        raise
    finally:
        _ = ground_body


if __name__ == "__main__":
    main()
