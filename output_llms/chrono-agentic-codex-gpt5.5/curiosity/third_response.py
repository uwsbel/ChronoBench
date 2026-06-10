"""Curiosity rover with a chassis-mounted 2D lidar sensor.

This self-contained PyChrono 9.0.0 simulation uses an NSC system with Bullet
contact, a rigid textured ground plane, the built-in Curiosity rover, and a
sensor manager that updates a forward-facing lidar mounted on the rover chassis.
The rover drives forward with smooth steering while the lidar visualizes depth
and point-cloud data from the moving platform.
"""

import math

import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.robot as robot
import pychrono.sensor as sens


# === Constants ===
TIME_STEP = 1.0e-3
SIM_END = 8.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once
MAX_STEERING = math.pi / 7.0

GROUND_SIZE_X = 24.0
GROUND_SIZE_Y = 24.0
GROUND_THICKNESS = 1.0
GROUND_DENSITY = 1000.0
GROUND_Z = -0.5

LIDAR_RATE = 5.0
LIDAR_HORIZONTAL_SAMPLES = 720
LIDAR_VERTICAL_SAMPLES = 1
LIDAR_HORIZONTAL_FOV = 2.0 * chrono.CH_PI
LIDAR_MAX_RANGE = 40.0
LIDAR_SAMPLE_RADIUS = 2
LIDAR_DIVERGENCE = 0.003


# === System & ground ===
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
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
ground.SetPos(chrono.ChVector3d(0, 0, GROUND_Z))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)


# === Rover ===
rover = robot.Curiosity(system)
driver = robot.CuriosityDCMotorControl()
rover.SetDriver(driver)

init_pos = chrono.ChVector3d(0, 0.2, 0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
rover.Initialize(chrono.ChFramed(init_pos, init_rot))

chassis_body = rover.GetChassis().GetBody()  # cache: lidar parent and pose logger


# === Sensor manager & lidar ===
manager = sens.ChSensorManager(system)

lidar_housing = chrono.ChVisualShapeBox(0.25, 0.18, 0.12)
lidar_housing.SetColor(chrono.ChColor(0.1, 0.8, 0.2))
chassis_body.AddVisualShape(
    lidar_housing,
    chrono.ChFramed(chrono.ChVector3d(0.55, 0.0, 1.05), chrono.QUNIT),
)

lidar_offset = chrono.ChFramed(
    chrono.ChVector3d(0.55, 0.0, 1.05),
    chrono.QuatFromAngleAxis(0.0, chrono.ChVector3d(0, 1, 0)),
)
lidar = sens.ChLidarSensor(
    chassis_body,
    LIDAR_RATE,
    lidar_offset,
    LIDAR_HORIZONTAL_SAMPLES,
    LIDAR_VERTICAL_SAMPLES,
    LIDAR_HORIZONTAL_FOV,
    0.0,
    0.0,
    LIDAR_MAX_RANGE,
    sens.LidarBeamShape_RECTANGULAR,
    LIDAR_SAMPLE_RADIUS,
    LIDAR_DIVERGENCE,
    LIDAR_DIVERGENCE,
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar.SetName("Curiosity 2D Lidar")
lidar.SetLag(0)
lidar.SetCollectionWindow(1.0 / LIDAR_RATE)
lidar.PushFilter(sens.ChFilterVisualize(LIDAR_HORIZONTAL_SAMPLES, LIDAR_VERTICAL_SAMPLES, "Raw Lidar Depth"))
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
lidar.PushFilter(sens.ChFilterXYZIAccess())
manager.AddSensor(lidar)


# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Curiosity rover with 2D lidar")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(4.0, -12.0, 5.0), chrono.ChVector3d(3.0, -1.0, 0.6))
vis.AddTypicalLights()
vis.AddLightWithShadow(
    chrono.ChVector3d(1.5, -2.5, 5.5),
    chrono.ChVector3d(0, 0, 0.5),
    3,
    4,
    10,
    40,
    512,
)
vis.AddGrid(
    1.0,
    1.0,
    24,
    24,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    chrono.ChColor(0.4, 0.4, 0.4),
)


# === Main loop ===
try:
    while vis.Run() and system.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(RENDER_EVERY):
            sim_time = system.GetChTime()
            steering = 0.0
            if 1.0 < sim_time < 4.0:
                steering = MAX_STEERING * (sim_time - 1.0) / 3.0
            elif 4.0 <= sim_time < 7.0:
                steering = MAX_STEERING * (7.0 - sim_time) / 3.0

            driver.SetSteering(steering)
            rover.Update()
            manager.Update()


            system.DoStepDynamics(TIME_STEP)
            if system.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:  # solver divergence / invalid numeric state
    raise
finally:
    pass
