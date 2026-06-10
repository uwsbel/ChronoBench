"""
Curiosity Mars Rover with Lidar Sensor

Simulates the Curiosity rover on rigid terrain with an on-board lidar sensor.
System type: ChSystemNSC (non-smooth contact, as required for rovers).
Rover drives forward with mild steering while lidar scans the terrain.
"""

import os
import math
import csv

# === Review-only: recording infrastructure ===

# === PyChrono imports ===
import pychrono.core as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

# === Named constants ===
TIME_STEP = 1e-3
SIM_END = 20.0
RENDER_FPS = 50.0
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))

ROVER_INIT_POS = chrono.ChVector3d(0, 0.2, 0)
ROVER_INIT_ROT = chrono.ChQuaterniond(1, 0, 0, 0)
MAX_STEERING = math.pi / 6

# === System & collision (NSC, Bullet) ===
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# === Ground (rigid terrain) ===
ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.5))  # top at z=0 for Curiosity
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# === Curiosity rover ===
rover = robot.Curiosity(system)
driver = robot.CuriosityDCMotorControl()
rover.SetDriver(driver)
rover.Initialize(chrono.ChFramed(ROVER_INIT_POS, ROVER_INIT_ROT))

# === Sensor manager ===
manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(
    chrono.ChVector3f(2, 2.5, 100),
    chrono.ChColor(1.0, 1.0, 1.0),
    500.0,
)

# === Lidar sensor mounted on rover chassis ===
lidar_offset = chrono.ChFramed(
    chrono.ChVector3d(-0.5, 0, 0.8),
    chrono.QuatFromAngleAxis(0.1, chrono.ChVector3d(0, 1, 0)),
)
lidar = sens.ChLidarSensor(
    rover.GetChassis().GetBody(),  # attach to chassis
    5.0,                          # update rate Hz
    lidar_offset,
    800,                          # horizontal samples
    1,                            # vertical samples (2D scan)
    2 * chrono.CH_PI,             # horizontal FOV
    0.0,                          # max vertical angle
    0.0,                          # min vertical angle
    100.0,                        # max range
    sens.LidarBeamShape_RECTANGULAR,
    2,                            # sample radius
    0.003,                        # vert divergence
    0.003,                        # hori divergence
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar.SetName("Curiosity Lidar")
lidar.SetLag(0)
lidar.SetCollectionWindow(1.0 / 5.0)

# Lidar filter chain (order matters):
lidar.PushFilter(sens.ChFilterVisualize(800, 1, "Raw Lidar Depth"))
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
lidar.PushFilter(sens.ChFilterXYZIAccess())
manager.AddSensor(lidar)

# === Irrlicht visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Curiosity Rover - Lidar Sensor")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(4, -4, 3), chrono.ChVector3d(0, 0, 1))
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 40, 40, chrono.ChCoordsysd(), chrono.ChColor(0.4, 0.4, 0.4))

# === CSV writer (review-only) ===

# === Main simulation loop ===
frame = 0
time = 0.0
try:
    while vis.Run() and system.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        # review-only: capture Irrlicht frame

        for _ in range(render_every):
            # Ramp steering: straight then turn
            if 3 < time < 8:
                steer = MAX_STEERING * (time - 3) / 5
            elif 8 < time < 13:
                steer = MAX_STEERING * (13 - time) / 5
            else:
                steer = 0.0
            driver.SetSteering(steer)
            rover.Update()

            # review-only: log CSV

            manager.Update()
            system.DoStepDynamics(TIME_STEP)
            time = system.GetChTime()
            if time >= SIM_END:
                break
finally:
    pass
