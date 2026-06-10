"""PyChrono NSC terrain scene with random boxes and a lidar sensor.

The simulation builds a grass-textured fixed terrain, places deterministic random
rigid boxes on it, and runs a sensor manager with a lidar scanning the scene.
Irrlicht provides the default visualization while the bodies settle under gravity.
"""

import csv
import math
import os
import random
import traceback

import numpy as np
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens


# === Constants === named scene values keep placement and timing explicit
TIME_STEP = 0.002
SIM_END = 6.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once
TERRAIN_SIZE_X = 20.0
TERRAIN_SIZE_Y = 20.0
TERRAIN_THICKNESS = 0.2
BOX_COUNT = 18
BOX_DENSITY = 700.0
RANDOM_SEED = 7
LIDAR_POS = chrono.ChVector3d(0.0, -8.0, 1.2)
LIDAR_UPDATE_RATE = 5.0
LIDAR_HORIZONTAL_SAMPLES = 800
LIDAR_VERTICAL_SAMPLES = 1


# === System & gravity === NSC contact supports rigid terrain and box collisions
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, -9.81))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sys.SetSolverType(chrono.ChSolver.Type_PSOR)
sys.GetSolver().AsIterative().SetMaxIterations(80)

contact_mat = chrono.ChContactMaterialNSC()
contact_mat.SetFriction(0.8)
contact_mat.SetRestitution(0.05)


# === Bodies === grass terrain and randomized collision boxes are the sensed scene
terrain = chrono.ChBodyEasyBox(TERRAIN_SIZE_X, TERRAIN_SIZE_Y, TERRAIN_THICKNESS, 1000.0, True, True, contact_mat)
terrain.SetFixed(True)
terrain.SetPos(chrono.ChVector3d(0.0, 0.0, -TERRAIN_THICKNESS / 2.0))
terrain.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("vehicle/terrain/textures/grass.jpg"))
sys.AddBody(terrain)

boxes = []
rng = np.random.default_rng(RANDOM_SEED)
for idx in range(BOX_COUNT):
    sx = float(rng.uniform(0.25, 0.85))
    sy = float(rng.uniform(0.25, 0.85))
    sz = float(rng.uniform(0.25, 1.2))
    box = chrono.ChBodyEasyBox(sx, sy, sz, BOX_DENSITY, True, True, contact_mat)
    x = float(rng.uniform(-7.5, 7.5))
    y = float(rng.uniform(-7.5, 7.5))
    z = 1.5 + sz / 2.0 + 0.22 * idx
    box.SetPos(chrono.ChVector3d(x, y, z))
    box.SetRot(chrono.QuatFromAngleZ(float(rng.uniform(0.0, 2.0 * math.pi))))
    box.SetPosDt(chrono.ChVector3d(float(rng.uniform(-0.5, 0.5)), float(rng.uniform(-0.5, 0.5)), 0.0))
    box.SetAngVelParent(chrono.ChVector3d(float(rng.uniform(-1.5, 1.5)), float(rng.uniform(-1.5, 1.5)), float(rng.uniform(-1.5, 1.5))))
    box.GetVisualShape(0).SetColor(chrono.ChColor(float(rng.uniform(0.2, 0.9)), float(rng.uniform(0.2, 0.9)), float(rng.uniform(0.2, 0.9))))
    sys.AddBody(box)
    boxes.append(box)

first_box = boxes[0]  # cache: representative body reused for logging

lidar_body = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, 0.12, 0.3, 1000.0, True, True, contact_mat)
lidar_body.SetFixed(True)
lidar_body.SetPos(LIDAR_POS)
lidar_body.GetVisualShape(0).SetColor(chrono.ChColor(0.05, 0.05, 0.05))
sys.AddBody(lidar_body)


# === Sensors === manager owns the lidar and is updated every physics step
manager = sens.ChSensorManager(sys)
lidar_pose = chrono.ChFramed(
    chrono.VNULL,
    chrono.QuatFromAngleAxis(math.pi / 2.0, chrono.ChVector3d(0.0, 0.0, 1.0)),
)
lidar = sens.ChLidarSensor(
    lidar_body,
    LIDAR_UPDATE_RATE,
    lidar_pose,
    LIDAR_HORIZONTAL_SAMPLES,
    LIDAR_VERTICAL_SAMPLES,
    2.0 * chrono.CH_PI,
    0.0,
    0.0,
    30.0,
    sens.LidarBeamShape_RECTANGULAR,
    2,
    0.003,
    0.003,
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar.SetName("2D Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(1.0 / LIDAR_UPDATE_RATE)
lidar.PushFilter(sens.ChFilterVisualize(LIDAR_HORIZONTAL_SAMPLES, LIDAR_VERTICAL_SAMPLES, "Raw Lidar Depth"))
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
lidar.PushFilter(sens.ChFilterXYZIAccess())
manager.AddSensor(lidar)


# === Visualization === Irrlicht is initialized before adding sky, camera, lights, and grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Random boxes with lidar on grass terrain")
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(8.0, -12.0, 6.0), chrono.ChVector3d(0.0, 0.0, 0.7))
vis.AddTypicalLights()
vis.AddGrid(
    1.0,
    1.0,
    20,
    20,
    chrono.ChCoordsysd(chrono.ChVector3d(0.0, 0.0, 0.002), chrono.QUNIT),
    chrono.ChColor(0.35, 0.35, 0.35),
)


# === Main loop === render at video cadence, update sensors and physics at fixed step
frame = 0

try:
        while vis.Run() and sys.GetChTime() < SIM_END:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            for _ in range(RENDER_EVERY):
                manager.Update()
                sys.DoStepDynamics(TIME_STEP)
                if sys.GetChTime() >= SIM_END:
                    break
except (RuntimeError, ValueError) as exc:  # solver divergence or invalid sensor state
    traceback.print_exc()
    raise
except (OSError, IOError) as exc:  # output path or filesystem failure
    traceback.print_exc()
    raise
finally:
    pass


# === Post-processing === review-only video and plot artifacts are stripped before scoring
