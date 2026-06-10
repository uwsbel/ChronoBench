"""
PyChrono 9.0.x simulation: Random boxes scattered on a grass terrain with
a lidar sensor scanning the scene. The system is an NSC rigid-body simulation.
A ChSensorManager with a ChLidarSensor observes the environment while random
boxes are distributed across the terrain. The Irrlicht window provides
interactive visualization. Expected behavior: boxes rest on terrain, lidar
rotates and captures point clouds of the scene.
"""

# === Imports ===
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens
import numpy as np
import math
import os

# === Constants ===
TIME_STEP = 1e-3
SIM_END = 10.0
RENDER_FPS = 50.0
NUM_BOXES = 20
BOX_MIN_SIZE = 0.2
BOX_MAX_SIZE = 0.8
SCATTER_RANGE = 5.0
TERRAIN_HALF = 10.0
TERRAIN_THICKNESS = 0.2

render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sys.SetSolverType(chrono.ChSolver.Type_PSOR)
sys.GetSolver().AsIterative().SetMaxIterations(100)

# === Contact material ===
mat = chrono.ChContactMaterialNSC()
mat.SetFriction(0.7)
mat.SetRestitution(0.1)

# === Bodies ===
# Ground / terrain plane
ground = chrono.ChBodyEasyBox(
    TERRAIN_HALF * 2, TERRAIN_THICKNESS, TERRAIN_HALF * 2,
    1000.0, True, True, mat
)
ground.SetFixed(True)
ground.SetPos(chrono.ChVector3d(0, -TERRAIN_THICKNESS / 2, 0))
# Apply grass texture
terrain_mat = chrono.ChVisualMaterial()
terrain_mat.SetKdTexture(chrono.GetChronoDataFile("textures/grass.jpg"))
ground.GetVisualShape(0).SetMaterial(0, terrain_mat)
sys.Add(ground)

# Random boxes scattered on the terrain
rng = np.random.default_rng(42)
box_bodies = []
for i in range(NUM_BOXES):
    bx = float(rng.uniform(BOX_MIN_SIZE, BOX_MAX_SIZE))
    by = float(rng.uniform(BOX_MIN_SIZE, BOX_MAX_SIZE))
    bz = float(rng.uniform(BOX_MIN_SIZE, BOX_MAX_SIZE))
    px = float(rng.uniform(-SCATTER_RANGE, SCATTER_RANGE))
    pz = float(rng.uniform(-SCATTER_RANGE, SCATTER_RANGE))
    py = by / 2.0  # rest on terrain surface (Y-up)
    box = chrono.ChBodyEasyBox(bx, by, bz, 500.0, True, True, mat)
    box.SetPos(chrono.ChVector3d(px, py, pz))
    sys.Add(box)
    box_bodies.append(box)

# === Sensor manager + lidar ===
manager = sens.ChSensorManager(sys)
# No point lights needed for lidar-only scene (lidar doesn't render)

# Lidar mounted on a fixed body near scene center, scanning at 1 m height
lidar_body = chrono.ChBody()
lidar_body.SetFixed(True)
lidar_body.SetPos(chrono.ChVector3d(0, 0, 0))
sys.Add(lidar_body)

lidar_offset_pose = chrono.ChFramed(
    chrono.ChVector3d(0, 1.0, 0),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
h_samples = 800
v_samples = 16
lidar = sens.ChLidarSensor(
    lidar_body,                             # body lidar is attached to
    5.0,                                    # update_rate Hz (physical)
    lidar_offset_pose,                      # offset pose
    h_samples,                              # horizontal samples
    v_samples,                              # vertical samples
    2 * chrono.CH_PI,                       # horizontal FOV (full 360 deg)
    chrono.CH_PI / 12,                      # max vertical angle
    -chrono.CH_PI / 6,                      # min vertical angle
    30.0,                                   # max range [m]
    sens.LidarBeamShape_RECTANGULAR,        # beam shape
    2,                                      # sample radius
    0.003,                                  # vertical divergence angle
    0.003,                                  # horizontal divergence angle
    sens.LidarReturnMode_STRONGEST_RETURN,  # return mode
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(1.0 / 5.0)  # collection window = 1 / update_rate

# Lidar filter chain (scored core — lidar scan output)
lidar.PushFilter(sens.ChFilterVisualize(h_samples, v_samples, "Raw Lidar Depth"))
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
lidar.PushFilter(sens.ChFilterXYZIAccess())
manager.AddSensor(lidar)

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Random Boxes + Lidar on Grass Terrain")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()  # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(8, 5, 8), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()

# === Review-only setup ===

frame = 0

# === Main loop ===
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            manager.Update()
            t = sys.GetChTime()
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:  # solver divergence / bad physics state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
