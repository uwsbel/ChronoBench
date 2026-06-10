"""
Man demo: sensor + lidar + random boxes on grass terrain.
System: NSC with collision, sensor manager, lidar sensor.
"""
import os
import math
import numpy as np

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens


# === Named constants ===
TIME_STEP = 1e-3
SIM_END = 10.0
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))

# Terrain
TERRAIN_SIZE = 20.0
TERRAIN_THICKNESS = 0.5

# Boxes
NUM_BOXES = 20
BOX_MIN_SIZE = 0.3
BOX_MAX_SIZE = 1.0
BOX_DENSITY = 1000.0

# Sensor
LIDAR_UPDATE_RATE = 5.0
LIDAR_H_SAMPLES = 800
LIDAR_V_SAMPLES = 300
LIDAR_FOV = 2 * chrono.CH_PI
LIDAR_MAX_RANGE = 100.0

# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sys.SetSolverType(chrono.ChSolver.Type_PSOR)
sys.GetSolver().AsIterative().SetMaxIterations(100)

# === Contact material ===
mat_ground = chrono.ChContactMaterialNSC()
mat_ground.SetFriction(0.8)
mat_ground.SetRestitution(0.0)

mat_box = chrono.ChContactMaterialNSC()
mat_box.SetFriction(0.6)
mat_box.SetRestitution(0.2)

# === Ground body (FIXED - terrain does not move) ===
ground = chrono.ChBodyEasyBox(TERRAIN_SIZE, TERRAIN_THICKNESS, TERRAIN_SIZE,
                               1000.0, True, True, mat_ground)
ground.SetPos(chrono.ChVector3d(0, -TERRAIN_THICKNESS / 2.0, 0))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/rock.jpg"))
sys.AddBody(ground)

# === Random boxes ===
np.random.seed(42)
boxes = []
for i in range(NUM_BOXES):
    sx = np.random.uniform(BOX_MIN_SIZE, BOX_MAX_SIZE)
    sy = np.random.uniform(BOX_MIN_SIZE, BOX_MAX_SIZE)
    sz = np.random.uniform(BOX_MIN_SIZE, BOX_MAX_SIZE)
    x = np.random.uniform(-TERRAIN_SIZE / 2.0 + sx, TERRAIN_SIZE / 2.0 - sx)
    y = sy / 2.0 + TERRAIN_THICKNESS
    z = np.random.uniform(-TERRAIN_SIZE / 2.0 + sz, TERRAIN_SIZE / 2.0 - sz)

    box = chrono.ChBodyEasyBox(sx, sy, sz, BOX_DENSITY, True, True, mat_box)
    box.SetPos(chrono.ChVector3d(x, y, z))
    box.SetRot(chrono.QuatFromAngleY(np.random.uniform(0, 2 * chrono.CH_PI)))
    sys.AddBody(box)
    boxes.append(box)

# === Sensor manager ===
manager = sens.ChSensorManager(sys)
intensity = 1.0
manager.scene.AddPointLight(
    chrono.ChVector3f(0, 5, 10),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(10, 5, -10),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)

# === Lidar sensor ===
# Attach to a fixed body above the scene
lidar_body = chrono.ChBody()
lidar_body.SetFixed(True)
lidar_body.SetPos(chrono.ChVector3d(0, 8, 0))
sys.AddBody(lidar_body)

lidar_offset = chrono.ChFramed(
    chrono.ChVector3d(0, 0, 0),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)

lidar = sens.ChLidarSensor(
    lidar_body,
    LIDAR_UPDATE_RATE,
    lidar_offset,
    LIDAR_H_SAMPLES,
    LIDAR_V_SAMPLES,
    LIDAR_FOV,
    chrono.CH_PI / 12,
    -chrono.CH_PI / 6,
    LIDAR_MAX_RANGE,
    sens.LidarBeamShape_RECTANGULAR,
    2,
    0.003,
    0.003,
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(1.0 / LIDAR_UPDATE_RATE)

lidar.PushFilter(sens.ChFilterVisualize(LIDAR_H_SAMPLES, LIDAR_V_SAMPLES, "Lidar Depth"))
lidar.PushFilter(sens.ChFilterDIAccess())
manager.AddSensor(lidar)

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Man Demo - Sensor + Lidar")
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(15, 10, -15), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AddGrid(2.0, 2.0, 20, 20,
            chrono.ChCoordsysd(), chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop ===

csv_file = None
writer = None
try:

    frame = 0
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()


        for step_idx in range(RENDER_EVERY):
            # Update sensors every physics step
            manager.Update()

            # Log data

            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break

        frame += 1

except (RuntimeError, ValueError) as exc:
    import traceback
    traceback.print_exc()
    raise

# === Review-only post-processing ===
