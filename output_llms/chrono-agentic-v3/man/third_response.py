"""
HMMWV vehicle on rigid terrain with lidar sensor and random box obstacles.

System type: ChSystemNSC (owned by HMMWV_Full wrapper).
Main bodies: HMMWV chassis, four wheel spindles, rigid terrain patch, random box obstacles.
Sensors: ChLidarSensor attached to the chassis for environment scanning.
Expected behavior: HMMWV drives forward on grass-textured terrain, lidar scans surrounding boxes.
"""

import math
import os
import random
import numpy as np
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.sensor as sens

# === Named constants ===
STEP_SIZE       = 2e-3              # physics time step (s)
SIM_END         = 20.0              # simulation duration (s)
RENDER_FPS      = 50.0              # Irrlicht render cadence (Hz)
TERRAIN_LENGTH  = 300.0             # terrain X extent (m)
TERRAIN_WIDTH   = 300.0             # terrain Y extent (m)
INIT_LOC        = chrono.ChVector3d(0, 0, 0.5)   # HMMWV spawn position
INIT_ROT        = chrono.QUNIT                    # heading: straight ahead
NUM_BOXES       = 20                # number of random box obstacles
BOX_SCATTER_R   = 30.0             # scatter radius around origin (m)
LIDAR_UPDATE_HZ = 5.0               # lidar update rate (Hz) — physical, not 1/dt
RENDER_EVERY    = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

# === Data paths ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())          # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')      # locate vehicle data files

# === Vehicle setup ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                          # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(STEP_SIZE)
hmmwv.Initialize()

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system = hmmwv.GetSystem()                  # ChSystemNSC owned by the wrapper
chassis = hmmwv.GetChassisBody()            # main chassis rigid body; cache: fetched once
# wheels/spindles: hmmwv.GetVehicle().GetAxle(i); terrain: RigidTerrain patch body below
# joints: suspension + steering links created inside the wrapper

system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED, after Initialize
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

# Post-Initialize visualization types
hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# === Terrain ===
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    TERRAIN_LENGTH,
    TERRAIN_WIDTH
)
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.6, 0.8, 0.4))
terrain.Initialize()

# === Random box obstacles ===
random.seed(42)                                 # deterministic layout
np_rng = np.random.default_rng(42)             # numpy rng for box dimensions
box_mat = chrono.ChContactMaterialNSC()
box_mat.SetFriction(0.7)
box_mat.SetRestitution(0.1)
for _ in range(NUM_BOXES):
    bx = random.uniform(-BOX_SCATTER_R, BOX_SCATTER_R)
    by = random.uniform(-BOX_SCATTER_R, BOX_SCATTER_R)
    # keep boxes away from vehicle spawn
    if abs(bx) < 3.0 and abs(by) < 3.0:
        bx += 5.0
    bw = float(np_rng.uniform(0.5, 2.0))
    bd = float(np_rng.uniform(0.5, 2.0))
    bh = float(np_rng.uniform(0.5, 1.5))
    box_body = chrono.ChBodyEasyBox(bw, bd, bh, 500.0, True, True, box_mat)
    box_body.SetPos(chrono.ChVector3d(bx, by, bh / 2.0))
    box_body.SetFixed(True)
    system.AddBody(box_body)

# === Sensor manager and lidar ===
manager = sens.ChSensorManager(system)
# Point light for OptiX rendering (lidar render)
manager.scene.AddPointLight(
    chrono.ChVector3f(0, 0, 100),
    chrono.ChColor(1, 1, 1),
    500.0,
)

# Lidar mounted on the HMMWV chassis, forward-facing
lidar_offset = chrono.ChFramed(
    chrono.ChVector3d(2.0, 0, 1.0),                           # offset from chassis: front/above
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),  # no tilt
)
H_SAMPLES = 800
V_SAMPLES = 32
lidar = sens.ChLidarSensor(
    chassis,                              # body the lidar rides on
    LIDAR_UPDATE_HZ,                      # update rate Hz — physical, not 1/dt
    lidar_offset,                          # offset pose
    H_SAMPLES,                            # horizontal samples
    V_SAMPLES,                            # vertical samples
    2 * chrono.CH_PI,                     # horizontal FOV (full 360°)
    chrono.CH_PI / 12,                    # max vertical angle
    -chrono.CH_PI / 6,                    # min vertical angle
    80.0,                                  # max range (m)
    sens.LidarBeamShape_RECTANGULAR,
    2,                                     # sample_radius
    0.003,                                 # vertical divergence angle
    0.003,                                 # horizontal divergence angle
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(1.0 / LIDAR_UPDATE_HZ)  # lidar collection window = 1/update_rate

# Lidar filter chain
lidar.PushFilter(sens.ChFilterVisualize(H_SAMPLES, V_SAMPLES, "Raw Lidar Depth"))
lidar.PushFilter(sens.ChFilterDIAccess())           # host access to depth+intensity
lidar.PushFilter(sens.ChFilterPCfromDepth())        # depth -> XYZI point cloud
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
lidar.PushFilter(sens.ChFilterXYZIAccess())         # host access to XYZI
manager.AddSensor(lidar)

# === Irrlicht visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV with Lidar and Obstacles")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()              # vehicle truths use directional light
vis.AttachVehicle(hmmwv.GetVehicle())

# === Interactive driver ===
render_step_size = 1.0 / RENDER_FPS   # precomputed once
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time  = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# === Review-only: recording setup ===

# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

try:
    while vis.Run() and system.GetChTime() < SIM_END:
        time = system.GetChTime()  # cache: fetched once per outer frame

        if step_number % RENDER_EVERY == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()


        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        hmmwv.Advance(STEP_SIZE)
        vis.Advance(STEP_SIZE)

        manager.Update()           # pump sensors exactly once per physics step


        step_number += 1
        realtime_timer.Spin(STEP_SIZE)

except (RuntimeError, ValueError) as exc:       # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
