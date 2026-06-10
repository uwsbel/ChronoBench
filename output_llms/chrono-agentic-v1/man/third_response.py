"""
HMMWV vehicle simulation with lidar sensor, sensor manager, and random obstacle boxes.

System type: ChSystemNSC (owned by HMMWV_Full wrapper).
Main bodies: HMMWV chassis + 4 axle/wheel assemblies, rigid terrain patch, N random boxes.
Sensors: ChSensorManager with a ChLidarSensor mounted on the chassis.
Terrain: RigidTerrain with grass.jpg texture (NSC contact method).
Driver: ChInteractiveDriverIRR (real-time interactive, scored-core default).
Expected behavior: HMMWV drives forward on grass terrain surrounded by random boxes;
  the lidar sensor scans the environment and visualises a live point cloud.
"""

import os
import math
import random
import numpy as np  # imported as required by prompt

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.sensor as sens

# === Constants ===
# Simulation timing
STEP_SIZE = 1e-3          # physics step (s)
SIM_END = 30.0            # total sim duration (s)
RENDER_FPS = 50.0         # Irrlicht render cadence
RENDER_STEP_SIZE = 1.0 / RENDER_FPS
RENDER_STEPS = max(1, math.ceil(RENDER_STEP_SIZE / STEP_SIZE))  # precomputed once

# Vehicle spawn
INIT_LOC = chrono.ChVector3d(0.0, 0.0, 0.5)   # chassis origin, slightly above ground
INIT_ROT = chrono.QuatFromAngleZ(0.0)          # facing +X

# Terrain
TERRAIN_LENGTH = 200.0
TERRAIN_WIDTH = 200.0

# Random boxes configuration
NUM_BOXES = 20
BOX_SPREAD_RADIUS = 40.0   # boxes scattered within this radius from origin
BOX_MIN_HALF = 0.3         # minimum half-size (m)
BOX_MAX_HALF = 1.0         # maximum half-size (m)
BOX_MASS = 500.0           # kg

# Lidar sensor parameters (physical rates — never 1/dt)
LIDAR_UPDATE_HZ = 5.0
LIDAR_H_SAMPLES = 800
LIDAR_V_SAMPLES = 300
LIDAR_H_FOV = 2 * chrono.CH_PI
LIDAR_MAX_VERT = chrono.CH_PI / 12
LIDAR_MIN_VERT = -chrono.CH_PI / 6
LIDAR_MAX_RANGE = 100.0

# === Vehicle setup (HMMWV_Full wrapper — owns the ChSystemNSC) ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)   # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                         # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(STEP_SIZE)
hmmwv.Initialize()

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system = hmmwv.GetSystem()                 # ChSystemNSC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize

# Cache frequently-used handles (fetched once, reused throughout)
chassis = hmmwv.GetChassisBody()           # cache: chassis rigid body
veh_obj = hmmwv.GetVehicle()              # cache: vehicle interface

# Set visualization types (after Initialize)
hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === Terrain (RigidTerrain with grass texture) ===
terrain = veh.RigidTerrain(system)

patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
)
# Use grass.jpg texture (as specified in the prompt)
patch.SetTexture(chrono.GetChronoDataFile("vehicle/terrain/textures/grass.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.4, 0.7, 0.2))

terrain.Initialize()

# === Random obstacle boxes ===
# Seed for reproducibility; boxes are placed away from vehicle spawn
rng = random.Random(42)
box_mat = chrono.ChContactMaterialNSC()
box_mat.SetFriction(0.7)
box_mat.SetRestitution(0.05)

placed_boxes = []  # cache: list of added box bodies
for i in range(NUM_BOXES):
    # Random position in annular zone: min 8 m, max BOX_SPREAD_RADIUS from origin
    angle = rng.uniform(0, 2 * math.pi)
    radius = rng.uniform(8.0, BOX_SPREAD_RADIUS)
    bx = radius * math.cos(angle)
    by = radius * math.sin(angle)

    # Random half-sizes
    hx = rng.uniform(BOX_MIN_HALF, BOX_MAX_HALF)
    hy = rng.uniform(BOX_MIN_HALF, BOX_MAX_HALF)
    hz = rng.uniform(BOX_MIN_HALF, BOX_MAX_HALF)

    # ChBodyEasyBox takes full extents (2*half)
    box = chrono.ChBodyEasyBox(2 * hx, 2 * hy, 2 * hz, BOX_MASS, True, True, box_mat)
    box.SetName(f"obstacle_box_{i}")
    box.SetPos(chrono.ChVector3d(bx, by, hz))  # rest with base at z=0
    box.SetFixed(False)
    box.EnableCollision(True)
    system.AddBody(box)
    placed_boxes.append(box)

# === Sensor manager and lidar sensor ===
manager = sens.ChSensorManager(system)

# Add point lights for sensor scene rendering
intensity = 1.0
manager.scene.AddPointLight(
    chrono.ChVector3f(0, 0, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(40, 40, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)

# Lidar sensor mounted on chassis, facing forward/upward
lidar_offset = chrono.ChFramed(
    chrono.ChVector3d(0, 0, 1.5),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
lidar = sens.ChLidarSensor(
    chassis,                               # attach to chassis body
    LIDAR_UPDATE_HZ,                       # update rate (Hz)
    lidar_offset,                          # offset pose on chassis
    LIDAR_H_SAMPLES,                       # horizontal samples
    LIDAR_V_SAMPLES,                       # vertical samples
    LIDAR_H_FOV,                           # horizontal FOV (full 360°)
    LIDAR_MAX_VERT,                        # max vertical angle
    LIDAR_MIN_VERT,                        # min vertical angle
    LIDAR_MAX_RANGE,                       # max range (m)
    sens.LidarBeamShape_RECTANGULAR,       # beam shape
    2,                                     # sample radius
    0.003,                                 # vertical divergence angle
    0.003,                                 # horizontal divergence angle
    sens.LidarReturnMode_STRONGEST_RETURN, # return mode
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(1.0 / LIDAR_UPDATE_HZ)  # collection window = 1/update_rate

# Lidar filter chain (order matters: DI access → PC conversion → visualise → XYZI access)
lidar.PushFilter(sens.ChFilterVisualize(LIDAR_H_SAMPLES, LIDAR_V_SAMPLES, "Raw Lidar Depth"))
lidar.PushFilter(sens.ChFilterDIAccess())           # depth + intensity host access
lidar.PushFilter(sens.ChFilterPCfromDepth())        # depth → XYZI point cloud
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
lidar.PushFilter(sens.ChFilterXYZIAccess())         # XYZI point-cloud host access
manager.AddSensor(lidar)

# === Visualization (HMMWV Irrlicht window) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV with Lidar on Grass Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()                          # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(veh_obj)

# === Driver (interactive — scored-core default for catalog vehicles) ===
driver = veh.ChInteractiveDriver(veh_obj)  # ChInteractiveDriverIRR not present in 9.0.0 build
steering_time = 1.0    # s to reach full steering
throttle_time = 1.0    # s to reach full throttle
braking_time = 0.3     # s to reach full braking
driver.SetSteeringDelta(RENDER_STEP_SIZE / steering_time)
driver.SetThrottleDelta(RENDER_STEP_SIZE / throttle_time)
driver.SetBrakingDelta(RENDER_STEP_SIZE / braking_time)
driver.Initialize()

# === Main loop setup ===


realtime_timer = chrono.ChRealtimeStepTimer()  # real-time pacing for scored core
step_number = 0
frame = 0

try:
    while vis.Run() and system.GetChTime() < SIM_END:
        time = system.GetChTime()  # cache: current sim time

        # Throttled Irrlicht rendering
        if step_number % RENDER_STEPS == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        # Driver inputs
        driver_inputs = driver.GetInputs()


        # Synchronize subsystems (order: driver → terrain → vehicle → vis)
        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        # Advance subsystems
        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        hmmwv.Advance(STEP_SIZE)         # advances the wrapper-owned ChSystem
        vis.Advance(STEP_SIZE)

        # Update sensor manager (exactly once per physics step, after Advance)
        manager.Update()


        step_number += 1
        realtime_timer.Spin(STEP_SIZE)  # real-time pacing

except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise

finally:
    pass
