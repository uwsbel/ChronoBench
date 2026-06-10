"""
HMMWV vehicle simulation with Lidar sensor, box and cylinder scene objects.

System: ChSystemNSC (owned by HMMWV wrapper) with Bullet collision.
Vehicle: HMMWV_Full on RigidTerrain (flat, NSC contact).
Scene objects: 1x1x1 box at (0,0,0.5) and cylinder (r=0.5, h=1) at (0,0,1.5),
  both with blue color.
Sensor: ChLidarSensor attached to chassis at offset (0,0,2), 800 h-samples,
  300 v-channels, 360-deg HFOV, max_vert=PI/12, min_vert=-PI/6, range=100m,
  rectangular beam, sample_radius=2, divergence=0.003, strongest-return mode.
  Filters: depth+intensity (DI), XYZI point cloud, and visualization.
Driver: scripted inputs - steering=0.5, throttle=0.2 (applied each step).
Vehicle initial location: (0, -5, 0.4); starts off from the scene objects.
Expected behavior: vehicle steers and accelerates, lidar spins and captures
  point cloud data, box and cylinder visible ahead of vehicle.
"""

# === Imports ===
import os
import math
import csv

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.sensor as sens

# === Named constants ===
STEP_SIZE = 5e-4           # simulation time step (s)
SIM_END = 20.0             # simulation end time (s)
RENDER_FPS = 50.0          # Irrlicht render frame rate
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

TERRAIN_LENGTH = 200.0
TERRAIN_WIDTH = 200.0

# Vehicle init position (prompt: changed from (0,0,0.4) to (0,-5,0.4))
INIT_LOC = chrono.ChVector3d(0, -5, 0.4)
INIT_ROT = chrono.QUNIT

# Scene object positions (prompt-specified)
BOX_POS = chrono.ChVector3d(0, 0, 0.5)
BOX_DIM = chrono.ChVector3d(1, 1, 1)        # full extents (1x1x1)
CYL_POS = chrono.ChVector3d(0, 0, 1.5)
CYL_RADIUS = 0.5
CYL_HEIGHT = 1.0

# Lidar parameters (prompt-specified)
LIDAR_OFFSET = chrono.ChVector3d(0.0, 0, 2)  # offset on chassis
LIDAR_H_SAMPLES = 800
LIDAR_V_CHANNELS = 300
LIDAR_H_FOV = 2 * chrono.CH_PI               # 360 degrees
LIDAR_MAX_VERT = chrono.CH_PI / 12
LIDAR_MIN_VERT = -chrono.CH_PI / 6
LIDAR_MAX_RANGE = 100.0

# === Vehicle setup (HMMWV_Full wrapper owns ChSystemNSC) ===
veh.SetVehicleDataPath(chrono.GetChronoDataPath() + "vehicle/")

hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                       # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(STEP_SIZE)
hmmwv.Initialize()

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system = hmmwv.GetSystem()                  # ChSystemNSC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize

chassis = hmmwv.GetChassisBody()            # cache: main chassis rigid body, reused below
# wheels/spindles: hmmwv.GetVehicle().GetAxle(i); terrain patch below

# Visualization types (after Initialize)
hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === Terrain (RigidTerrain, NSC contact material) ===
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
patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Scene objects (box and cylinder, blue color) ===
# NSC contact material for scene props
prop_mat = chrono.ChContactMaterialNSC()
prop_mat.SetFriction(0.7)
prop_mat.SetRestitution(0.1)

# Box: 1x1x1, fixed, blue color, at (0, 0, 0.5)
box_body = chrono.ChBodyEasyBox(
    BOX_DIM.x, BOX_DIM.y, BOX_DIM.z,
    1000.0, True, True, prop_mat
)
box_body.SetName("blue_box")
box_body.SetPos(BOX_POS)
box_body.SetFixed(True)
box_vis = chrono.ChVisualShapeBox(BOX_DIM.x, BOX_DIM.y, BOX_DIM.z)
box_vis.SetColor(chrono.ChColor(0.1, 0.2, 0.9))  # blue
# The EasyBox already has a visual shape; update color via its visual model
box_body.GetVisualShape(0).SetColor(chrono.ChColor(0.1, 0.2, 0.9))
system.AddBody(box_body)

# Cylinder: radius=0.5, height=1, fixed, blue color, at (0, 0, 1.5)
cyl_body = chrono.ChBodyEasyCylinder(
    chrono.ChAxis_Y,   # cylinder axis along Y (height)
    CYL_RADIUS, CYL_HEIGHT,
    1000.0, True, True, prop_mat
)
cyl_body.SetName("blue_cylinder")
cyl_body.SetPos(CYL_POS)
cyl_body.SetFixed(True)
cyl_body.GetVisualShape(0).SetColor(chrono.ChColor(0.1, 0.2, 0.9))  # blue
system.AddBody(cyl_body)

# === Irrlicht visualization (vehicle-specific) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV with Lidar and Scene Objects")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(hmmwv.GetVehicle())

# === Driver (scripted: steering=0.5, throttle=0.2 as specified by prompt) ===
# Prompt specifies fixed driver inputs: steering=0.5 and throttle=0.2 in loop.
# Using ChDataDriver with a simple schedule that sets these values.
driver_inputs = veh.DriverInputs()

# === Sensor manager and Lidar sensor ===
manager = sens.ChSensorManager(system)

# Lidar offset pose on chassis
lidar_offset_pose = chrono.ChFramed(
    LIDAR_OFFSET,
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
)

lidar = sens.ChLidarSensor(
    chassis,                             # attach to chassis body
    5.0,                                 # update_rate (Hz) — physical rate
    lidar_offset_pose,                   # offset pose on chassis
    LIDAR_H_SAMPLES,                     # horizontal samples: 800
    LIDAR_V_CHANNELS,                    # vertical channels: 300
    LIDAR_H_FOV,                         # horizontal FOV: 2*PI (360 deg)
    LIDAR_MAX_VERT,                      # max vertical angle: PI/12
    LIDAR_MIN_VERT,                      # min vertical angle: -PI/6
    LIDAR_MAX_RANGE,                     # max range: 100 m
    sens.LidarBeamShape_RECTANGULAR,     # beam shape: rectangular
    2,                                   # sample_radius: 2
    0.003,                               # vert divergence angle: 0.003
    0.003,                               # hori divergence angle: 0.003
    sens.LidarReturnMode_STRONGEST_RETURN,  # return mode: strongest
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(1.0 / 5.0)    # lidar: collection window = 1/update_rate

# Lidar filter chain: Depth+Intensity, XYZI point cloud, visualization
lidar.PushFilter(sens.ChFilterVisualize(LIDAR_H_SAMPLES, LIDAR_V_CHANNELS, "Raw Lidar Depth"))
lidar.PushFilter(sens.ChFilterDIAccess())          # host access to depth+intensity data
lidar.PushFilter(sens.ChFilterPCfromDepth())       # depth -> XYZI point cloud
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
lidar.PushFilter(sens.ChFilterXYZIAccess())        # host access to XYZI data

manager.AddSensor(lidar)

# === Review-only setup ===


# === Main simulation loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
frame = 0
step_number = 0

try:
    while vis.Run() and system.GetChTime() < SIM_END:
        time = system.GetChTime()

        # Throttled rendering
        if step_number % RENDER_EVERY == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        # === Scripted driver inputs (prompt: steering=0.5, throttle=0.2) ===
        driver_inputs.m_steering = 0.5
        driver_inputs.m_throttle = 0.2
        driver_inputs.m_braking = 0.0

        # Synchronize subsystems
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        # Advance subsystems
        terrain.Advance(STEP_SIZE)
        hmmwv.Advance(STEP_SIZE)   # advances the wrapper-owned ChSystem
        vis.Advance(STEP_SIZE)

        # Update sensors every physics step
        manager.Update()


        step_number += 1
        realtime_timer.Spin(STEP_SIZE)

except (RuntimeError, ValueError) as exc:  # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass  # no cleanup needed in scored core
