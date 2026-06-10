"""
Vehicle application simulation: HMMWV on rigid terrain with scene objects and lidar sensor.

System type: NSC (ChSystemNSC, owned by HMMWV_Full wrapper)
Main bodies: HMMWV chassis + wheels, rigid terrain patch, box object, cylinder object
Sensors: ChLidarSensor (3D, 800x300, 360 deg FOV, 100 m range) attached to chassis
Expected behavior: HMMWV starts at (0, -5, 0.4) and drives with steering=0.5, throttle=0.2;
  lidar scans the environment including box and cylinder obstacles placed ahead.
"""

import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.sensor as sens

# === Constants ===
# Geometry / physics parameters (precomputed once)
STEP_SIZE = 1e-3
SIM_END = 10.0
RENDER_FPS = 50.0
render_every = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

TERRAIN_LENGTH = 200.0
TERRAIN_WIDTH = 200.0

# Vehicle spawn
INIT_LOC = chrono.ChVector3d(0, -5, 0.4)
INIT_ROT = chrono.QUNIT

# Scene object positions
BOX_POS = chrono.ChVector3d(0, 0, 0.5)
CYL_POS = chrono.ChVector3d(0, 0, 1.5)

# Lidar parameters
LIDAR_OFFSET = chrono.ChFramed(
    chrono.ChVector3d(0.0, 0, 2),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
LIDAR_H_SAMPLES = 800
LIDAR_V_SAMPLES = 300
LIDAR_H_FOV = 2 * chrono.CH_PI
LIDAR_MAX_VERT = chrono.CH_PI / 12
LIDAR_MIN_VERT = -chrono.CH_PI / 6
LIDAR_MAX_RANGE = 100.0
LIDAR_UPDATE_RATE = 5.0

# === Data paths (mandatory for catalog vehicles) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle setup ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(STEP_SIZE)
hmmwv.Initialize()

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system = hmmwv.GetSystem()                  # ChSystemNSC owned by the wrapper
chassis = hmmwv.GetChassisBody()            # main chassis rigid body  # cache: fetched once, reused
# wheels/spindles: hmmwv.GetVehicle().GetAxle(i) ...; terrain: RigidTerrain patch body below
# joints: suspension + steering links created inside the wrapper

system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

# === Terrain ===
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Scene Objects ===
# Box (1x1x1) with blue texture at (0, 0, 0.5)
box_mat = chrono.ChContactMaterialNSC()
box_mat.SetFriction(0.7)
box_mat.SetRestitution(0.0)
box_body = chrono.ChBodyEasyBox(1.0, 1.0, 1.0, 1000.0, True, True, box_mat)
box_body.SetName("box_object")
box_body.SetPos(BOX_POS)
box_body.SetFixed(True)
try:
    blue_texture = chrono.ChTexture()
    blue_texture.SetTextureFilename(chrono.GetChronoDataFile("textures/blue.png"))
    box_body.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
except (AttributeError, RuntimeError):
    # Fallback: apply blue color to visual shape
    box_vis = box_body.GetVisualShape(0)
    box_vis.SetColor(chrono.ChColor(0.0, 0.0, 1.0))
system.AddBody(box_body)

# Cylinder (r=0.5, h=1) with blue texture at (0, 0, 1.5)
cyl_mat = chrono.ChContactMaterialNSC()
cyl_mat.SetFriction(0.7)
cyl_mat.SetRestitution(0.0)
cyl_body = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.5, 1.0, 1000.0, True, True, cyl_mat)
cyl_body.SetName("cylinder_object")
cyl_body.SetPos(CYL_POS)
cyl_body.SetFixed(True)
try:
    cyl_body.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
except (AttributeError, RuntimeError):
    cyl_vis = cyl_body.GetVisualShape(0)
    cyl_vis.SetColor(chrono.ChColor(0.0, 0.0, 1.0))
system.AddBody(cyl_body)

# === Visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV Vehicle Application with Lidar")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(hmmwv.GetVehicle())

# === Driver setup ===
driver = veh.ChInteractiveDriverIRR(vis)
render_step_size = 1.0 / RENDER_FPS  # precomputed once
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# === Sensor Manager ===
manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(
    chrono.ChVector3f(2, 2.5, 100),
    chrono.ChColor(1.0, 1.0, 1.0),
    500.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(-2, -2.5, 100),
    chrono.ChColor(1.0, 1.0, 1.0),
    500.0,
)

# === Lidar Sensor ===
lidar = sens.ChLidarSensor(
    chassis,                            # body lidar is attached to
    LIDAR_UPDATE_RATE,                  # update rate (Hz)
    LIDAR_OFFSET,                       # offset pose from chassis
    LIDAR_H_SAMPLES,                    # horizontal samples
    LIDAR_V_SAMPLES,                    # vertical channels
    LIDAR_H_FOV,                        # horizontal FOV (rad)
    LIDAR_MAX_VERT,                     # max vertical angle (rad)
    LIDAR_MIN_VERT,                     # min vertical angle (rad)
    LIDAR_MAX_RANGE,                    # max range (m)
    sens.LidarBeamShape_RECTANGULAR,    # beam shape
    2,                                  # sample radius
    0.003,                              # divergence angle
    0.003,                              # horizontal divergence angle
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(1.0 / LIDAR_UPDATE_RATE)

# Lidar filter chain: Depth+Intensity data, XYZI point cloud, and visualization
lidar.PushFilter(sens.ChFilterVisualize(LIDAR_H_SAMPLES, LIDAR_V_SAMPLES, "Raw Lidar Depth"))
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
lidar.PushFilter(sens.ChFilterXYZIAccess())
manager.AddSensor(lidar)

# === Review-only recording setup ===

# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
frame = 0

try:
    while vis.Run() and system.GetChTime() < SIM_END:
        time = system.GetChTime()

        if step_number % render_every == 0:
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
        manager.Update()


        step_number += 1
        realtime_timer.Spin(STEP_SIZE)

except (RuntimeError, ValueError) as exc:  # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
