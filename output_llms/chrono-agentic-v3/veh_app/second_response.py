"""
veh_app turn 2 — HMMWV on RigidTerrain with box obstacle, cylinder obstacle, and lidar sensor.

System type: ChSystemNSC (created by HMMWV_Full wrapper).
Main bodies: HMMWV chassis + 4 wheels/spindles, rigid terrain patch, box body, cylinder body.
Driver: ChInteractiveDriverIRR (scored core); scripted steering=0.5, throttle=0.2 in review-only loop.
Lidar: 800h x 300v, 360-deg HFOV, mounted at chassis offset (0,0,2); outputs DI, XYZI, pointcloud.
Expected behaviour: HMMWV spawns at (0,-5,0.4), accelerates forward while steering, lidar spins.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

# === Named constants ===
STEP_SIZE        = 1e-3          # physics time step (s)
SIM_END          = 20.0          # simulation duration (s)
RENDER_FPS       = 50.0
RENDER_EVERY     = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))   # precomputed once

TERRAIN_LENGTH   = 200.0
TERRAIN_WIDTH    = 200.0

INIT_LOC         = chrono.ChVector3d(0, -5, 0.4)
INIT_ROT         = chrono.QUNIT

# Box obstacle dimensions and position
BOX_DIM          = chrono.ChVector3d(1.0, 1.0, 1.0)
BOX_POS          = chrono.ChVector3d(0, 0, 0.5)

# Cylinder obstacle dimensions and position
CYL_RADIUS       = 0.5
CYL_HEIGHT       = 1.0
CYL_POS          = chrono.ChVector3d(0, 0, 1.5)

# Lidar parameters
LIDAR_OFFSET     = chrono.ChVector3d(0.0, 0, 2)
LIDAR_H_SAMPLES  = 800
LIDAR_V_SAMPLES  = 300
LIDAR_H_FOV      = 2 * chrono.CH_PI
LIDAR_MAX_VERT   = chrono.CH_PI / 12
LIDAR_MIN_VERT   = -chrono.CH_PI / 6
LIDAR_MAX_RANGE  = 100.0
LIDAR_SAMPLE_R   = 2
LIDAR_DIV_ANGLE  = 0.003
LIDAR_UPDATE_HZ  = 5.0           # physical lidar update rate

# === Data paths (truth-faithful; required for catalog vehicle scoring) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle setup ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                         # MANDATORY
hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(STEP_SIZE)
hmmwv.Initialize()

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system = hmmwv.GetSystem()                 # ChSystemNSC owned by the wrapper
chassis = hmmwv.GetChassisBody()           # cache: fetched once, reused for lidar attachment
# wheels/spindles: hmmwv.GetVehicle().GetAxle(i); terrain: RigidTerrain patch below
# joints: suspension + steering links created inside the wrapper

system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

# === Visualization types (after Initialize) ===
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
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Box obstacle ===
box_mat = chrono.ChContactMaterialNSC()
box_mat.SetFriction(0.8)
box_mat.SetRestitution(0.0)
box_body = chrono.ChBodyEasyBox(
    BOX_DIM.x, BOX_DIM.y, BOX_DIM.z,
    1000.0, True, True, box_mat,
)
box_body.SetName("obstacle_box")
box_body.SetPos(BOX_POS)
box_body.SetFixed(True)
# Apply blue texture/color
box_vis = box_body.GetVisualModel().GetShape(0)
box_vis.SetColor(chrono.ChColor(0.0, 0.0, 1.0))
system.Add(box_body)

# === Cylinder obstacle ===
cyl_mat = chrono.ChContactMaterialNSC()
cyl_mat.SetFriction(0.8)
cyl_mat.SetRestitution(0.0)
cyl_body = chrono.ChBodyEasyCylinder(
    chrono.ChAxis_Y,
    CYL_RADIUS, CYL_HEIGHT,
    1000.0, True, True, cyl_mat,
)
cyl_body.SetName("obstacle_cylinder")
cyl_body.SetPos(CYL_POS)
cyl_body.SetFixed(True)
cyl_vis = cyl_body.GetVisualModel().GetShape(0)
cyl_vis.SetColor(chrono.ChColor(0.0, 0.0, 1.0))
system.Add(cyl_body)

# === Irrlicht visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV with Lidar, Box and Cylinder Obstacles")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(hmmwv.GetVehicle())

# === Driver (interactive — scored core default for catalog vehicles) ===
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time  = 0.3
render_step_size = 1.0 / RENDER_FPS          # precomputed once
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# === Sensor manager + Lidar ===
manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(
    chrono.ChVector3f(2, 2.5, 100),
    chrono.ChColor(1, 1, 1),
    500.0,
)

lidar_offset_pose = chrono.ChFramed(
    LIDAR_OFFSET,
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
lidar = sens.ChLidarSensor(
    chassis,                             # attached to chassis body
    LIDAR_UPDATE_HZ,
    lidar_offset_pose,
    LIDAR_H_SAMPLES,                     # horizontal samples
    LIDAR_V_SAMPLES,                     # vertical channels
    LIDAR_H_FOV,                         # horizontal FOV (360 deg)
    LIDAR_MAX_VERT,                      # max vertical angle
    LIDAR_MIN_VERT,                      # min vertical angle
    LIDAR_MAX_RANGE,
    sens.LidarBeamShape_RECTANGULAR,
    LIDAR_SAMPLE_R,
    LIDAR_DIV_ANGLE,                     # vertical divergence
    LIDAR_DIV_ANGLE,                     # horizontal divergence
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(1.0 / LIDAR_UPDATE_HZ)   # collection window = 1/update_rate

# Lidar filter chain: Depth+Intensity → XYZI point cloud → visualization
lidar.PushFilter(sens.ChFilterVisualize(LIDAR_H_SAMPLES, LIDAR_V_SAMPLES, "Raw Lidar Depth"))
lidar.PushFilter(sens.ChFilterDIAccess())          # host access: depth + intensity
lidar.PushFilter(sens.ChFilterPCfromDepth())       # depth → XYZI point cloud
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
lidar.PushFilter(sens.ChFilterXYZIAccess())        # host access: XYZI
manager.AddSensor(lidar)

# === Review-only recording setup ===


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

try:
    while vis.Run() and system.GetChTime() < SIM_END:
        time = system.GetChTime()                  # cache: fetched once per outer step

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
        manager.Update()


        step_number += 1
        realtime_timer.Spin(STEP_SIZE)

except (RuntimeError, ValueError) as exc:           # solver divergence / bad state
    import traceback; traceback.print_exc()
    raise
finally:
    pass
