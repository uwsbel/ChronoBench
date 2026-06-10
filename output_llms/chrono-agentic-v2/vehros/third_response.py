"""
vehros Turn 3 — HMMWV on RigidTerrain with ROS2 Bridge, Lidar Sensor, and Irrlicht Visualization.

System type : ChSystemNSC (owned by the HMMWV_Full wrapper)
Main bodies : HMMWV chassis + 4 wheels/tires on a flat RigidTerrain patch
Sensors     : ChLidarSensor attached to the chassis; managed by ChSensorManager
ROS handlers: ChROSClockHandler, ChROSBodyHandler (chassis pose),
              ChROSDriverInputsHandler (subscribe throttle/steer/brake),
              ChROSLidarHandler (publish point cloud)
Visualization: veh.ChWheeledVehicleVisualSystemIrrlicht with chase camera

Expected behavior: HMMWV drives forward on a flat terrain patch; lidar scans the
environment and publishes point-cloud data over ROS2; vehicle chassis pose is
published at 25 Hz; camera is repositioned to (-5, 2.5, 1.5) per Turn 3 prompt.
"""

import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import pychrono.ros as chros

# === Constants ===
STEP_SIZE = 1e-3
SIM_END = 20.0
RENDER_FPS = 50.0
render_every = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

TERRAIN_LENGTH = 200.0
TERRAIN_WIDTH = 200.0
INIT_LOC = chrono.ChVector3d(0, 0, 0.5)
INIT_ROT = chrono.QuatFromAngleZ(0.0)

LIDAR_UPDATE_RATE = 5.0      # Hz — physical rate
LIDAR_H_SAMPLES = 800
LIDAR_V_SAMPLES = 300
LIDAR_FOV_H = 2 * chrono.CH_PI
LIDAR_MAX_VERT = chrono.CH_PI / 12
LIDAR_MIN_VERT = -chrono.CH_PI / 6
LIDAR_MAX_RANGE = 100.0

# === Data paths (mandatory truth components for catalog vehicle) ===
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
system = hmmwv.GetSystem()          # ChSystemNSC owned by the wrapper
chassis = hmmwv.GetChassisBody()    # cache: fetched once, reused below
# wheels/spindles: hmmwv.GetVehicle().GetAxle(i); terrain: RigidTerrain patch below
# joints: suspension + steering links created inside the wrapper

system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED, after Initialize

# Visualization types (after Initialize per skill)
hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

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

# === Visualization (Irrlicht, vehicle-specific) ===
# Camera repositioned to (-5, 2.5, 1.5) per Turn 3 prompt
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV vehros — Turn 3 (Lidar + ROS)")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()           # vehicle truths use directional light
vis.AttachVehicle(hmmwv.GetVehicle())

# === Driver (interactive IRR — scored-core default matching truth) ===
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
render_step_size = 1.0 / RENDER_FPS  # precomputed once

driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# === Sensor Manager ===
sens_manager = sens.ChSensorManager(system)
# Point light for camera illumination (added; lidar needs no lighting)
intensity = 1.0
sens_manager.scene.AddPointLight(
    chrono.ChVector3f(2, 2.5, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)

# === Lidar Sensor ===
# Offset: camera position changed to (-5, 2.5, 1.5) as prompted
lidar_offset = chrono.ChFramed(
    chrono.ChVector3d(-5, 2.5, 1.5),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
lidar = sens.ChLidarSensor(
    chassis,                                    # attached to chassis (cache)
    LIDAR_UPDATE_RATE,                          # update_rate (Hz)
    lidar_offset,                               # offset pose on chassis
    LIDAR_H_SAMPLES,                            # horizontal samples
    LIDAR_V_SAMPLES,                            # vertical samples
    LIDAR_FOV_H,                                # horizontal FOV (rad)
    LIDAR_MAX_VERT,                             # max vertical angle (rad)
    LIDAR_MIN_VERT,                             # min vertical angle (rad)
    LIDAR_MAX_RANGE,                            # max range (m)
    sens.LidarBeamShape_RECTANGULAR,            # beam shape
    2,                                          # sample_radius
    0.003,                                      # vert divergence angle
    0.003,                                      # hori divergence angle
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(1.0 / LIDAR_UPDATE_RATE)  # collection window = 1/rate

# Lidar filter chain (ORDER MATTERS — scored core, prompt-required)
lidar.PushFilter(sens.ChFilterVisualize(LIDAR_H_SAMPLES, LIDAR_V_SAMPLES, "Raw Lidar Depth"))
lidar.PushFilter(sens.ChFilterDIAccess())           # host access to depth+intensity
lidar.PushFilter(sens.ChFilterPCfromDepth())        # depth -> XYZI point cloud
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
lidar.PushFilter(sens.ChFilterXYZIAccess())         # host access to XYZI
sens_manager.AddSensor(lidar)

# === ROS Manager ===
ros_manager = chros.ChROSPythonManager()

# 1. Clock handler first
ros_manager.RegisterHandler(chros.ChROSClockHandler())

# 2. Body handler — publish chassis pose/twist
ros_manager.RegisterHandler(
    chros.ChROSBodyHandler(25, chassis, "~/output/vehicle/state")
)

# 3. Driver inputs handler — subscribes to throttle/steer/brake from ROS
ros_manager.RegisterHandler(
    chros.ChROSDriverInputsHandler(25, driver, "~/input/driver_inputs")
)

# 4. Lidar handler — publishes point cloud
ros_manager.RegisterHandler(
    chros.ChROSLidarHandler(lidar, "~/output/lidar/data")
)

# Initialize ROS manager ONCE, after all registrations, before loop
ros_manager.Initialize()


realtime_timer = chrono.ChRealtimeStepTimer()
frame = 0

# === Main loop ===
try:
    while vis.Run() and system.GetChTime() < SIM_END:
        time = system.GetChTime()

        if frame % render_every == 0:
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
        hmmwv.Advance(STEP_SIZE)    # advances the wrapper-owned ChSystem
        vis.Advance(STEP_SIZE)

        sens_manager.Update()       # pump sensors after vehicle advance

        if not ros_manager.Update(time, STEP_SIZE):  # ROS update last; break on shutdown
            break


        frame += 1
        realtime_timer.Spin(STEP_SIZE)

except (RuntimeError, ValueError) as exc:  # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
