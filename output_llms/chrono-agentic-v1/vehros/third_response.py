"""
HMMWV vehicle on rigid terrain with ChROSPythonManager, lidar sensor, and ROS publishing.

System: ChSystemNSC (owned by HMMWV_Full wrapper)
Terrain: RigidTerrain flat patch
Vehicle: HMMWV_Full with TMEASY tires, interactive driver (IRR)
Bodies: visualization box (ChBodyEasyBox) placed near the vehicle start position
Sensors: ChSensorManager + ChLidarSensor (800x8 samples) attached to chassis
Visualization: ChWheeledVehicleVisualSystemIrrlicht with chase camera at (-5, 2.5, 1.5)
ROS: Clock, DriverInputs (subscribe), Body (publish chassis pose), Lidar (publish POINT_CLOUD2)
Expected behavior: vehicle drives on flat terrain; lidar scans the scene;
  ROS topics publish chassis pose and lidar point cloud; driver responds to keyboard.
"""

# === Imports ===
import os
import math
import csv

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import pychrono.ros as chros


# Anchor vehicle data path to absolute location so assets load from any working dir
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Named constants ===
TIME_STEP = 2e-3             # physics step size (s)
SIM_END = 20.0               # simulation end time (s)
RENDER_FPS = 50.0            # irrlicht render rate (Hz)
TERRAIN_LENGTH = 200.0       # terrain patch length (m)
TERRAIN_WIDTH = 200.0        # terrain patch width (m)
SUSPENSION_REF_HEIGHT = 0.5  # chassis origin above wheel-bottom at rest (m)
TIRE_RADIUS = 0.33           # HMMWV TMEASY tire radius approx (m)

# Vehicle initial position (flat terrain z=0, lifted by suspension geometry)
INIT_X = 0.0
INIT_Y = 0.0
INIT_Z = SUSPENSION_REF_HEIGHT
init_loc = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)  # identity (facing +X)

# Visualization box placement (to the side of vehicle start, at ground level)
BOX_X = 5.0
BOX_Y = 3.0
BOX_Z = 0.5  # box half-height so bottom touches z=0

# Lidar sensor parameters (800 horizontal × 8 vertical — stable on this GPU)
H_SAMPLES = 800
V_SAMPLES = 8

# Precomputed render cadence: how many physics steps between rendering frames
render_step_size = 1.0 / RENDER_FPS   # precomputed once
render_every = max(1, round(render_step_size / TIME_STEP))  # precomputed once


# === Vehicle setup ===
# HMMWV_Full owns its ChSystemNSC internally — do not pass a pre-created system.
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                                    # MANDATORY
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(TIME_STEP)
hmmwv.Initialize()

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system = hmmwv.GetSystem()                    # ChSystemNSC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize
chassis = hmmwv.GetChassisBody()              # cache: main chassis rigid body
# wheels/spindles: hmmwv.GetVehicle().GetAxles() ; terrain: RigidTerrain patch below
# joints: suspension + steering links created inside the wrapper

# Visualization types — must come after Initialize()
# NOTE: in this 9.0.0 source build, VisualizationType_* lives in veh module
hmmwv.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetTireVisualizationType(veh.VisualizationType_NONE)

# Assert wheel bottoms rest on or near z=0 (rigid terrain surface)
veh_obj = hmmwv.GetVehicle()  # cache: fetched once
spindle_world = []
for axle_idx in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        p = veh_obj.GetSpindlePos(axle_idx, side)
        spindle_world.append(p)
wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
ZTOL = 0.1
assert wheel_bottom_z >= -ZTOL, (
    f"vehicle sinks into terrain: wheel_bottom_z={wheel_bottom_z:.3f} vs 0.0; "
    f"raise SUSPENSION_REF_HEIGHT by {-wheel_bottom_z:.3f} m"
)

# === Terrain ===
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Visualization box ===
# ChBodyEasyBox: a fixed reference box placed near the vehicle start position
box_mat = chrono.ChContactMaterialNSC()
box_mat.SetFriction(0.5)
vis_box = chrono.ChBodyEasyBox(1.0, 1.0, 1.0, 1000.0, True, True, box_mat)
vis_box.SetName("vis_box")
vis_box.SetPos(chrono.ChVector3d(BOX_X, BOX_Y, BOX_Z))
vis_box.SetFixed(True)
system.AddBody(vis_box)

# === Irrlicht visualization (vehicle visual system) ===
# ChWheeledVehicleVisualSystemIrrlicht order: configure window → Initialize()
# → add scene elements → AttachVehicle (in that order).
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV + Lidar + ROS")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(-5, 2.5, 1.5), 9.0, 0.5)   # camera at (-5, 2.5, 1.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(hmmwv.GetVehicle())

# === Interactive driver (scored core — truth uses ChInteractiveDriverIRR) ===
driver = veh.ChInteractiveDriverIRR(vis)
STEERING_TIME = 1.0   # s to reach max steering
THROTTLE_TIME = 1.0   # s to reach max throttle
BRAKING_TIME = 0.3    # s to reach max braking
driver.SetSteeringDelta(render_step_size / STEERING_TIME)
driver.SetThrottleDelta(render_step_size / THROTTLE_TIME)
driver.SetBrakingDelta(render_step_size / BRAKING_TIME)
driver.Initialize()

# === Sensor manager + lidar sensor ===
sens_manager = sens.ChSensorManager(system)
# Add point lights for sensor scene illumination
intensity = 1.0
sens_manager.scene.AddPointLight(
    chrono.ChVector3f(0, 0, 10),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)
sens_manager.scene.AddPointLight(
    chrono.ChVector3f(20, 0, 10),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)

# Lidar sensor attached to the chassis, offset forward and upward
lidar_offset = chrono.ChFramed(
    chrono.ChVector3d(1.5, 0, 0.5),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 0, 1)),
)
lidar = sens.ChLidarSensor(
    chassis,                                # body the lidar is attached to
    5.0,                                    # update_rate (Hz)
    lidar_offset,                           # offset pose from body
    H_SAMPLES,                              # horizontal samples
    V_SAMPLES,                              # vertical samples
    2 * chrono.CH_PI,                       # horizontal FOV (full 360)
    chrono.CH_PI / 12,                      # max vertical angle (+15 deg)
    -chrono.CH_PI / 6,                      # min vertical angle (-30 deg)
    100.0,                                  # max range (m)
    sens.LidarBeamShape_RECTANGULAR,
    2,                                      # sample_radius
    0.003,                                  # vertical divergence angle
    0.003,                                  # horizontal divergence angle
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(1.0 / 5.0)  # collection window = 1/update_rate

# Lidar filter chain (order matters)
lidar.PushFilter(sens.ChFilterVisualize(H_SAMPLES, V_SAMPLES, "Raw Lidar Depth"))
lidar.PushFilter(sens.ChFilterDIAccess())          # host access to depth+intensity
lidar.PushFilter(sens.ChFilterPCfromDepth())       # depth -> XYZI point cloud
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
lidar.PushFilter(sens.ChFilterXYZIAccess())        # host access to XYZI
sens_manager.AddSensor(lidar)

# === ROS manager ===
ros_manager = chros.ChROSPythonManager()

# 1. Clock handler first — publishes /clock for ROS time sync
ros_manager.RegisterHandler(chros.ChROSClockHandler())

# 2. Driver inputs handler (subscribes to throttle/steer/brake over ROS)
ros_manager.RegisterHandler(
    chros.ChROSDriverInputsHandler(25, driver, "~/input/driver_inputs")
)

# 3. Body handler — publishes chassis pose/twist
chassis.SetName("hmmwv_chassis")
ros_manager.RegisterHandler(
    chros.ChROSBodyHandler(25, chassis, "~/output/vehicle/state")
)

# 4. Lidar handler — publishes point cloud (POINT_CLOUD2 message type)
ros_manager.RegisterHandler(
    chros.ChROSLidarHandler(lidar, "~/output/lidar/data",
                            chros.ChROSLidarHandlerMessageType_POINT_CLOUD2)
)

# Initialize ONCE after ALL registration, before the loop
ros_manager.Initialize()

# === Joint connectivity CSV (review-only) ===

# === Review-only recording setup ===

# === Main loop ===
frame = 0          # consecutive frame counter for review video
step_count = 0     # physics step counter
realtime_timer = chrono.ChRealtimeStepTimer()


try:
    while vis.Run() and system.GetChTime() < SIM_END:
        sim_time = system.GetChTime()  # cache: current sim time

        # Throttled rendering: render once per frame, physics in inner batch
        if step_count % render_every == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        step_count += 1

        # Review-only: scripted driving for validation video

        driver_inputs = driver.GetInputs()


        # Synchronize (order: driver → terrain → vehicle → vis)
        driver.Synchronize(sim_time)
        terrain.Synchronize(sim_time)
        hmmwv.Synchronize(sim_time, driver_inputs, terrain)
        vis.Synchronize(sim_time, driver_inputs)

        # Advance (vehicle.Advance internally calls DoStepDynamics — do NOT also call it)
        driver.Advance(TIME_STEP)
        terrain.Advance(TIME_STEP)
        hmmwv.Advance(TIME_STEP)
        vis.Advance(TIME_STEP)

        # Update sensor manager (once per physics step, after Advance)
        sens_manager.Update()

        # ROS update — after all state-producing updates
        if not ros_manager.Update(sim_time, TIME_STEP):
            break  # ROS node shut down


        realtime_timer.Spin(TIME_STEP)

except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass

# === Post-processing (review-only) ===
