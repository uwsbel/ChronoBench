"""HMMWV on rigid terrain, published over ROS2 with an attached 2D/3D lidar.

System: ChSystemNSC owned by the veh.HMMWV_Full wrapper (rigid-terrain catalog
vehicle, NSC contact). The full HMMWV (chassis, suspension, wheels, TMEASY tires)
drives on a flat RigidTerrain patch. A fixed visualization box stands beside the
spawn so the lidar returns are visible. A ChSensorManager carries a ChLidarSensor
mounted on the chassis; its depth + point-cloud filter chain feeds a
ChROSLidarHandler that publishes the scan to a ROS2 topic. A ChROSPythonManager
also publishes /clock, the chassis pose (ChROSBodyHandler), and subscribes driver
inputs (ChROSDriverInputsHandler). Expected behavior: the HMMWV drives forward on
the terrain while the lidar sweeps the scene and every handler streams to ROS2.
"""

import os
import math

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import pychrono.ros as chros
import pychrono.irrlicht as chronoirr

# === Named constants === geometry / physics (no bare literals downstream)
TIME_STEP = 2e-3                 # integration step (s)
TIRE_STEP = 1e-3                 # tire substep (s)
SIM_END = 12.0                   # bounded recording length (s)
RENDER_FPS = 50.0                # review render cadence
TERRAIN_LENGTH = 100.0           # rigid terrain patch length (m)
TERRAIN_WIDTH = 100.0            # rigid terrain patch width (m)
TERRAIN_TOP_Z = 0.0              # terrain top surface height (m)
SUSPENSION_REF_HEIGHT = 0.5      # HMMWV chassis-origin height above wheel-bottom
TIRE_RADIUS = 0.46               # HMMWV tire radius (m), for footprint assert
ZTOL = 0.10                      # allowed wheel-bottom clearance vs terrain top

VEH_INIT_X = 0.0
VEH_INIT_Y = 0.0
VEH_INIT_Z = TERRAIN_TOP_Z + SUSPENSION_REF_HEIGHT     # derived spawn height
INIT_LOC = chrono.ChVector3d(VEH_INIT_X, VEH_INIT_Y, VEH_INIT_Z)
INIT_ROT = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization box placed ahead-and-left of the spawn (lidar target, no overlap)
BOX_POS = chrono.ChVector3d(8.0, 3.0, 0.5)
BOX_SIZE = chrono.ChVector3d(1.0, 1.0, 1.0)

# Lidar (3D scanning lidar mounted above the chassis roof)
LIDAR_RATE = 5.0                 # physical Hz (NOT 1/dt)
LIDAR_H_SAMPLES = 800
LIDAR_V_SAMPLES = 300
LIDAR_MAX_RANGE = 100.0

# === Data paths (truth-faithful catalog-vehicle preamble) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())          # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')      # locate vehicle data files

# === Vehicle (wrapper owns the ChSystemNSC + chassis/suspension/wheel bodies) ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)            # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                                 # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
hmmwv.SetTireType(veh.TireModelType_TMEASY)                  # rigid-terrain road tire
hmmwv.SetTireStepSize(TIRE_STEP)
hmmwv.Initialize()

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
sys = hmmwv.GetSystem()                                      # ChSystemNSC owned by the wrapper
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)   # REQUIRED, after Initialize
chassis = hmmwv.GetChassisBody()                            # cache: main chassis rigid body, reused below
chassis.SetName("chassis")
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())       # report total vehicle mass

# Footprint assert — wheels rest on (not through) the terrain after Initialize
veh_obj = hmmwv.GetVehicle()
spindle_world = []
for axle in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_world.append(veh_obj.GetSpindlePos(axle, side))
wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= TERRAIN_TOP_Z - ZTOL, (
    f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs terrain top z={TERRAIN_TOP_Z:.3f}; raise SUSPENSION_REF_HEIGHT"
)

# === Terrain (rigid flat patch on the shared system) ===
terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChContactMaterialNSC()                   # NSC material pairs with NSC method
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, TERRAIN_TOP_Z), chrono.QUNIT),
    TERRAIN_LENGTH, TERRAIN_WIDTH,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.6, 0.6, 0.7))
terrain.Initialize()

# === Visualization box (fixed lidar target beside the spawn) ===
vis_box = chrono.ChBodyEasyBox(BOX_SIZE.x, BOX_SIZE.y, BOX_SIZE.z, 1000.0, True, True, patch_mat)
vis_box.SetPos(BOX_POS)
vis_box.SetFixed(True)
vis_box.SetName("vis_box")
vis_box.GetVisualShape(0).SetColor(chrono.ChColor(0.8, 0.2, 0.2))
sys.Add(vis_box)

# === Visualization === full Irrlicht scene: window + sky + camera + lights
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV + Lidar over ROS2")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(-5, 2.5, 1.5), 9.0, 0.5)   # camera viewpoint
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                                  # vehicle truths use a directional light
vis.AttachVehicle(hmmwv.GetVehicle())

# === Sensor manager + lidar (OptiX; built after the Irrlicht GL context exists) ===
sens_manager = sens.ChSensorManager(sys)
sens_manager.scene.AddPointLight(chrono.ChVector3f(100, 100, 100), chrono.ChColor(1, 1, 1), 5000.0)
sens_manager.scene.AddPointLight(chrono.ChVector3f(-100, -100, 100), chrono.ChColor(1, 1, 1), 5000.0)

lidar_offset = chrono.ChFramed(
    chrono.ChVector3d(0.0, 0.0, 1.8),                       # above the chassis roof
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
lidar = sens.ChLidarSensor(
    chassis,                                                # attached to the chassis
    LIDAR_RATE,                                             # update_rate (Hz)
    lidar_offset,                                           # offset pose
    LIDAR_H_SAMPLES,                                        # horizontal samples
    LIDAR_V_SAMPLES,                                        # vertical samples
    2 * chrono.CH_PI,                                       # horizontal_fov (rad)
    chrono.CH_PI / 12,                                     # max_vert_angle
    -chrono.CH_PI / 6,                                     # min_vert_angle
    LIDAR_MAX_RANGE,                                        # max_range
    sens.LidarBeamShape_RECTANGULAR,
    2,                                                     # sample_radius
    0.003,                                                 # vert divergence_angle
    0.003,                                                 # hori divergence_angle
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(1.0 / LIDAR_RATE)
# Lidar filter chain (ORDER MATTERS) — depth -> XYZI point cloud for the ROS handler.
lidar.PushFilter(sens.ChFilterDIAccess())                  # host access to depth+intensity
lidar.PushFilter(sens.ChFilterPCfromDepth())               # depth -> XYZI point cloud
lidar.PushFilter(sens.ChFilterXYZIAccess())                # host access to XYZI (fed to ROS)
sens_manager.AddSensor(lidar)

# === Driver (interactive, truth-faithful; ROS subscribes inputs through it) ===
RENDER_STEP_SIZE = 1.0 / RENDER_FPS
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(RENDER_STEP_SIZE / 1.0)
driver.SetThrottleDelta(RENDER_STEP_SIZE / 1.0)
driver.SetBrakingDelta(RENDER_STEP_SIZE / 0.3)
driver.Initialize()

# === ROS publishing (clock first, then body / driver-inputs / lidar) ===
ros_manager = chros.ChROSPythonManager()
ros_manager.RegisterHandler(chros.ChROSClockHandler())     # /clock first
ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, chassis, "~/output/hmmwv/state"))
ros_manager.RegisterHandler(chros.ChROSDriverInputsHandler(25, driver, "~/input/driver_inputs"))
ros_manager.RegisterHandler(chros.ChROSLidarHandler(lidar, "~/output/lidar/data"))
ros_manager.Initialize()                                   # exactly once, after all handlers

# === Main loop === drive forward, sweep lidar, publish every handler over ROS2
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once

os.makedirs("cam", exist_ok=True)                          # guard against missing output dir
realtime_timer = chrono.ChRealtimeStepTimer()              # spin so wall-clock ~ sim time
frame = 0
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        time = sys.GetChTime()

        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        driver_inputs = driver.GetInputs()

        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(TIME_STEP)
        terrain.Advance(TIME_STEP)
        hmmwv.Advance(TIME_STEP)                           # advances the wrapper-owned system
        vis.Advance(TIME_STEP)

        sens_manager.Update()                              # pump the lidar buffer
        if not ros_manager.Update(time, TIME_STEP):        # publish to ROS2 — break on shutdown
            break

        realtime_timer.Spin(TIME_STEP)
except (RuntimeError, ValueError) as exc:                  # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass

# === Post-processing === assemble review videos + plots, then drop frame PNGs
