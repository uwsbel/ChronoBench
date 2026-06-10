"""HMMWV driving over an SCM (Bekker-Wong soft-soil) hill with onboard lidar.

System type: NSC vehicle wrapper system (ChSystemNSC owned by veh.HMMWV_Full),
contact via Bullet collision. The HMMWV uses a TMEASY tire model (required so the
vehicle actually drives on deformable SCM soil) and rides over a heightmap-based
SCM "bump" hill terrain. Five fixed box obstacles are scattered across the terrain
as range targets, and a 2D lidar sensor rides on the chassis, visualizing its raw
depth and reconstructed point cloud each update.

Expected behavior: the soft soil deforms (leaves ruts) under the wheels, the
vehicle climbs the bump, the lidar sweeps the obstacles + terrain, and the
sensor preview windows show the live depth + point cloud.
"""

import math
import os
import numpy as np
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens

# === Constants === geometry / physics / sim control (no bare literals downstream)
TIME_STEP = 2e-3                 # SCM is stiff — a small step keeps the soil stable
TIRE_STEP = 1e-3                 # TMEASY tire sub-step (required on SCM)
SIM_END = 6.0                    # seconds of simulated driving
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))            # precomputed once

SCM_LENGTH = 40.0                # heightmap patch extent (m), matches bump64 footprint
SCM_WIDTH = 40.0
SCM_H_MIN = -1.0                 # bump heightmap min/max elevation (m)
SCM_H_MAX = 1.0
SCM_RES = 0.05                   # SCM grid resolution (m)

INIT_LOC = chrono.ChVector3d(-8.0, 0.0, 1.0)   # spawn behind the bump, on the soil
INIT_ROT = chrono.ChQuaterniond(1, 0, 0, 0)

NUM_OBSTACLES = 5                # range targets scattered over the terrain
LIDAR_HSAMPLES = 800            # horizontal beams for the 2D scan
LIDAR_VSAMPLES = 1              # 2D lidar -> single vertical sample
LIDAR_RATE = 5.0                # physical lidar update rate (Hz)
LIDAR_RANGE = 100.0             # max range (m)

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
# The wrapper builds and owns a ChSystemNSC plus the chassis, spindles, suspension
# and steering links internally; we fetch the handles we need into named locals.
chrono.SetChronoDataPath(chrono.GetChronoDataPath())          # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')      # locate vehicle data files

hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)            # SCM / deformable terrain uses SMC
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                                 # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
hmmwv.SetTireType(veh.TireModelType_TMEASY)                  # TMEASY required to drive on SCM
hmmwv.SetTireStepSize(TIRE_STEP)
hmmwv.Initialize()

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

system = hmmwv.GetSystem()                                   # ChSystemNSC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED, after Initialize
chassis = hmmwv.GetChassisBody()              # cache: main chassis rigid body, reused every step
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())        # report total vehicle mass

# === Terrain === SCM soft soil initialized from the shipped "bump" heightmap (the hill)
terrain = veh.SCMTerrain(system)
terrain.SetSoilParameters(
    2e6,    # Bekker_Kphi   — frictional modulus (Pa)
    0,      # Bekker_Kc     — cohesive modulus
    1.1,    # Bekker_n      — exponent (soft soil)
    0,      # Mohr_cohesion — cohesive limit (Pa)
    30,     # Mohr_friction — friction angle (deg)
    0.01,   # Janosi_shear  — shear coefficient (m)
    2e8,    # elastic_K     — elastic stiffness (Pa/m)
    3e4,    # damping_R     — vertical damping (Pa s/m)
)
# Moving patch on the CHASSIS (stable OOBB) so only soil near the vehicle updates.
terrain.AddMovingPatch(chassis, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(5, 3, 1))
terrain.Initialize(veh.GetDataFile("terrain/height_maps/bump64.bmp"),
                   SCM_LENGTH, SCM_WIDTH, SCM_H_MIN, SCM_H_MAX, SCM_RES)
terrain.SetMeshWireframe(False)
terrain.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)

# === Tire collision cylinders === TMEASY tires need explicit collision shapes for SCM ray-casts
tire_rad = hmmwv.GetVehicle().GetAxles()[0].m_wheels[0].GetTire().GetRadius()
tire_w = hmmwv.GetVehicle().GetAxles()[0].m_wheels[0].GetTire().GetWidth()
tire_mat = chrono.ChContactMaterialSMC()
tire_mat.SetFriction(0.9)
tire_mat.SetRestitution(0.1)
TIRE_FAMILY = 1
for axle in hmmwv.GetVehicle().GetAxles():
    for iw in range(2):
        spindle = axle.m_wheels[iw].GetSpindle()
        spindle.AddCollisionShape(
            chrono.ChCollisionShapeCylinder(tire_mat, tire_rad + 0.04, tire_w),
            chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(math.pi / 2)),
        )
        spindle.EnableCollision(True)
        sp_cm = spindle.GetCollisionModel()
        sp_cm.SetFamily(TIRE_FAMILY)
        sp_cm.DisallowCollisionsWith(TIRE_FAMILY)
system.GetCollisionSystem().BindAll()         # rebuild collision models after shape edits

# === Footprint check === confirm wheels start on (not through) the terrain
veh_obj = hmmwv.GetVehicle()
spindle_world = [veh_obj.GetSpindlePos(a, side)
                 for a in range(veh_obj.GetNumberAxles()) for side in (veh.LEFT, veh.RIGHT)]
wheel_bottom_z = min(p.z for p in spindle_world) - tire_rad
assert wheel_bottom_z >= SCM_H_MIN - 0.5, (
    f"wheels start below terrain floor: wheel bottom z={wheel_bottom_z:.3f}")

# === Obstacles === five fixed box range targets scattered over the terrain
rng = np.random.default_rng(2)                # deterministic placement for reproducibility
obstacle_mat = chrono.ChContactMaterialSMC()
obstacle_mat.SetFriction(0.8)
obstacle_mat.SetRestitution(0.0)
obstacles = []
for i in range(NUM_OBSTACLES):
    ox = float(rng.uniform(-6.0, 12.0))
    oy = float(rng.uniform(-8.0, 8.0))
    box = chrono.ChBodyEasyBox(1.0, 1.0, 1.0, 1000.0, True, True, obstacle_mat)
    box.SetName(f"obstacle_{i}")
    box.SetPos(chrono.ChVector3d(ox, oy, SCM_H_MAX + 0.5))   # rest above soil so lidar sees them
    box.SetFixed(True)
    box.EnableCollision(True)
    system.AddBody(box)
    obstacles.append(box)

# === Visualization === full vehicle Irrlicht scene: window + sky + chase camera + light
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV on SCM hill with lidar")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                     # vehicle truths use a directional light
vis.AttachVehicle(hmmwv.GetVehicle())

# === Driver === interactive driver bound to the visual system (catalog-vehicle default)
driver = veh.ChInteractiveDriverIRR(vis)
render_step_size = 1.0 / RENDER_FPS
driver.SetSteeringDelta(render_step_size / 1.0)
driver.SetThrottleDelta(render_step_size / 1.0)
driver.SetBrakingDelta(render_step_size / 0.3)
driver.Initialize()

# === Sensors === lidar riding on the chassis, scanning the obstacles + hill
manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(1, 1, 1), 500.0)
lidar_offset = chrono.ChFramed(
    chrono.ChVector3d(0.0, 0.0, 1.8),                         # mounted atop the chassis
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
lidar = sens.ChLidarSensor(
    chassis,                          # rides on the chassis body
    LIDAR_RATE,                       # update_rate (Hz) — physical rate
    lidar_offset,                     # offset pose
    LIDAR_HSAMPLES,                   # horizontal samples
    LIDAR_VSAMPLES,                   # vertical samples (1 -> 2D lidar)
    2 * chrono.CH_PI,                 # horizontal fov (rad)
    0.0,                              # max vertical angle (2D -> 0)
    0.0,                              # min vertical angle (2D -> 0)
    LIDAR_RANGE,                      # max range (m)
    sens.LidarBeamShape_RECTANGULAR,
    2,                                # sample radius
    0.003,                            # vertical divergence angle
    0.003,                            # horizontal divergence angle
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(1.0 / LIDAR_RATE)   # lidar collection window = 1 / update_rate
lidar.PushFilter(sens.ChFilterVisualize(LIDAR_HSAMPLES, LIDAR_VSAMPLES, "Raw Lidar Depth"))
lidar.PushFilter(sens.ChFilterDIAccess())             # host access to depth+intensity
lidar.PushFilter(sens.ChFilterPCfromDepth())          # depth -> XYZI point cloud
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
lidar.PushFilter(sens.ChFilterXYZIAccess())           # host access to XYZI
manager.AddSensor(lidar)

# === Main loop === throttled render; SCM + tires advance each step; lidar pumped each step
os.makedirs("cam", exist_ok=True)             # guard against missing output dir

realtime_timer = chrono.ChRealtimeStepTimer()
while vis.Run() and system.GetChTime() < SIM_END:
    time = system.GetChTime()

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
    hmmwv.Advance(TIME_STEP)                  # advances the wrapper-owned system
    vis.Advance(TIME_STEP)
    manager.Update()                          # pump the lidar once per physics step


    realtime_timer.Spin(TIME_STEP)            # spin so wall-clock matches sim time

# === Post-processing === assemble review video + physics table, then drop frame dirs
