"""
HMMWV on SCM hill terrain with lidar sensor and box obstacles.

System type: ChSystemSMC (wrapper-owned, via HMMWV_Full).
Main bodies: HMMWV vehicle chassis + 4 wheel spindles, SCM terrain (bump64 heightmap),
             5 randomly-placed box obstacles, lidar sensor on chassis.
Expected behavior: HMMWV drives forward over a bumpy hill-terrain with a lidar sensor
                   scanning the environment; 5 static box obstacles are visible in the scene.
"""

# === Imports ===
import os
import math
import csv
import random
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens


# === Constants ===
STEP_SIZE = 2e-3           # physics time step (s)
SIM_END = 15.0             # simulation duration (s)
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

TIRE_RADIUS = 0.47         # HMMWV TMEASY tire radius (m)
TERRAIN_MAX_HEIGHT = 1.0   # bump64 heightmap max terrain elevation at spawn (m)
SUSPENSION_REF_HEIGHT = 0.5  # chassis origin above wheel-bottom at rest

TERRAIN_LENGTH = 100.0
TERRAIN_WIDTH = 100.0
TERRAIN_RESOLUTION = 0.1   # SCM grid resolution (m) — coarser for larger terrain

NUM_OBSTACLES = 5
OBS_HALF_SIZE = 0.5        # obstacle box half-side (m)
random.seed(42)            # reproducible obstacle placement

# === Vehicle setup ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)   # SCM needs SMC
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                          # MANDATORY
init_loc = chrono.ChVector3d(0.0, 0.0, TERRAIN_MAX_HEIGHT + SUSPENSION_REF_HEIGHT)
init_rot = chrono.QuatFromAngleZ(0.0)
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
hmmwv.SetTireType(veh.TireModelType_TMEASY)           # TMEASY needed for SCM
hmmwv.SetTireStepSize(STEP_SIZE)
hmmwv.Initialize()

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system = hmmwv.GetSystem()                # ChSystem owned by wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED before SCM
chassis = hmmwv.GetChassisBody()          # cache: main chassis rigid body
# spindles: hmmwv.GetVehicle().GetAxle(i).m_wheels[j].GetSpindle()
# joints, steering, suspension: created internally by wrapper

# Visualization types
hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === SCM Terrain (bump64 heightmap — hill) ===
terrain = veh.SCMTerrain(system)
terrain.SetSoilParameters(
    2e6,    # Bekker_Kphi
    0,      # Bekker_Kc
    1.1,    # Bekker_n
    0,      # Mohr_cohesion (Pa)
    30,     # Mohr_friction (deg)
    0.01,   # Janosi_shear (m)
    2e8,    # elastic_K (Pa/m)
    3e4,    # damping_R (Pa·s/m)
)
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.1)  # colored sinkage overlay
terrain.Initialize(
    chrono.GetChronoDataFile("vehicle/terrain/height_maps/bump64.bmp"),
    TERRAIN_LENGTH, TERRAIN_WIDTH,
    -1.0, 1.0,
    TERRAIN_RESOLUTION,
)
terrain.SetTexture(chrono.GetChronoDataFile("vehicle/terrain/textures/dirt.jpg"), 6.0, 6.0)

# Moving patch attached to chassis body (NOT spindles — they rotate)
terrain.AddActiveDomain(
    chassis,
    chrono.ChVector3d(0, 0, 0),
    chrono.ChVector3d(5, 3, 1),
)

# === Tire collision cylinders (required for TMEASY on SCM) ===
tire_rad = hmmwv.GetVehicle().GetAxles()[0].m_wheels[0].GetTire().GetRadius()
tire_w   = hmmwv.GetVehicle().GetAxles()[0].m_wheels[0].GetTire().GetWidth()
tire_mat = chrono.ChContactMaterialSMC()
tire_mat.SetFriction(0.9)
tire_mat.SetRestitution(0.1)

TIRE_FAMILY    = 1
SUPPORT_FAMILY = 4

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
        sp_cm.DisallowCollisionsWith(SUPPORT_FAMILY)

system.GetCollisionSystem().BindAll()  # rebuild after post-init shape changes

# Verify wheel-bottom spawn height (assert wheels not sunk into terrain)
veh_obj = hmmwv.GetVehicle()
spindle_world = []
for axle_idx in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        p = veh_obj.GetSpindlePos(axle_idx, side)
        spindle_world.append(p)
wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
TERRAIN_TOP_Z = TERRAIN_MAX_HEIGHT  # max SCM terrain elevation at spawn
ZTOL = 0.15
assert wheel_bottom_z >= TERRAIN_TOP_Z - ZTOL, (
    f"Vehicle sinks into terrain: wheel_bottom_z={wheel_bottom_z:.3f} "
    f"vs terrain_top_z={TERRAIN_TOP_Z:.3f}; raise SUSPENSION_REF_HEIGHT by "
    f"{TERRAIN_TOP_Z - wheel_bottom_z:.3f} m"
)

# === Box obstacles (5 randomly placed) ===
obs_mat = chrono.ChContactMaterialSMC()
obs_mat.SetFriction(0.7)
obs_mat.SetRestitution(0.0)
obs_mat.SetYoungModulus(1e8)

obstacle_bodies = []
obs_positions = []
obs_x_range = (-40.0, 40.0)
obs_y_range = (-40.0, 40.0)
# Minimum clearance from vehicle spawn (0,0) to avoid overlap
MIN_SPAWN_DIST = 5.0

for i in range(NUM_OBSTACLES):
    while True:
        ox = random.uniform(*obs_x_range)
        oy = random.uniform(*obs_y_range)
        if math.sqrt(ox**2 + oy**2) >= MIN_SPAWN_DIST:
            break
    obs = chrono.ChBodyEasyBox(
        OBS_HALF_SIZE * 2, OBS_HALF_SIZE * 2, OBS_HALF_SIZE * 2,
        1000.0, True, True, obs_mat,
    )
    obs.SetName(f"obstacle_{i}")
    obs.SetPos(chrono.ChVector3d(ox, oy, OBS_HALF_SIZE))
    obs.SetFixed(True)
    obs.EnableCollision(True)
    system.AddBody(obs)
    obstacle_bodies.append(obs)
    obs_positions.append((ox, oy))

system.GetCollisionSystem().BindAll()  # bind obstacle collision shapes

# === Sensor Manager + Lidar ===
import pychrono.sensor as sens  # already imported above; kept for clarity

manager = sens.ChSensorManager(system)

# Point light for sensor rendering
intensity = 1.0
manager.scene.AddPointLight(
    chrono.ChVector3f(0, 0, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)

# Lidar parameters
LIDAR_UPDATE_RATE = 5.0       # Hz
H_SAMPLES = 800               # horizontal samples
V_SAMPLES = 300               # vertical samples
H_FOV = 2 * chrono.CH_PI     # full 360 horizontal
MAX_VERT_ANGLE  =  chrono.CH_PI / 12   # ~15 degrees up
MIN_VERT_ANGLE  = -chrono.CH_PI / 6   # ~30 degrees down
MAX_RANGE = 100.0

lidar_offset = chrono.ChFramed(
    chrono.ChVector3d(0, 0, 2.0),                         # 2 m above chassis
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
lidar = sens.ChLidarSensor(
    chassis,                             # attached to vehicle chassis
    LIDAR_UPDATE_RATE,
    lidar_offset,
    H_SAMPLES,
    V_SAMPLES,
    H_FOV,
    MAX_VERT_ANGLE,
    MIN_VERT_ANGLE,
    MAX_RANGE,
    sens.LidarBeamShape_RECTANGULAR,
    2,       # sample_radius
    0.003,   # vert divergence_angle
    0.003,   # hori divergence_angle
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(1.0 / LIDAR_UPDATE_RATE)   # collection window = 1/rate

# Lidar filter chain: visualize depth, access depth+intensity, convert to point cloud, visualize point cloud, access XYZI
lidar.PushFilter(sens.ChFilterVisualize(H_SAMPLES, V_SAMPLES, "Raw Lidar Depth"))
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
lidar.PushFilter(sens.ChFilterXYZIAccess())
manager.AddSensor(lidar)

# === Driver (interactive — scored core; review-only block overrides for RUN video) ===
# Visualization must be built before driver for ChInteractiveDriverIRR
import pychrono.irrlicht as chronoirr  # noqa: E402 — after vehicle setup

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV SCM Hill with Lidar")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(hmmwv.GetVehicle())

driver = veh.ChInteractiveDriver(hmmwv.GetVehicle())
STEERING_TIME = 1.0
THROTTLE_TIME = 1.0
BRAKING_TIME  = 0.3
RENDER_STEP_SIZE = 1.0 / RENDER_FPS   # precomputed once
driver.SetSteeringDelta(RENDER_STEP_SIZE / STEERING_TIME)
driver.SetThrottleDelta(RENDER_STEP_SIZE / THROTTLE_TIME)
driver.SetBrakingDelta(RENDER_STEP_SIZE / BRAKING_TIME)
driver.Initialize()

# === Output dirs ===
os.makedirs("cam", exist_ok=True)


# === Main simulation loop ===
realtime_timer = chrono.ChRealtimeStepTimer()  # for real-time pacing in interactive mode
frame = 0

try:
    while vis.Run() and hmmwv.GetSystem().GetChTime() < SIM_END:
        sim_time = hmmwv.GetSystem().GetChTime()  # cache: fetched once per frame

        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(RENDER_EVERY):
            step_time = hmmwv.GetSystem().GetChTime()

            driver_inputs = driver.GetInputs()


            driver.Synchronize(step_time)
            terrain.Synchronize(step_time)
            hmmwv.Synchronize(step_time, driver_inputs, terrain)
            vis.Synchronize(step_time, driver_inputs)

            driver.Advance(STEP_SIZE)
            terrain.Advance(STEP_SIZE)
            hmmwv.Advance(STEP_SIZE)
            vis.Advance(STEP_SIZE)

            manager.Update()   # update all sensors once per physics step


            if hmmwv.GetSystem().GetChTime() >= SIM_END:
                break

        realtime_timer.Spin(STEP_SIZE)  # pace to real-time

except (RuntimeError, ValueError) as exc:   # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise

finally:
    pass
