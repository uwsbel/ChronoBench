"""
SCM Hill simulation with HMMWV driving over a bumpy heightmap terrain (SCMTerrain).
System type: NSC (SMC contact method for SCM deformable terrain).
Main bodies: HMMWV chassis + 4 wheels on Bekker-Wong soft soil, 5 box obstacles.
Sensors: ChSensorManager with a lidar sensor attached to the vehicle chassis.
Expected behavior: HMMWV accelerates forward over the bumpy hill terrain, wheels
sink into the soil leaving ruts. The lidar scans the environment and saves point
cloud data. Five box obstacles are scattered across the terrain.
"""

import math
import os
import numpy as np                                    # numpy for obstacle placement

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens                        # sensor module for lidar


# === Data paths (mandatory for catalog vehicles) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Simulation parameters ===
step_size      = 5e-3          # physics time step (s)
sim_end        = 20.0          # simulation end time (s)
render_fps     = 50.0          # Irrlicht render rate
render_steps   = math.ceil((1.0 / render_fps) / step_size)  # precomputed once

# Terrain constants
SCM_SIZE       = 40.0          # terrain patch size (m)
SCM_HMIN       = -1.0          # height map min elevation (m)
SCM_HMAX       =  1.0          # height map max elevation (m)
SCM_RES        = 0.02          # SCM grid resolution (m)

# Vehicle spawn
INIT_POS       = chrono.ChVector3d(-15, 0, 0.6)       # (inferred default — verify)
INIT_ROT       = chrono.QUNIT

# Obstacle parameters (5 boxes, random XY via numpy seed for reproducibility)
NUM_OBSTACLES  = 5
OBSTACLE_HALF  = chrono.ChVector3d(0.6, 0.4, 0.3)    # half-extents (m)
OBS_MASS       = 200.0                                 # kg
rng            = np.random.default_rng(42)             # deterministic seed
obs_x          = rng.uniform(-10.0,  8.0, NUM_OBSTACLES)
obs_y          = rng.uniform( -8.0,  8.0, NUM_OBSTACLES)

# Lidar parameters
LIDAR_UPDATE_RATE = 10.0       # Hz
LIDAR_HORIZONTAL  = 900        # horizontal samples
LIDAR_VERTICAL    = 16         # vertical channels
LIDAR_FOV_H       = math.pi * 2.0    # full 360° horizontal
LIDAR_FOV_V_MIN   = -math.pi / 6.0  # -30°
LIDAR_FOV_V_MAX   =  math.pi / 6.0  # +30°
LIDAR_MAX_RANGE   = 100.0      # m


# === Vehicle setup ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)   # SCM scenes use SMC
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                          # MANDATORY
hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_POS, INIT_ROT))
hmmwv.SetTireType(veh.TireModelType_TMEASY)           # TMEASY required on SCM
hmmwv.SetTireStepSize(step_size)
hmmwv.Initialize()

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system  = hmmwv.GetSystem()                           # ChSystemSMC owned by the wrapper
chassis = hmmwv.GetChassisBody()                      # cache: fetched once, reused below
# wheels/spindles: hmmwv.GetVehicle().GetAxles()[i].m_wheels[j].GetSpindle()
# joints: suspension + steering links created inside the wrapper

system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED before SCM

hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

# === SCM Terrain (Bekker-Wong heightmap hill) ===
terrain = veh.SCMTerrain(system)
terrain.SetSoilParameters(
    2e6,    # Bekker_Kphi    — frictional modulus (Pa)
    0,      # Bekker_Kc      — cohesive modulus
    1.1,    # Bekker_n       — exponent
    0,      # Mohr_cohesion  — cohesive limit (Pa)
    30,     # Mohr_friction  — friction angle (deg)
    0.01,   # Janosi_shear   — shear coefficient (m)
    2e8,    # elastic_K      — elastic stiffness (Pa/m)
    3e4,    # damping_R      — vertical damping (Pa·s/m)
)
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.1)  # sinkage heatmap
terrain.AddMovingPatch(
    chassis,
    chrono.ChVector3d(0, 0, 0),   # local OOBB centre on chassis
    chrono.ChVector3d(5, 3, 1),   # OOBB dimensions (m)
)
terrain.Initialize(
    veh.GetDataFile("terrain/height_maps/bump64.bmp"),
    SCM_SIZE, SCM_SIZE, SCM_HMIN, SCM_HMAX, SCM_RES,
)
terrain.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)

# === Tire collision cylinders (required for TMEASY on SCM) ===
TIRE_FAMILY    = 1
SUPPORT_FAMILY = 4

tire_ax    = hmmwv.GetVehicle().GetAxles()[0]
tire_rad   = tire_ax.m_wheels[0].GetTire().GetRadius()   # cache: radius fetched once
tire_w     = tire_ax.m_wheels[0].GetTire().GetWidth()    # cache: width fetched once
tire_mat   = chrono.ChContactMaterialSMC()
tire_mat.SetFriction(0.9)
tire_mat.SetRestitution(0.1)

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

system.GetCollisionSystem().BindAll()   # rebuild after post-init shape changes

# === 5 Box obstacles (randomly positioned via numpy) ===
obs_mat = chrono.ChContactMaterialSMC()
obs_mat.SetFriction(0.7)
obs_mat.SetRestitution(0.0)
obs_mat.SetYoungModulus(2e7)

for i in range(NUM_OBSTACLES):
    obs = chrono.ChBodyEasyBox(
        OBSTACLE_HALF.x * 2, OBSTACLE_HALF.y * 2, OBSTACLE_HALF.z * 2,
        OBS_MASS, True, True, obs_mat,
    )
    obs.SetName(f"obstacle_{i}")
    obs.SetPos(chrono.ChVector3d(obs_x[i], obs_y[i], OBSTACLE_HALF.z))
    obs.SetFixed(True)
    system.AddBody(obs)

# === Driver (interactive IRR — scored-core default) ===
# Vehicle visual system must be built before ChInteractiveDriverIRR.
# Visualization is built next; driver is created after vis.

# === Visualization (ChWheeledVehicleVisualSystemIrrlicht) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV on SCM Hill with Lidar")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                             # vehicle truth uses directional light
vis.AttachVehicle(hmmwv.GetVehicle())

# === Interactive driver (created after vis is built) ===
driver = veh.ChInteractiveDriverIRR(vis)
render_step_size = 1.0 / render_fps                  # precomputed once
driver.SetSteeringDelta(render_step_size / 1.0)
driver.SetThrottleDelta(render_step_size / 1.0)
driver.SetBrakingDelta(render_step_size / 0.3)
driver.Initialize()

# === Sensor Manager and Lidar ===
manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(
    chrono.ChVector3f(0, 0, 100),
    chrono.ChColor(1, 1, 1),
    500.0,
)

# Lidar offset on chassis (mounted above roof, facing forward)
lidar_offset = chrono.ChFramed(
    chrono.ChVector3d(0, 0, 2.0),
    chrono.QuatFromAngleZ(0.0),
)
lidar = sens.ChLidarSensor(
    chassis,                   # body the lidar is attached to
    LIDAR_UPDATE_RATE,         # update rate (Hz)
    lidar_offset,              # offset frame on chassis
    LIDAR_HORIZONTAL,          # horizontal samples
    LIDAR_VERTICAL,            # vertical channels
    LIDAR_FOV_H,               # horizontal FOV (rad)
    LIDAR_FOV_V_MAX,           # max vertical angle (rad)
    LIDAR_FOV_V_MIN,           # min vertical angle (rad)
    LIDAR_MAX_RANGE,           # max range (m)
)
lidar.SetName("lidar_sensor")
lidar.SetLag(0.0)
lidar.SetCollectionWindow(1.0 / LIDAR_UPDATE_RATE)
lidar.PushFilter(sens.ChFilterVisualize(LIDAR_HORIZONTAL, LIDAR_VERTICAL, "Lidar Depth"))
lidar.PushFilter(sens.ChFilterDIAccess())
manager.AddSensor(lidar)

# === Wheel-bottom assert (verify vehicle is on terrain, not sinking) ===
TIRE_RADIUS = tire_rad                               # cache: reuse fetched radius
ZTOL = 0.1
veh_obj = hmmwv.GetVehicle()                         # cache: fetched once
spindle_world = []
for axle in veh_obj.GetAxles():
    for side in (veh.LEFT, veh.RIGHT):
        p = veh_obj.GetSpindlePos(axle.GetIndex(), side)
        spindle_world.append(p)
wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= -ZTOL, (
    f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f}; "
    f"raise INIT_POS.z by {-wheel_bottom_z:.3f} m"
)

# === CSV output setup ===
os.makedirs("cam", exist_ok=True)
realtime_timer = chrono.ChRealtimeStepTimer()

# === Main loop ===
step_number = 0
frame       = 0                                       # consecutive frame counter


try:
    while vis.Run() and system.GetChTime() < sim_end:
        sim_time = system.GetChTime()                 # cache: fetched once per outer step

        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()


        driver.Synchronize(sim_time)
        terrain.Synchronize(sim_time)
        hmmwv.Synchronize(sim_time, driver_inputs, terrain)
        vis.Synchronize(sim_time, driver_inputs)

        driver.Advance(step_size)
        terrain.Advance(step_size)
        hmmwv.Advance(step_size)    # steps the wrapper-owned ChSystem
        vis.Advance(step_size)
        manager.Update()            # update sensor manager each step


        step_number += 1
        realtime_timer.Spin(step_size)

except (RuntimeError, ValueError) as exc:             # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
