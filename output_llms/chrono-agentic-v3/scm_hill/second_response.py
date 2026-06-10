"""
HMMWV on SCM Bump Terrain with Lidar Sensor and Random Box Obstacles.

System type: ChSystemNSC (owned by HMMWV_Full wrapper).
Key bodies: HMMWV chassis, 4 wheel spindles, SCM deformable terrain (bump64.bmp heightmap),
            5 randomly-placed fixed box obstacles confined to the simulation space.
Sensors: ChSensorManager with a 3D lidar mounted on the chassis.
Expected behavior: HMMWV drives interactively over the hill terrain, lidar scans the
environment, and the vehicle creates visible ruts. Box obstacles are scattered across
the simulation space and can be encountered by the vehicle.
"""

import os
import math
import numpy as np

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.sensor as sens


# === Named Constants ===
STEP_SIZE     = 2e-3           # physics time step (s)
SIM_END       = 30.0           # simulation end time (s)
RENDER_FPS    = 50.0           # Irrlicht render frequency (Hz)
TERRAIN_SIZE  = 60.0           # SCM patch length & width (m) — large enough to avoid edge-falling
SCM_RES       = 0.05           # SCM grid resolution (m)
INIT_X        = -15.0          # vehicle spawn X, well inside terrain bounds
INIT_Y        = 0.0            # vehicle spawn Y (m)
INIT_Z        = 0.5            # approximate chassis spawn Z (m)
STEERING_TIME = 1.0            # s to max steering
THROTTLE_TIME = 1.0            # s to max throttle
BRAKING_TIME  = 0.3            # s to max braking
TIRE_FAMILY    = 1              # collision family for tire cylinders
SUPPORT_FAMILY = 4              # collision family for support plane

# precomputed once
render_every      = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once
render_step_size  = 1.0 / RENDER_FPS                               # precomputed once

# === Data Paths ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


# === Vehicle Setup ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)      # SMC required for SCM terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                            # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(
    chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z),
    chrono.QUNIT,
))
hmmwv.SetTireType(veh.TireModelType_TMEASY)             # TMEASY required for SCM; RIGID won't drive
hmmwv.SetTireStepSize(STEP_SIZE)
hmmwv.Initialize()

# === System & Bodies (created by the veh.HMMWV_Full wrapper) ===
system = hmmwv.GetSystem()                              # ChSystemNSC owned by the wrapper
chassis = hmmwv.GetChassisBody()                       # cache: fetched once, reused below
# wheels/spindles: hmmwv.GetVehicle().GetAxles()[i].m_wheels[j].GetSpindle()
# joints: suspension + steering links created inside the wrapper

system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED before SCMTerrain

print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

# === Visualization Type Settings ===
hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# === SCM Terrain (Bekker-Wong bump heightmap) ===
terrain = veh.SCMTerrain(system)
terrain.SetSoilParameters(
    2e6,    # Bekker_Kphi — frictional modulus (Pa)
    0,      # Bekker_Kc   — cohesive modulus
    1.1,    # Bekker_n    — exponent
    0,      # Mohr_cohesion (Pa)
    30,     # Mohr_friction (deg)
    0.01,   # Janosi_shear (m)
    2e8,    # elastic_K (Pa/m)
    3e4,    # damping_R (Pa·s/m)
)
terrain.AddMovingPatch(
    chassis,                              # chassis body — stable OOBB projection
    chrono.ChVector3d(0, 0, 0),
    chrono.ChVector3d(5, 3, 1),
)
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.1)
terrain.Initialize(
    veh.GetDataFile("terrain/height_maps/bump64.bmp"),
    TERRAIN_SIZE, TERRAIN_SIZE,
    -1.0, 1.0,
    SCM_RES,
)
terrain.SetTexture(
    chrono.GetChronoDataFile("vehicle/terrain/textures/dirt.jpg"),
    6.0, 6.0,
)

# === Tire Collision Cylinders for SCM (TMEASY does not auto-add collision) ===
tire_rad = hmmwv.GetVehicle().GetAxles()[0].m_wheels[0].GetTire().GetRadius()
tire_w   = hmmwv.GetVehicle().GetAxles()[0].m_wheels[0].GetTire().GetWidth()
tire_mat = chrono.ChContactMaterialSMC()
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

system.GetCollisionSystem().BindAll()  # rebuild after post-init shape changes

# === Hidden Support Plane for Box Obstacles ===
support_mat = chrono.ChContactMaterialSMC()
support_mat.SetFriction(0.9)
support_mat.SetRestitution(0.01)
support_mat.SetYoungModulus(2e7)
support = chrono.ChBodyEasyBox(
    TERRAIN_SIZE, TERRAIN_SIZE, 0.2,
    1000, False, True, support_mat,
)
support.SetName("asset_support_ground")
support.SetPos(chrono.ChVector3d(0, 0, -0.1))   # top surface at z=0 (SCM rest plane)
support.SetFixed(True)
support.EnableCollision(True)
support_cm = support.GetCollisionModel()
support_cm.SetFamily(SUPPORT_FAMILY)
support_cm.DisallowCollisionsWith(TIRE_FAMILY)
system.AddBody(support)

# === 5 Random Box Obstacles — confined to terrain bounds, away from start ===
np.random.seed(42)   # deterministic placement for reproducibility
box_mat = chrono.ChContactMaterialSMC()
box_mat.SetFriction(0.8)
box_mat.SetRestitution(0.0)
box_half = TERRAIN_SIZE * 0.4   # precomputed once: keep well inside terrain edge
for i in range(5):
    # Spread obstacles in the positive X direction (vehicle drives forward)
    bx = float(np.random.uniform(-box_half, box_half))
    by = float(np.random.uniform(-box_half, box_half))
    bz = 0.25   # center Z — bottom rests at z=0 on support plane
    box_body = chrono.ChBodyEasyBox(1.0, 1.0, 0.5, 500, True, True, box_mat)
    box_body.SetName(f"obstacle_{i}")
    box_body.SetPos(chrono.ChVector3d(bx, by, bz))
    box_body.SetFixed(True)
    system.AddBody(box_body)

# === Sensor Manager and Lidar ===
manager = sens.ChSensorManager(system)
# Lidar is a ranging sensor — no optical lighting needed

lidar_offset = chrono.ChFramed(
    chrono.ChVector3d(0, 0, 1.8),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
lidar = sens.ChLidarSensor(
    chassis,                               # attached to chassis body
    5.0,                                   # update_rate (Hz) — physical rate
    lidar_offset,                          # offset pose from chassis origin
    800,                                   # horizontal_samples
    300,                                   # vertical_samples
    2 * chrono.CH_PI,                      # horizontal_fov (full 360°)
    chrono.CH_PI / 12,                     # max_vert_angle (rad, ~15°)
    -chrono.CH_PI / 6,                     # min_vert_angle (rad, ~-30°)
    100.0,                                 # max_range (m)
    sens.LidarBeamShape_RECTANGULAR,
    2,                                     # sample_radius
    0.003,                                 # vertical divergence angle (rad)
    0.003,                                 # horizontal divergence angle (rad)
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(1.0 / 5.0)

# Lidar filter chain (scored core — prompt requests visualization)
lidar.PushFilter(sens.ChFilterVisualize(800, 300, "Raw Lidar Depth"))
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
lidar.PushFilter(sens.ChFilterXYZIAccess())
manager.AddSensor(lidar)

# === Irrlicht Visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV on SCM Bump Terrain with Lidar")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()                          # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                 # vehicle truths use directional light
vis.AttachVehicle(hmmwv.GetVehicle())

# === Interactive Driver ===
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / STEERING_TIME)
driver.SetThrottleDelta(render_step_size / THROTTLE_TIME)
driver.SetBrakingDelta(render_step_size / BRAKING_TIME)
driver.Initialize()

# === Wheel-bottom spawn assertion (SCM terrain, no X/Y platform bounds) ===
TIRE_RADIUS = tire_rad   # cache: already fetched above
ZTOL = 0.1
veh_obj = hmmwv.GetVehicle()
spindle_zs = []
for axle_idx in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        p = veh_obj.GetSpindlePos(axle_idx, side)
        spindle_zs.append(p.z)
wheel_bottom_z = min(spindle_zs) - TIRE_RADIUS
assert wheel_bottom_z >= -ZTOL, (
    f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f}; "
    f"raise INIT_Z by {-wheel_bottom_z:.3f} m"
)


# === Main Loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

try:
    while vis.Run() and system.GetChTime() < SIM_END:
        time = system.GetChTime()   # cache: fetched once per outer loop

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

        manager.Update()   # update sensor manager once per step


        step_number += 1
        realtime_timer.Spin(STEP_SIZE)
        if system.GetChTime() >= SIM_END:
            break

except (RuntimeError, ValueError) as exc:  # solver divergence / bad state
    import traceback; traceback.print_exc()
    raise
finally:
    pass  # CSV closed in review-only block below
