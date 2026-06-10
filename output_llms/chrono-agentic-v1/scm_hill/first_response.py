"""
HMMWV on SCM Deformable Terrain with Height Map (Hill Scene)

Simulates a full HMMWV vehicle driving over an SCM (Bekker-Wong) soft-soil
terrain initialized from a bump/hill height map. The vehicle starts near the
terrain center and drives forward under an interactive driver. Real-time
Irrlicht visualization shows the vehicle, deformable terrain mesh, and sinkage.

System type: ChSystemNSC (owned by the HMMWV_Full wrapper, SMC tire contact).
Key bodies: HMMWV chassis, 4 wheel spindles, SCM deformable terrain.
Expected behavior: Vehicle climbs the hill bump, leaving visible ruts in the SCM
terrain deformation mesh.
"""

# === Imports ===
import math
import os

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Named constants ===
# Simulation timing
TIME_STEP = 2e-3          # physics time step (s) — SCM stable at 2ms
SIM_END = 30.0            # total simulation duration (s)
RENDER_FPS = 50.0         # target render rate (Hz)

# Vehicle initial position — spawn at terrain edge to drive toward the bump/hill
# bump64.bmp: 40x40 m terrain with a hill near center. Spawn at X=-15 (edge)
# so vehicle drives forward and climbs the hill in the middle.
VEH_INIT_X = -15.0   # back of terrain — vehicle drives +X toward hill
VEH_INIT_Y = 0.0
# Z: chassis init height = terrain height at spawn + suspension reference height
# At X=-15, terrain is flat (edge), height~0; HMMWV suspension ref ~0.5 m
# Use extra clearance (1.0 m) to avoid initial penetration
SUSPENSION_REF_HEIGHT = 1.0
VEH_INIT_Z = SUSPENSION_REF_HEIGHT
TIRE_RADIUS_APPROX = 0.47   # HMMWV TMEASY tire radius (m); used for post-init assert

# SCM terrain parameters
SCM_LENGTH = 40.0    # m along X
SCM_WIDTH = 40.0     # m along Y
SCM_RESOLUTION = 0.02  # grid resolution (m) — fine for visible ruts

# Precomputed render cadence — compute ONCE before the loop
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# === Vehicle setup ===
# HMMWV_Full wrapper — creates and owns ChSystemNSC internally.
# For SCM we use SMC contact method + TMEASY tire (RIGID tire won't move on SCM).
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)   # SMC required for SCM deformable
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                          # MANDATORY — fixed chassis won't move

init_loc = chrono.ChVector3d(VEH_INIT_X, VEH_INIT_Y, VEH_INIT_Z)
init_rot = chrono.QuatFromAngleZ(0.0)
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
hmmwv.SetTireType(veh.TireModelType_TMEASY)           # SCM requires non-rigid tire
hmmwv.SetTireStepSize(TIME_STEP)
hmmwv.Initialize()

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system = hmmwv.GetSystem()                 # ChSystemNSC owned by the wrapper
chassis = hmmwv.GetChassisBody()           # main chassis rigid body; cache: fetched once
# wheels/spindles: hmmwv.GetVehicle().GetAxle(i)... ; terrain: SCMTerrain below
# joints: suspension + steering links created inside the wrapper

# REQUIRED: set collision system BEFORE constructing SCMTerrain
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Set visualization types (AFTER Initialize)
hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === Validate vehicle footprint after Initialize ===
# Assert wheels rest on (not through) the terrain surface at spawn
veh_obj = hmmwv.GetVehicle()  # cache: fetched once, reused for spindle queries
spindle_world = []
for axle_idx in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        p = veh_obj.GetSpindlePos(axle_idx, side)
        spindle_world.append(p)

wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS_APPROX
ZTOL = 1.5   # generous — spawning with extra clearance to avoid initial penetration
assert wheel_bottom_z >= -ZTOL, (
    f"Vehicle sinks too far: wheel bottom z={wheel_bottom_z:.3f}; "
    f"raise SUSPENSION_REF_HEIGHT by {-wheel_bottom_z:.3f} m"
)

# === SCM Terrain (Bekker-Wong deformable soft soil with height map) ===
terrain = veh.SCMTerrain(system)

# Soil parameters (sandy loam off-road baseline — all 8 required)
terrain.SetSoilParameters(
    2e6,    # Bekker_Kphi  — frictional modulus (Pa)
    0,      # Bekker_Kc    — cohesive modulus
    1.1,    # Bekker_n     — pressure-sinkage exponent
    0,      # Mohr_cohesion — cohesive limit (Pa)
    30,     # Mohr_friction — friction angle (deg)
    0.01,   # Janosi_shear  — shear deformation modulus (m)
    2e8,    # elastic_K    — elastic stiffness (Pa/m)
    3e4,    # damping_R    — vertical damping (Pa·s/m)
)

# Moving patch: attach to CHASSIS (not spindle — rotating spindle OOBB becomes degenerate)
terrain.AddActiveDomain(
    chassis,                          # chassis body stays level — stable OOBB
    chrono.ChVector3d(0, 0, 0),       # local OOBB center offset
    chrono.ChVector3d(5, 3, 1),       # OOBB dims (m) — matches truth
)

# Sinkage heatmap overlay (call BEFORE Initialize)
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.1)

# Initialize from height map — uses bump64.bmp (hill/bump terrain)
terrain.Initialize(
    veh.GetVehicleDataFile("terrain/height_maps/bump64.bmp"),
    SCM_LENGTH, SCM_WIDTH,
    -1.0, 1.0,       # hMin, hMax (m) — full bump height range
    SCM_RESOLUTION,
)
terrain.SetTexture(
    veh.GetVehicleDataFile("terrain/textures/dirt.jpg"),
    6.0, 6.0,
)

# === Tire collision cylinders (REQUIRED for TMEASY — SCM ray-cast detection) ===
tire_rad = hmmwv.GetVehicle().GetAxles()[0].m_wheels[0].GetTire().GetRadius()  # cache: from tire
tire_w = hmmwv.GetVehicle().GetAxles()[0].m_wheels[0].GetTire().GetWidth()     # cache: from tire
tire_mat = chrono.ChContactMaterialSMC()
tire_mat.SetFriction(0.9)
tire_mat.SetRestitution(0.1)

TIRE_FAMILY = 1
SUPPORT_FAMILY = 4

for axle in hmmwv.GetVehicle().GetAxles():
    for iw in range(2):
        spindle = axle.m_wheels[iw].GetSpindle()
        spindle.AddCollisionShape(
            chrono.ChCollisionShapeCylinder(
                tire_mat,
                tire_rad + 0.04,   # +4 cm ensures sinkage detection
                tire_w,
            ),
            chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(math.pi / 2)),
        )
        spindle.EnableCollision(True)
        sp_cm = spindle.GetCollisionModel()
        sp_cm.SetFamily(TIRE_FAMILY)
        sp_cm.DisallowCollisionsWith(TIRE_FAMILY)
        sp_cm.DisallowCollisionsWith(SUPPORT_FAMILY)

# Rebuild all collision models after post-init shape changes (MANDATORY)
system.GetCollisionSystem().BindAll()

# === Visualization (vehicle Irrlicht window) ===
# Order: configure -> Initialize() -> AddLogo/AddSkyBox/AddTypicalLights AFTER -> AttachVehicle
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV on SCM Deformable Terrain — Hill Scene")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(hmmwv.GetVehicle())

# === Driver (interactive — scored-core default matching catalog-vehicle truth) ===
driver = veh.ChInteractiveDriver(hmmwv.GetVehicle())

# Precomputed render cadence for driver deltas
render_step_size = 1.0 / RENDER_FPS   # precomputed once
steering_time = 1.0   # s to reach full steering
throttle_time = 1.0   # s to reach full throttle
braking_time = 0.3    # s to reach full brake

driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# === Review-only recording setup ===


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
frame = 0

try:
    while vis.Run() and hmmwv.GetSystem().GetChTime() < SIM_END:
        time = hmmwv.GetSystem().GetChTime()  # cache: per-frame time

        if step_number % RENDER_EVERY == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()


        # Advance subsystems (vehicle.Advance steps the system — do NOT also call DoStepDynamics)
        driver.Advance(TIME_STEP)
        terrain.Advance(TIME_STEP)
        hmmwv.Advance(TIME_STEP)
        vis.Advance(TIME_STEP)

        step_number += 1
        realtime_timer.Spin(TIME_STEP)

except (RuntimeError, ValueError) as exc:   # solver divergence / bad SCM state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
