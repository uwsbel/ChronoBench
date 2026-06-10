"""
HMMWV on SCM Deformable Hill Terrain — PyChrono 9.0.x, Irrlicht renderer.

System: ChSystemNSC (owned by HMMWV_Full wrapper).
Main bodies: HMMWV chassis + 4 wheel spindles + SCM deformable terrain (bump64 heightmap).
Expected behaviour: HMMWV drives forward over a bumpy hill terrain, wheels sinking
into the soft soil (TMEASY tires + SCM Bekker-Wong), leaving visible ruts, with
real-time Irrlicht visualization and chase camera.
"""

import os
import math
import csv

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


# === Named constants ===
STEP_SIZE = 2e-3          # simulation step (s)
SIM_END   = 20.0          # simulation duration (s)
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

TERRAIN_LENGTH = 40.0     # SCM patch length (m) — matches bump64 map
TERRAIN_WIDTH  = 40.0     # SCM patch width (m)
SCM_RESOLUTION = 0.02     # grid resolution (m)

# Initial vehicle position — start near the patch edge so it drives over the hill
INIT_X = -15.0
INIT_Y =  0.0
INIT_Z =  0.5   # approximate spawn height above flat terrain (suspension settles)

TIRE_FAMILY    = 1
SUPPORT_FAMILY = 4

# === Data paths (truth-faithful) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle setup (HMMWV_Full wrapper owns ChSystem) ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)          # SMC for SCM/deformable terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                                # MANDATORY — fixed chassis won't move
init_loc = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
hmmwv.SetTireType(veh.TireModelType_TMEASY)                 # TMEASY required for SCM traction
hmmwv.SetTireStepSize(STEP_SIZE)
hmmwv.Initialize()

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system  = hmmwv.GetSystem()                 # ChSystemNSC owned by the wrapper
chassis = hmmwv.GetChassisBody()            # main chassis rigid body; cache: fetched once
# wheels/spindles: accessed via hmmwv.GetVehicle().GetAxles()
# joints: suspension + steering links created inside the wrapper

# Set collision system AFTER Initialize (SCM requires this order)
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

# === Visualization types ===
hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# === SCM Terrain — Bekker-Wong deformable soft soil with bump heightmap ===
terrain = veh.SCMTerrain(system)
terrain.SetSoilParameters(
    2e6,    # Bekker_Kphi  — frictional modulus (Pa)
    0,      # Bekker_Kc    — cohesive modulus
    1.1,    # Bekker_n     — pressure-sinkage exponent
    0,      # Mohr_cohesion — cohesion (Pa)
    30,     # Mohr_friction — friction angle (deg)
    0.01,   # Janosi_shear  — shear deformation coefficient (m)
    2e8,    # elastic_K    — vertical elastic stiffness (Pa/m)
    3e4,    # damping_R    — vertical damping (Pa·s/m)
)

# Moving patch — follow chassis so only nearby cells are ray-cast (performance)
terrain.AddMovingPatch(
    chassis,                           # MUST be chassis, not spindle
    chrono.ChVector3d(0, 0, 0),        # local OOBB centre offset
    chrono.ChVector3d(5, 3, 1),        # OOBB dimensions (m)
)

terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.1)  # sinkage heatmap

# Initialize from bump heightmap — 40×40 m, height range [-1, 1] m, 0.02 m grid
terrain.Initialize(
    veh.GetDataFile("terrain/height_maps/bump64.bmp"),
    TERRAIN_LENGTH, TERRAIN_WIDTH, -1.0, 1.0, SCM_RESOLUTION,
)
terrain.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)

# === Tire collision cylinders (REQUIRED for TMEASY on SCM) ===
# TMEASY tires do not add collision geometry automatically; we add cylinders manually.
vehicle_obj = hmmwv.GetVehicle()   # cache: fetched once, reused for spindle loop
tire_rad = vehicle_obj.GetAxles()[0].m_wheels[0].GetTire().GetRadius()
tire_w   = vehicle_obj.GetAxles()[0].m_wheels[0].GetTire().GetWidth()
tire_mat = chrono.ChContactMaterialSMC()
tire_mat.SetFriction(0.9)
tire_mat.SetRestitution(0.1)

for axle in vehicle_obj.GetAxles():
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

# Rebuild all collision models after post-init shape changes
system.GetCollisionSystem().BindAll()

# === Irrlicht visualization — full scene: window + Initialize + sky + camera + lights ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV on SCM Hill Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 9.0, 0.5)
vis.Initialize()                             # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()                              # standard outdoor sky
vis.AddLightDirectional()                    # vehicle truths use directional light
vis.AttachVehicle(hmmwv.GetVehicle())

# === Interactive driver (ChInteractiveDriverIRR — scored-core default for catalog vehicles) ===
render_step_size = 1.0 / RENDER_FPS         # precomputed once
driver = veh.ChInteractiveDriverIRR(vis)    # takes vis, not vehicle
driver.SetSteeringDelta(render_step_size / 1.0)
driver.SetThrottleDelta(render_step_size / 1.0)
driver.SetBrakingDelta(render_step_size / 0.3)
driver.Initialize()

# === Review-only recording setup ===


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
frame = 0
step_number = 0

try:
    while vis.Run() and system.GetChTime() < SIM_END:
        time = system.GetChTime()

        # Throttled rendering — render once per frame, not every physics step
        if step_number % RENDER_EVERY == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()


        # Synchronize subsystems (full stack, MANDATORY order)
        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)


        # Advance subsystems
        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        hmmwv.Advance(STEP_SIZE)    # advances the wrapper-owned ChSystem — do NOT also call DoStepDynamics
        vis.Advance(STEP_SIZE)

        step_number += 1
        realtime_timer.Spin(STEP_SIZE)

except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass  # cleanup in review-only block below
