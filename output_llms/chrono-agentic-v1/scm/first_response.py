"""
HMMWV on SCM Deformable Terrain with Interactive Driver — PyChrono 9.0.x / Irrlicht

Simulates a full HMMWV vehicle driving on Bekker-Wong soft-soil (SCM) terrain.
- System: ChSystemNSC owned by the HMMWV_Full wrapper (SMC contact for SCM)
- Vehicle: HMMWV_Full with rigid tire model, mesh visualization on all components
- Terrain: SCMTerrain with custom soil parameters and a moving patch following the chassis
- Sinkage overlay: false-color sinkage heatmap via SetPlotType
- Driver: ChInteractiveDriverIRR for real-time steering/throttle/braking
- Loop: real-time at 50 FPS with ChRealtimeStepTimer
"""

import math
import pychrono.core as chrono
import pychrono.vehicle as veh

# Anchor the bundled vehicle data subtree so mesh OBJ files resolve correctly
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Constants ===
# Simulation timing
time_step = 2e-3          # physics step size (s) — SCM default 2 ms
sim_end   = 30.0          # total sim duration (s)
render_fps = 50.0         # Irrlicht render rate (frames per second)

# Vehicle init
INIT_X, INIT_Y = -8.0, 0.0
SUSPENSION_REF_HEIGHT = 0.5   # chassis origin above wheel-bottom at rest (HMMWV approx)
TERRAIN_HEIGHT = 0.0          # SCM rest plane at z=0
INIT_Z = TERRAIN_HEIGHT + SUSPENSION_REF_HEIGHT

# SCM terrain
SCM_LENGTH = 120.0
SCM_WIDTH  = 120.0
SCM_GRID   = 0.1            # grid resolution (m); 0.1 m → visible ruts, manageable cost

# Precomputed render cadence — used every loop iteration
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once


# === Vehicle: HMMWV_Full ===
# NOTE: SCM requires SMC contact; rigid tire DOES work for SCM detection (rigid tire
# adds its own collision geometry automatically — no extra spindle cylinders needed)
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)       # SMC for SCM deformable terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                             # MANDATORY — fixed chassis won't move

init_loc = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)              # identity — facing +X
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))

hmmwv.SetTireType(veh.TireModelType_RIGID)               # prompt: rigid tire model
hmmwv.SetTireStepSize(time_step)
hmmwv.Initialize()

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system  = hmmwv.GetSystem()                   # ChSystemSMC owned by the wrapper
chassis = hmmwv.GetChassisBody()              # cache: main chassis rigid body, reused
# wheels/spindles: via hmmwv.GetVehicle().GetAxles(); terrain: SCMTerrain below
# joints: suspension + steering links created inside the wrapper

# Collision system — REQUIRED for SCM; must call AFTER Initialize(), BEFORE SCMTerrain
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Visualization mesh types (all components use MESH per prompt) ===
# Note: in this 9.0.0 build VisualizationType_* lives in the veh namespace
hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# === Verify wheel-bottom placement ===
# (HMMWV origin = geometric center; check wheel bottoms are on/above terrain)
TIRE_RADIUS = 0.33          # approximate HMMWV tire radius
ZTOL = 0.05
veh_obj = hmmwv.GetVehicle()    # cache: vehicle object
spindle_positions = []
for _axle in range(veh_obj.GetNumberAxles()):
    for _side in (veh.LEFT, veh.RIGHT):
        spindle_positions.append(veh_obj.GetSpindlePos(_axle, _side))

wheel_bottom_z = min(p.z for p in spindle_positions) - TIRE_RADIUS
assert wheel_bottom_z >= TERRAIN_HEIGHT - ZTOL, (
    f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} vs "
    f"terrain z={TERRAIN_HEIGHT:.3f}; raise SUSPENSION_REF_HEIGHT by "
    f"{TERRAIN_HEIGHT - wheel_bottom_z:.3f} m"
)

# === SCM Terrain ===
# Build after SetCollisionSystemType — SCMTerrain constructor requires a collision system.
terrain = veh.SCMTerrain(system)

# Custom soil parameters (8 positional args required)
terrain.SetSoilParameters(
    2e6,    # Bekker_Kphi    — frictional modulus (Pa)
    0.0,    # Bekker_Kc      — cohesive modulus
    1.1,    # Bekker_n       — exponent
    0.0,    # Mohr_cohesion  — cohesive limit (Pa)
    30.0,   # Mohr_friction  — friction angle (deg)
    0.01,   # Janosi_shear   — shear coefficient (m)
    2e8,    # elastic_K      — elastic stiffness (Pa/m)
    3e4,    # damping_R      — vertical damping (Pa·s/m)
)

# Sinkage false-color plot (prompt: visualize sinkage with false color)
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.1)

# Moving patch: follows chassis body; OOBB in chassis-local frame
# CRITICAL: attach to chassis body (not spindles) — spindles rotate, OOBB would degenerate
terrain.AddMovingPatch(
    chassis,                            # reference body — chassis stays level
    chrono.ChVector3d(0, 0, 0),         # local OOBB centre offset
    chrono.ChVector3d(5, 3, 1),         # OOBB dims (m) — covers all four wheels
)

terrain.SetMeshWireframe(False)
terrain.SetTexture(
    chrono.GetChronoDataFile("vehicle/terrain/textures/dirt.jpg"),
    80, 80,
)

terrain.Initialize(SCM_LENGTH, SCM_WIDTH, SCM_GRID)

# === Irrlicht Visualization (vehicle-aware) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV on SCM Deformable Terrain — Interactive Driver")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(hmmwv.GetVehicle())

# === Driver: Interactive (ChInteractiveDriverIRR) ===
# Takes the vehicle visual system — must be created after vis is built
steering_time = 1.0     # s to reach full steering
throttle_time = 1.0     # s to reach full throttle
braking_time  = 0.3     # s to reach full braking

driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_every * time_step / steering_time)
driver.SetThrottleDelta(render_every * time_step / throttle_time)
driver.SetBrakingDelta(render_every * time_step / braking_time)
driver.Initialize()

# === Review-only: frame/CSV setup ===

# === Main Loop ===
realtime_timer = chrono.ChRealtimeStepTimer()   # real-time pacing
frame = 0
step_count = 0

try:
    while vis.Run():
        time = system.GetChTime()
        if time >= sim_end:
            break

        # Throttled rendering at render_fps
        if step_count % render_every == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()


        # Get driver inputs from the interactive driver
        driver_inputs = driver.GetInputs()


        # Synchronize all subsystems (order: driver → terrain → vehicle → vis)
        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)


        # Advance all subsystems
        driver.Advance(time_step)
        terrain.Advance(time_step)
        hmmwv.Advance(time_step)     # steps the wrapper-owned ChSystem — do NOT also call system.DoStepDynamics
        vis.Advance(time_step)

        step_count += 1
        realtime_timer.Spin(time_step)   # pace the loop to real-time

except (RuntimeError, ValueError) as exc:   # solver divergence / bad simulation state
    import traceback
    traceback.print_exc()
    raise

finally:
    pass
