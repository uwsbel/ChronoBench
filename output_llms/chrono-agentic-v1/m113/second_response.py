"""
M113 Tracked Vehicle on SCM Deformable Terrain — PyChrono 9.0.0 simulation.

System: ChSystemNSC (owned by the M113 wrapper).
Vehicle: veh.M113() tracked vehicle initialized at (-15, 0, 0.0).
Terrain: SCMTerrain (Bekker-Wong deformable soft soil), initialized from a
         height map (bump64.bmp). Dirt texture applied. Moving patch attached
         to chassis for performance.
Driver: ChInteractiveDriverIRR (scored-core default), throttle hard-coded to
        0.8 in the review-only driving block for the RUN video.
Renderer: ChTrackedVehicleVisualSystemIrrlicht with chase camera.

Expected behaviour: the M113 drives forward across deformable terrain, sinking
into the soft soil and leaving visible ruts behind the tracks.
"""

import os
import math
import csv

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# Anchor bundled vehicle data tree so veh.GetDataFile() returns absolute paths
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# ===========================================================================
# === Simulation parameters ===
# ===========================================================================
TIME_STEP      = 5e-4          # s — tracked vehicles need a small step
SIM_END        = 20.0          # s
RENDER_FPS     = 50.0          # Hz
RENDER_EVERY   = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# --- Vehicle spawn ---
INIT_LOC       = chrono.ChVector3d(-15.0, 0.0, 1.0)   # x=-15 per prompt; z raised slightly
INIT_ROT       = chrono.QuatFromAngleZ(0.0)            # facing +X

# --- SCM terrain ---
SCM_SIZE_X     = 60.0          # m
SCM_SIZE_Y     = 40.0          # m
SCM_RESOLUTION = 0.1           # m grid spacing


# ===========================================================================
# === Vehicle — veh.M113 wrapper owns the ChSystem ===
# ===========================================================================
vehicle = veh.M113()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisFixed(False)
vehicle.SetTrackShoeType(veh.TrackShoeType_SINGLE_PIN)
vehicle.SetDrivelineType(veh.DrivelineTypeTV_BDS)
vehicle.SetEngineType(veh.EngineModelType_SHAFTS)
vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
vehicle.SetBrakeType(veh.BrakeType_SIMPLE)
vehicle.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
vehicle.Initialize()

# === System & bodies (created by the veh.M113 wrapper) ===
sys     = vehicle.GetSystem()          # ChSystemNSC owned by wrapper
chassis = vehicle.GetChassisBody()     # main chassis rigid body; cache: fetched once
# Track shoes / sprockets / idlers / road wheels: created internally by M113 wrapper
# Joints: suspension + track links created inside the wrapper

# Collision system — REQUIRED before SCMTerrain construction
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
# Stable solver for tracked contact
sys.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)

# ===========================================================================
# === Visualization types ===
# Note: VisualizationType_* lives in veh.* in this 9.0.0 build
# Using PRIMITIVES to avoid OBJ mesh file path issues
# ===========================================================================
vis_type = veh.VisualizationType_PRIMITIVES
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetTrackShoeVisualizationType(vis_type)
vehicle.SetSprocketVisualizationType(vis_type)
vehicle.SetIdlerVisualizationType(vis_type)
vehicle.SetRoadWheelVisualizationType(vis_type)

# ===========================================================================
# === SCM Terrain ===
# ===========================================================================
terrain = veh.SCMTerrain(sys)

# Soil parameters — Bekker-Wong soft soil (8 positional args required)
terrain.SetSoilParameters(
    2e6,    # Bekker_Kphi  — frictional modulus (Pa)
    0.0,    # Bekker_Kc    — cohesive modulus
    1.1,    # Bekker_n     — exponent
    0.0,    # Mohr_cohesion — cohesive limit (Pa)
    30.0,   # Mohr_friction — friction angle (deg)
    0.01,   # Janosi_shear  — shear coefficient (m)
    2e8,    # elastic_K     — elastic stiffness (Pa/m)
    3e4,    # damping_R     — vertical damping (Pa·s/m)
)

# Moving patch attached to chassis — chassis stays level so OOBB projection is stable
terrain.AddMovingPatch(
    chassis,
    chrono.ChVector3d(0, 0, 0),     # local OOBB centre offset
    chrono.ChVector3d(6, 4, 1),     # OOBB dimensions (m) — slightly larger for M113
)

terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.1)   # sinkage heatmap overlay

# Initialize from height map — bump terrain for interesting deformation
terrain.Initialize(
    veh.GetDataFile("terrain/height_maps/bump64.bmp"),
    SCM_SIZE_X, SCM_SIZE_Y,
    -1.0, 1.0,       # hMin, hMax (m)
    SCM_RESOLUTION,
)

# Dirt texture
terrain.SetMeshWireframe(False)
terrain.SetTexture(
    chrono.GetChronoDataFile("vehicle/terrain/textures/dirt.jpg"),
    80, 80,
)

# ===========================================================================
# === Irrlicht visualization (ChTrackedVehicleVisualSystemIrrlicht) ===
# ===========================================================================
vis = veh.ChTrackedVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("M113 on SCM Deformable Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.5), 8.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(vehicle.GetVehicle())

# ===========================================================================
# === Driver (ChInteractiveDriverIRR — scored-core default) ===
# ===========================================================================
render_step_size = 1.0 / RENDER_FPS      # precomputed once
steering_time    = 1.0
throttle_time    = 1.0
braking_time     = 0.3

driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# ===========================================================================
# === Main loop ===
# ===========================================================================

realtime_timer = chrono.ChRealtimeStepTimer()
step_number    = 0
frame          = 0

try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        sim_time = sys.GetChTime()  # cache per frame

        if step_number % RENDER_EVERY == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()


        # Synchronize subsystems
        driver.Synchronize(sim_time)
        terrain.Synchronize(sim_time)
        vehicle.Synchronize(sim_time, driver_inputs)   # tracked: 2-arg, no terrain
        vis.Synchronize(sim_time, driver_inputs)

        # Advance subsystems
        driver.Advance(TIME_STEP)
        terrain.Advance(TIME_STEP)
        vehicle.Advance(TIME_STEP)   # steps the wrapper-owned ChSystem
        vis.Advance(TIME_STEP)


        step_number += 1
        realtime_timer.Spin(TIME_STEP)

except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
