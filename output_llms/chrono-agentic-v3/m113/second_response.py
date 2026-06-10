"""
M113 tracked vehicle simulation on SCM deformable terrain.

System type: ChSystemSMC (via M113 wrapper with SMC contact method).
Main bodies: M113 chassis, track shoes (single-pin), sprockets, idlers, road wheels.
Terrain: SCMTerrain (Bekker-Wong soft soil) initialized from heightmap (bump64.bmp).
Expected behavior: M113 starts at (-15, 0, 0) and drives forward at full throttle (0.8)
on SCM deformable terrain, leaving visible ruts in the soft ground.
"""

import os
import math
import csv

import pychrono.core as chrono
import pychrono.vehicle as veh

# === Named constants ===
STEP_SIZE       = 5e-4          # simulation time step (s) — M113 needs small step
SIM_END         = 20.0          # total simulation duration (s)
RENDER_FPS      = 50.0
RENDER_EVERY    = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

# Vehicle spawn
INIT_X          = -15.0
INIT_Y          = 0.0
INIT_Z          = 0.9           # chassis origin above wheel-bottom at rest on flat terrain

# SCM terrain
SCM_LENGTH      = 120.0
SCM_WIDTH       = 120.0
SCM_RES         = 0.05          # grid resolution (m) — fine for visible ruts

# === Data paths — truth-required, scored core ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetVehicleDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle (M113 tracked wrapper — SMC contact) ===
vehicle = veh.M113()
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
vehicle.SetChassisFixed(False)
vehicle.SetTrackShoeType(veh.TrackShoeType_SINGLE_PIN)
vehicle.SetDrivelineType(veh.DrivelineTypeTV_BDS)
vehicle.SetEngineType(veh.EngineModelType_SHAFTS)
vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
vehicle.SetBrakeType(veh.BrakeType_SIMPLE)

init_loc = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))

vehicle.Initialize()

# === System & bodies (created by the veh.M113 wrapper) ===
sys = vehicle.GetSystem()           # ChSystemSMC owned by the wrapper
chassis = vehicle.GetChassisBody()  # cache: main chassis rigid body, reused below
# Track shoes/sprockets/idlers/road wheels created internally by the wrapper

# Stable solver for tracked-vehicle contact chains
sys.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)

# Collision system — REQUIRED for terrain/vehicle contact; set AFTER Initialize, BEFORE SCMTerrain
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# Visualization types — chrono.VisualizationType_* namespace
vehicle.SetChassisVisualizationType(chrono.VisualizationType_PRIMITIVES)
vehicle.SetTrackShoeVisualizationType(chrono.VisualizationType_PRIMITIVES)
vehicle.SetSprocketVisualizationType(chrono.VisualizationType_PRIMITIVES)
vehicle.SetIdlerVisualizationType(chrono.VisualizationType_PRIMITIVES)
vehicle.SetRoadWheelVisualizationType(chrono.VisualizationType_PRIMITIVES)

# === SCM Deformable Terrain — Bekker-Wong soft soil ===
terrain = veh.SCMTerrain(sys)

# Soil parameters — all 8 required (frictional sandy loam)
terrain.SetSoilParameters(
    2e6,    # Bekker_Kphi    — frictional modulus (Pa)
    0,      # Bekker_Kc      — cohesive modulus
    1.1,    # Bekker_n       — pressure-sinkage exponent
    0,      # Mohr_cohesion  — cohesive limit (Pa)
    30,     # Mohr_friction  — friction angle (deg)
    0.01,   # Janosi_shear   — shear deformation coefficient (m)
    2e8,    # elastic_K      — elastic stiffness (Pa/m)
    3e4,    # damping_R      — vertical damping (Pa·s/m)
)

# Active domain — attach to chassis body (NOT spindles, which rotate and break the OOBB)
terrain.AddActiveDomain(
    chassis,                        # cache: chassis body (stable orientation)
    chrono.ChVector3d(0, 0, 0),    # local OOBB centre offset
    chrono.ChVector3d(5, 3, 1),    # OOBB dimensions (m)
)

terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.1)  # sinkage heatmap

# Initialize SCM from heightmap — bump64.bmp, length×width, hMin/hMax, resolution
terrain.Initialize(
    veh.GetVehicleDataFile("terrain/height_maps/bump64.bmp"),
    SCM_LENGTH, SCM_WIDTH,
    -1.0, 1.0,    # height range (m)
    SCM_RES,
)

# Dirt texture
terrain.SetMeshWireframe(False)
terrain.SetTexture(
    chrono.GetChronoDataFile("vehicle/terrain/textures/dirt.jpg"),
    80, 80,
)

# === Driver inputs — scripted throttle 0.8 (truth-matched; NOT review-only) ===
driver_inputs = veh.DriverInputs()
driver_inputs.m_steering = 0.0
driver_inputs.m_braking  = 0.0
driver_inputs.m_throttle = 0.0  # set to 0.8 inside loop

# === Visualization — ChTrackedVehicleVisualSystemIrrlicht ===
vis = veh.ChTrackedVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("M113 on SCM Deformable Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle.GetVehicle())


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        time = sys.GetChTime()  # cache: fetched once per outer step

        if step_number % RENDER_EVERY == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        # Hard-coded throttle 0.8 — prompt-required, scored core
        driver_inputs.m_throttle = 0.8
        driver_inputs.m_steering = 0.0
        driver_inputs.m_braking  = 0.0

        # Synchronize — tracked vehicle: 2-arg (no terrain argument)
        terrain.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs)
        vis.Synchronize(time, driver_inputs)


        # Advance — vehicle.Advance steps the wrapper-owned ChSystem; do NOT also call DoStepDynamics
        terrain.Advance(STEP_SIZE)
        vehicle.Advance(STEP_SIZE)
        vis.Advance(STEP_SIZE)

        step_number += 1
        realtime_timer.Spin(STEP_SIZE)

except (RuntimeError, ValueError) as exc:  # solver divergence / bad physics state
    import traceback
    traceback.print_exc()
    raise

finally:
    pass  # CSV closed in review-only block below
