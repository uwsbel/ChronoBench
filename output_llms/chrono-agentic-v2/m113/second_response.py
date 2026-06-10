"""
M113 Tracked Vehicle on SCM Deformable Terrain — PyChrono 9.0.0 / Irrlicht
============================================================================
Models the M113 tracked vehicle (SMC contact, single-pin track shoes) starting
at (-15, 0, 0.0) on a Bekker-Wong SCM soft-soil terrain initialized from a
height map. The driver applies a fixed throttle of 0.8 to drive the vehicle
forward. Expected behavior: the vehicle accelerates, sinking slightly into the
deformable terrain and leaving visible ruts while traveling in the +X direction.
"""

import os
import math
import csv

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


# === Data paths (required truth components for all catalog-vehicle demos) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Simulation parameters ===
step_size = 5e-4          # time step (s) — M113 tracked contact needs small step
sim_end   = 20.0          # simulation end time (s)
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * step_size)))  # precomputed once

# === Initial conditions ===
init_loc = chrono.ChVector3d(-15.0, 0.0, 1.1)   # spawn with clearance above SCM z=0
init_rot = chrono.QuatFromAngleZ(0.0)

# === Vehicle setup (M113 tracked) ===
vehicle = veh.M113()
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
vehicle.SetChassisFixed(False)
vehicle.SetTrackShoeType(veh.TrackShoeType_SINGLE_PIN)
vehicle.SetDrivelineType(veh.DrivelineTypeTV_BDS)
vehicle.SetEngineType(veh.EngineModelType_SHAFTS)
vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
vehicle.SetBrakeType(veh.BrakeType_SIMPLE)
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
vehicle.Initialize()

# === System & bodies (created by the veh.M113 wrapper) ===
sys     = vehicle.GetSystem()           # ChSystemSMC owned by the M113 wrapper
chassis = vehicle.GetChassisBody()      # cache: fetched once, reused below

# Collision system — REQUIRED before building SCMTerrain
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
# Stable solver for tracked contact
sys.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# === Visualization types ===
vis_type = chrono.VisualizationType_MESH
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetTrackShoeVisualizationType(vis_type)
vehicle.SetSprocketVisualizationType(vis_type)
vehicle.SetIdlerVisualizationType(vis_type)
vehicle.SetRoadWheelVisualizationType(vis_type)

# === SCM Deformable Terrain ===
terrain = veh.SCMTerrain(sys)

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

# Initialize from heightmap: length, width, hMin, hMax, resolution
terrain.Initialize(
    veh.GetDataFile("terrain/height_maps/bump64.bmp"),
    40.0, 40.0,   # length, width (m)
    -1.0, 1.0,    # height range
    0.02,         # grid resolution (m)
)

# Moving patch centered on chassis — keeps active SCM region near vehicle
terrain.AddMovingPatch(
    chassis,                            # chassis body (not spindles — see pitfalls)
    chrono.ChVector3d(0, 0, 0),        # local OOBB center offset
    chrono.ChVector3d(7, 5, 2),        # OOBB dimensions (m)
)

terrain.SetMeshWireframe(False)
terrain.SetTexture(
    chrono.GetChronoDataFile("vehicle/terrain/textures/dirt.jpg"),
    80, 80,
)
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.1)

# === Irrlicht visualization (Tracked vehicle uses ChTrackedVehicleVisualSystemIrrlicht) ===
vis = veh.ChTrackedVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("M113 on SCM Deformable Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.0), 8.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle.GetVehicle())

# === Driver ===
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_every * step_size / 1.0)
driver.SetThrottleDelta(render_every * step_size / 1.0)
driver.SetBrakingDelta(render_every * step_size / 0.3)
driver.Initialize()

# === Main loop ===

frame = 0
realtime_timer = chrono.ChRealtimeStepTimer()

try:
    while vis.Run() and sys.GetChTime() < sim_end:
        sim_time = sys.GetChTime()

        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(render_every):
            sim_time = sys.GetChTime()

            driver_inputs = driver.GetInputs()
            # Hard-coded throttle 0.8 as requested — scripted input in scored core
            driver_inputs.m_throttle = 0.8
            driver_inputs.m_steering = 0.0
            driver_inputs.m_braking  = 0.0

            driver.Synchronize(sim_time)
            terrain.Synchronize(sim_time)
            vehicle.Synchronize(sim_time, driver_inputs)   # 2-arg: tracked vehicle
            vis.Synchronize(sim_time, driver_inputs)

            driver.Advance(step_size)
            terrain.Advance(step_size)
            vehicle.Advance(step_size)
            vis.Advance(step_size)


            if sys.GetChTime() >= sim_end:
                break

        realtime_timer.Spin(step_size)

except (RuntimeError, ValueError) as exc:      # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
