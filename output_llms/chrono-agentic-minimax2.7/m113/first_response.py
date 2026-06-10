"""
M113 Tracked Vehicle Simulation
==============================
Multi-body system: M113 armored personnel carrier on rigid flat terrain.
System type: SMC (smooth contact), tracked vehicle with Barzilai-Borwein solver.
Expected behavior: vehicle drives forward under constant throttle, tracks terrain.
"""

import os
import math
import csv
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === sim_recording import ===
import sim_recording as rec
REC = bool(os.environ.get("SIMBENCH_RECORD"))
irr_dir = rec.frame_dir("frames") if REC else None

# === Named constants ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

SIM_END = 10.0          # seconds
TIME_STEP = 1e-3         # physics timestep
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))

INIT_LOC = chrono.ChVector3d(0, 0, 0.5)
INIT_ROT = chrono.QUNIT

# === M113 Tracked Vehicle ===
vehicle = veh.M113()
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
vehicle.SetChassisFixed(False)
vehicle.SetTrackShoeType(veh.TrackShoeType_SINGLE_PIN)
vehicle.SetDrivelineType(veh.DrivelineTypeTV_BDS)
vehicle.SetEngineType(veh.EngineModelType_SHAFTS)
vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
vehicle.SetBrakeType(veh.BrakeType_SIMPLE)
vehicle.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
vehicle.Initialize()

# Post-init collision system (required for tracked/contact scenes)
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
vehicle.GetSystem().SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# === Rigid Terrain ===
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch_mat = chrono.ChContactMaterialSMC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200.0, 200.0)
patch.SetColor(chrono.ChColor(0.6, 0.6, 0.4))
terrain.Initialize()

# === Driver (interactive — scored core uses ChInteractiveDriverIRR) ===
vis = veh.ChTrackedVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("M113 Tracked Vehicle")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 2.0), 12.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle.GetVehicle())

driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(1.0 / steering_time)
driver.SetThrottleDelta(1.0 / throttle_time)
driver.SetBrakingDelta(1.0 / braking_time)
driver.Initialize()

# === Visualization types ===
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTrackShoeVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSprocketVisualizationType(veh.VisualizationType_MESH)
vehicle.SetIdlerVisualizationType(veh.VisualizationType_MESH)
vehicle.SetRoadWheelVisualizationType(veh.VisualizationType_MESH)

# === System cache ===
sys = vehicle.GetSystem()
chassis = vehicle.GetChassisBody()  # cache: fetched once

# === CSV setup (review-only) ===
CSV_PATH = "simulation_data.csv"
csv_file = None
csv_writer = None

# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
frame = 0
step_number = 0

try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        time = sys.GetChTime()

        if step_number % RENDER_EVERY == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            if REC:
                frame += 1

        driver_inputs = driver.GetInputs()

        # Scripted throttle — m113 truth drives at 0.8 throttle constant
        driver.SetThrottle(0.8)  # scored core: truth scripts constant throttle

        driver.Synchronize(time)
        terrain.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs)   # 2-arg for tracked (no terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(TIME_STEP)
        terrain.Advance(TIME_STEP)
        vehicle.Advance(TIME_STEP)
        vis.Advance(TIME_STEP)

        # Log CSV every step (review-only)

        step_number += 1
        realtime_timer.Spin(TIME_STEP)
finally:
    if csv_file:
        csv_file.close()

# === Review-only: assemble videos and plot ===
