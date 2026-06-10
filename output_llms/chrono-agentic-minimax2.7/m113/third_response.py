"""
M113 Tracked Vehicle Simulation
==============================
Demo: m113 (M113 armored personnel carrier)
Plan type: mbs_in_scene (tracked vehicle + terrain)

Scene: M113 tracked vehicle drives over rigid flat terrain with a mobility box obstacle.
- Tracked vehicle with SMC contact (track-terrain contact requires SMC)
- Rigid flat terrain
- Scripted throttle (0.8) hard-coded in the loop — scored core
- Added long box obstacle for mobility testing

Physics: ChSystemSMC, tracked vehicle (2-arg Synchronize), rigid terrain.
"""

import os
import math
import csv
import numpy as np

# review-only: sim_recording for frame capture, CSV, video assembly

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# =============================================================================
# === Named constants (from prompt + derived) ===
VEHICLE_INIT_X = -5.0
VEHICLE_INIT_Y = 0.0
VEHICLE_INIT_Z = 0.5

TERRAIN_LENGTH = 80.0
TERRAIN_WIDTH = 40.0

BOX_LENGTH = 8.0     # long box for mobility test
BOX_WIDTH = 2.0
BOX_HEIGHT = 0.6
BOX_X = 2.0          # box directly in vehicle path (vehicle drives from -5 toward +x)
BOX_Y = 0.0
BOX_Z = BOX_HEIGHT / 2.0  # sit on terrain surface

TIME_STEP = 5e-4
SIM_END = 12.0
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))


# =============================================================================
# === System & gravity ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# SMC required for tracked vehicle (track-terrain contact uses SMC)
system = chrono.ChSystemSMC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# =============================================================================
# === M113 Tracked Vehicle ===
vehicle = veh.M113()
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
vehicle.SetChassisFixed(False)
vehicle.SetTrackShoeType(veh.TrackShoeType_SINGLE_PIN)
vehicle.SetDrivelineType(veh.DrivelineTypeTV_BDS)
vehicle.SetEngineType(veh.EngineModelType_SHAFTS)
vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
vehicle.SetBrakeType(veh.BrakeType_SIMPLE)
init_pos = chrono.ChCoordsysd(
    chrono.ChVector3d(VEHICLE_INIT_X, VEHICLE_INIT_Y, VEHICLE_INIT_Z),
    chrono.QUNIT,
)
vehicle.SetInitPosition(init_pos)
vehicle.Initialize()

# Post-init collision system (required for tracked/terrain contact)
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
vehicle.GetSystem().SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# Set visualization types for tracked vehicle parts (so vehicle is visible)
# Use integer values: MESH=2, PRIMITIVES=1, NONE=0 (source build lacks these constants)
vehicle.SetChassisVisualizationType(2)          # MESH
vehicle.SetTrackShoeVisualizationType(2)        # MESH
vehicle.SetSprocketVisualizationType(2)         # MESH
vehicle.SetIdlerVisualizationType(2)            # MESH
vehicle.SetRoadWheelVisualizationType(2)        # MESH

# Fetch handles for visibility
sys = vehicle.GetSystem()  # cache: system from wrapper
chassis = vehicle.GetChassisBody()

# =============================================================================
# === Terrain (Rigid flat) ===
terrain_mat = chrono.ChContactMaterialSMC()
terrain_mat.SetFriction(0.9)
terrain_mat.SetRestitution(0.01)
terrain_mat.SetYoungModulus(2e7)

terrain = veh.RigidTerrain(sys)
patch = terrain.AddPatch(
    terrain_mat,
    chrono.CSYSNORM,
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
terrain.Initialize()

# =============================================================================
# === Mobility Box Obstacle ===
box_mat = chrono.ChContactMaterialSMC()
box_mat.SetFriction(0.8)
box_mat.SetRestitution(0.01)

box_body = chrono.ChBodyEasyBox(
    BOX_LENGTH, BOX_WIDTH, BOX_HEIGHT,
    1000.0,  # density
    True,     # visualize
    True,     # collide
    box_mat,
)
box_body.SetPos(chrono.ChVector3d(BOX_X, BOX_Y, BOX_Z))
box_body.SetFixed(False)  # dynamic so vehicle can push/collide with it
sys.AddBody(box_body)

# Ensure collision system knows about new body
sys.GetCollisionSystem().BindAll()

# =============================================================================
# === Visualization (full Irrlicht block — must precede driver) ===
vis = veh.ChTrackedVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("M113 Tracked Vehicle")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 2.0), 12.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AddTypicalLights()
vis.AttachVehicle(vehicle.GetVehicle())

# =============================================================================
# === Driver (scripted throttle — scored core, not review-only) ===
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(TIME_STEP / steering_time)
driver.SetThrottleDelta(TIME_STEP / throttle_time)
driver.SetBrakingDelta(TIME_STEP / braking_time)
driver.Initialize()

# =============================================================================
# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
frame = 0


while vis.Run() and sys.GetChTime() < SIM_END:
    # Throttled rendering
    if step_number % RENDER_EVERY == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        # review-only: capture frame
            vis.WriteImageToFile(rec.frame_path(irr_dir, frame))
            frame += 1

    sim_time = sys.GetChTime()

    # Scripted throttle (0.8) — hard-coded in loop, scored core
    driver.SetThrottle(0.8)  # prompt: hard-coded throttle value
    driver.SetSteering(0.0)
    driver.SetBraking(0.0)

    driver_inputs = driver.GetInputs()

    # Synchronize order: driver → vehicle → vis (tracked uses 2-arg, no terrain)
    driver.Synchronize(sim_time)
    vehicle.Synchronize(sim_time, driver_inputs)
    vis.Synchronize(sim_time, driver_inputs)

    driver.Advance(TIME_STEP)
    vehicle.Advance(TIME_STEP)
    vis.Advance(TIME_STEP)

    step_number += 1
    realtime_timer.Spin(TIME_STEP)

    # review-only CSV logging
        chassis_pos = chassis.GetPos()
        speed = chassis.GetLinVel().Length()
        box_pos = box_body.GetPos()
        data_writer.writerow([
            sim_time,
            chassis_pos.x, chassis_pos.z, speed,
            driver_inputs.m_throttle,
            box_pos.x, box_pos.z,
        ])
        for body_item, name in [(chassis, "chassis"), (box_body, "box")]:
            pos = body_item.GetPos()
            vel = body_item.GetLinVel()
            motion_writer.writerow([
                sim_time, name,
                pos.x, pos.y, pos.z,
                vel.x, vel.y, vel.z,
            ])
