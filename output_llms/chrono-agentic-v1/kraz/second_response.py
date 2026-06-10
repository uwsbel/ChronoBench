"""
Kraz tractor-trailer simulation with a double lane change maneuver.

System type: ChSystemNSC (NSC contact, rigid terrain).
Main bodies: Kraz tractor + trailer (wrapper-managed), rigid terrain patch.
Expected behaviour: The Kraz truck starts at (-15, 0, 0.5), accelerates forward
and executes a time-controlled double lane change maneuver.  The chase camera
follows the tractor from behind at distance 25 m with a 10.5 m vertical offset,
tracking at (3, 0, 2.1) on the tractor frame.
"""

import os
import math
import csv

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


# === Constants ===
STEP_SIZE = 1e-3          # physics timestep (s)
SIM_END   = 30.0          # total simulation time (s)
RENDER_FPS = 50.0         # target render framerate
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

# Vehicle initial pose
INIT_LOC = chrono.ChVector3d(-15.0, 0.0, 0.5)
INIT_ROT  = chrono.QuatFromAngleZ(0.0)          # facing +X

# Camera chase params (per prompt)
CAM_TRACK_PT = chrono.ChVector3d(3.0, 0.0, 2.1)
CAM_DISTANCE = 25.0
CAM_OFFSET   = 10.5

# Double lane change path parameters
TARGET_SPEED = 12.0       # m/s cruise speed

# Terrain dimensions
TERRAIN_LEN = 500.0
TERRAIN_WID = 200.0

# Review-only recording flag

# === Vehicle setup ===
vehicle = veh.Kraz()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
vehicle.SetTireStepSize(STEP_SIZE)
vehicle.Initialize()

# === System & bodies (created by the veh.Kraz wrapper) ===
system  = vehicle.GetSystem()                    # ChSystemNSC owned by the wrapper
chassis = vehicle.GetTractorChassisBody()        # tractor chassis rigid body
# trailer: vehicle.GetTrailer(); joints: suspension + hitch created inside wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED

# Visualization types (must come after Initialize)
vehicle.SetChassisVisualizationType(
    chrono.VisualizationType_MESH, chrono.VisualizationType_MESH
)
vehicle.SetSuspensionVisualizationType(
    chrono.VisualizationType_PRIMITIVES, chrono.VisualizationType_PRIMITIVES
)
vehicle.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(
    chrono.VisualizationType_MESH, chrono.VisualizationType_MESH
)
vehicle.SetTireVisualizationType(
    chrono.VisualizationType_MESH, chrono.VisualizationType_MESH
)

# === Terrain ===
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    TERRAIN_LEN,
    TERRAIN_WID,
)
patch.SetTexture(chrono.GetChronoDataFile("vehicle/terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Driver — double lane change path follower ===
# DoubleLaneChangePath(start, ramp, width, length, run, left_turn)
path_start = chrono.ChVector3d(-15.0, 0.0, 0.5)
path = veh.DoubleLaneChangePath(
    path_start,
    13.5,   # ramp length (m)
    4.0,    # lane width (m)
    11.0,   # length of offset section (m)
    100.0,  # total run length (m)
    True,   # left turn first
)

driver = veh.ChPathFollowerDriver(
    vehicle.GetTractor(),
    path,
    "dlc_path",
    TARGET_SPEED,
)
driver.GetSteeringController().SetLookAheadDistance(5.0)
driver.GetSteeringController().SetGains(0.8, 0.0, 0.0)
driver.GetSpeedController().SetGains(0.4, 0.0, 0.0)
driver.Initialize()

# === Visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Kraz Double Lane Change")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(CAM_TRACK_PT, CAM_DISTANCE, CAM_OFFSET)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(vehicle.GetTractor())

# === CSV data writer (review-only) ===

# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
frame = 0

try:
    while vis.Run() and system.GetChTime() < SIM_END:
        time = system.GetChTime()

        # --- Render (throttled) ---
        if step_number % RENDER_EVERY == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        # --- Driver inputs ---
        driver_inputs = driver.GetInputs()


        # --- Synchronize ---
        driver.Synchronize(time)
        terrain.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        # --- CSV logging (review-only) ---

        # --- Advance ---
        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        vehicle.Advance(STEP_SIZE)
        vis.Advance(STEP_SIZE)

        step_number += 1
        realtime_timer.Spin(STEP_SIZE)

except (RuntimeError, ValueError) as exc:     # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
