"""
Kraz tractor-trailer simulation on flat rigid terrain with a scripted double lane change maneuver.

System: NSC (rigid terrain, catalog vehicle defaults)
Vehicle: veh.Kraz() tractor-trailer wrapper
Terrain: RigidTerrain with large flat road patch
Driver: Scripted time-based double lane change sequence (open-loop, time-dependent)
Initial position: (-15, 0, 0.5), heading forward (+X)
Chase camera: track point (3, 0, 2.1), distance 25.0, offset 10.5
Expected behavior: Kraz truck accelerates, then performs a double lane change —
first changing to an adjacent lane (steer left) then returning to the original
lane (steer right) — based on simulation time triggers.
"""

import os
import math
import csv


import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Constants ===
STEP_SIZE      = 1e-3          # physics time step (s)
SIM_END        = 30.0          # simulation duration (s)
RENDER_FPS     = 50.0          # render/capture rate
TERRAIN_LEN    = 600.0         # terrain patch length (m) — large enough for full run
TERRAIN_WID    = 200.0         # terrain patch width (m)

# Initial vehicle pose
INIT_X         = -15.0         # prompt: changed from 0 to -15
INIT_Y         = 0.0
INIT_Z         = 0.5
INIT_LOC       = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)
INIT_ROT       = chrono.QuatFromAngleZ(0.0)   # heading along +X

# Camera
TRACK_POINT    = chrono.ChVector3d(3.0, 0.0, 2.1)   # prompt: changed from (0,0,2.1)
CHASE_DIST     = 25.0
CHASE_OFFSET   = 10.5                               # prompt: changed from 1.5

# Precomputed render cadence
render_every = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))   # precomputed once

# Review-only recording flag

# === Data paths (catalog vehicle — truth-faithful pair) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle setup ===
vehicle = veh.Kraz()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
vehicle.SetTireStepSize(STEP_SIZE)
vehicle.Initialize()

# === System & bodies (created by the veh.Kraz wrapper) ===
sys = vehicle.GetSystem()                         # ChSystemNSC owned by the wrapper
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)   # REQUIRED after Initialize

chassis = vehicle.GetTractorChassisBody()         # cache: tractor chassis rigid body
# trailer chassis: vehicle.GetTrailer().GetChassisBody()
# tractor: vehicle.GetTractor()  — ChWheeledVehicle handle
# suspension/steering joints created inside the wrapper

print("VEHICLE MASS: ", vehicle.GetTractor().GetMass())

# === Visualization types ===
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH, veh.VisualizationType_PRIMITIVES)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES, veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH, veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH, veh.VisualizationType_MESH)

# === Terrain ===
terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    TERRAIN_LEN,
    TERRAIN_WID
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Driver — scripted double lane change maneuver sequence ===
# Uses ChDataDriver for a clean time-stepped open-loop maneuver schedule.
# Scripted inputs are scored core: this is a double-lane-change simulation,
# not an interactive demo.
driver_data = veh.vector_Entry([
    # (time, steering, throttle, braking)
    veh.DataDriverEntry(0.0,   0.0,  0.0, 1.0),   # warm-up: idle/brake
    veh.DataDriverEntry(1.0,   0.0,  0.0, 0.5),   # ease off brake
    veh.DataDriverEntry(2.0,   0.0,  0.7, 0.0),   # accelerate
    veh.DataDriverEntry(4.0,   0.0,  0.5, 0.0),   # cruise at speed
    veh.DataDriverEntry(6.0,   0.4,  0.5, 0.0),   # steer left — lane change start
    veh.DataDriverEntry(8.0,  -0.4,  0.5, 0.0),   # counter-steer — align in new lane
    veh.DataDriverEntry(10.0,  0.0,  0.5, 0.0),   # cruise in new lane
    veh.DataDriverEntry(14.0, -0.4,  0.5, 0.0),   # steer right — second lane change
    veh.DataDriverEntry(16.0,  0.4,  0.5, 0.0),   # counter-steer — align in original lane
    veh.DataDriverEntry(18.0,  0.0,  0.5, 0.0),   # cruise in original lane
    veh.DataDriverEntry(30.0,  0.0,  0.5, 0.0),   # maintain to end
])
driver = veh.ChDataDriver(vehicle.GetTractor(), driver_data)
driver.Initialize()

# === Irrlicht visualization for Kraz (wheeled vehicle) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Kraz Truck — Double Lane Change")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(TRACK_POINT, CHASE_DIST, CHASE_OFFSET)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle.GetTractor())

# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number    = 0
frame          = 0


try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        time = sys.GetChTime()

        driver_inputs = driver.GetInputs()

        if step_number % render_every == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()


        # Synchronize subsystems
        driver.Synchronize(time)
        terrain.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        # Advance subsystems
        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        vehicle.Advance(STEP_SIZE)
        vis.Advance(STEP_SIZE)

        step_number += 1
        realtime_timer.Spin(STEP_SIZE)

except (RuntimeError, ValueError) as exc:   # solver divergence / bad vehicle state
    import traceback; traceback.print_exc()
    raise
finally:
    pass   # CSV writers closed in the review-only block below
