"""
HMMWV Circular Path Follower Simulation (PyChrono 9.0.x, Irrlicht)

Models a full HMMWV vehicle autonomously following a circular path of radius 40 m
using ChPathFollowerDriver with PID steering control (Kp=0.8, Ki=0, Kd=0) and a
constant throttle of 0.3. The terrain is a flat RigidTerrain (NSC contact) sized
200 x 200 m (increased from 100 m to fit the circular path). Two fixed-body sphere
markers (red = target point, blue = sentinel point) visualize the steering controller
output for the user. The simulation uses veh.ChWheeledVehicleVisualSystemIrrlicht.

System: ChSystemNSC (owned by the veh.HMMWV_Full wrapper)
Main bodies: HMMWV chassis, four wheel spindles/tires, flat terrain patch,
             sentinel sphere marker, target sphere marker
Expected behavior: HMMWV drives a left-turning circular arc continuously;
    the sentinel and target markers follow the PID steering controller points.
"""

import os
import math
import csv

import pychrono.core as chrono
import pychrono.vehicle as veh


# Set vehicle data path so veh.GetVehicleDataFile resolves bundled assets
veh.SetVehicleDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Named constants ===
# Simulation timing
TIME_STEP = 5e-4             # physics time step (s)
SIM_END = 30.0               # total simulation duration (s)
RENDER_FPS = 50.0            # visualization frame rate (Hz)
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# Terrain
TERRAIN_LENGTH = 200.0       # X extent (m) — 200 m to fit circular path
TERRAIN_WIDTH  = 200.0       # Y extent (m)

# Vehicle spawn — HMMWV chassis origin sits ~0.5 m above wheel-bottom at rest
INIT_X = 0.0
INIT_Y = -40.0               # south of circle center; path run-in heads +X
INIT_Z = 0.5                 # chassis height above flat terrain (z=0)

# Circular path
PATH_RADIUS  = 40.0          # arc radius (m)
PATH_RUN_IN  = 10.0          # straight run-in length before circle (m)
PATH_TURNS   = 3             # laps

# Driver / speed
TARGET_SPEED     = 8.0       # target cruise speed (m/s)
THROTTLE_CONST   = 0.3       # constant throttle override (review-only video)
STEER_LOOKAHEAD  = 5.0       # look-ahead distance (m) — appropriate for this radius
STEER_KP, STEER_KI, STEER_KD = 0.8, 0.0, 0.0   # PID steering gains
SPEED_KP,  SPEED_KI,  SPEED_KD  = 0.4, 0.0, 0.0

# Path-point sphere markers
BALL_RADIUS = 0.5            # visual radius of sentinel/target spheres (m)

# Review-only flag (set by SIMBENCH_RECORD env var)


# === Vehicle setup ===
# HMMWV_Full creates and owns a ChSystemNSC internally — do NOT pass a system
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)    # NSC matches rigid-terrain truths
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                           # MANDATORY — must be free to move

init_rot = chrono.QuatFromAngleZ(math.pi / 2)          # vehicle nose points +X
hmmwv.SetInitPosition(chrono.ChCoordsysd(
    chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z),
    init_rot,
))
hmmwv.SetTireType(veh.TireModelType_RIGID)             # RIGID tires: correct for rigid terrain
hmmwv.SetTireStepSize(TIME_STEP)
hmmwv.Initialize()

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system  = hmmwv.GetSystem()          # ChSystemNSC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize
chassis = hmmwv.GetChassisBody()     # cache: chassis rigid body, fetched once
# wheels/spindles: hmmwv.GetVehicle().GetAxle(i).m_wheels[j].GetSpindle()
# joints: suspension + steering links created inside the HMMWV_Full wrapper

# Visualization types (must come AFTER Initialize)
hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)


# === Terrain (200 x 200 m rigid flat patch) ===
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,         # centered at origin
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
)
patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


# === Circular path ===
# CirclePath: straight run-in from start, then circular arc left
path_start = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)
path = veh.CirclePath(path_start, PATH_RADIUS, PATH_RUN_IN, True, PATH_TURNS)

# === Path-follower driver (replaces interactive driver for autonomous path following) ===
driver = veh.ChPathFollowerDriver(
    hmmwv.GetVehicle(),
    path,
    "circle_path",
    TARGET_SPEED,
)
# PID steering controller with appropriate gains for the circular path
steer_ctrl = driver.GetSteeringController()   # cache: fetched once, reused every frame
steer_ctrl.SetLookAheadDistance(STEER_LOOKAHEAD)
steer_ctrl.SetGains(STEER_KP, STEER_KI, STEER_KD)
# PID speed controller
speed_ctrl = driver.GetSpeedController()      # cache: fetched once
speed_ctrl.SetGains(SPEED_KP, SPEED_KI, SPEED_KD)
driver.Initialize()


# === Path visualization — sentinel and target sphere markers ===
# Sentinel marker: look-ahead point on the path (blue)
sentinel_marker = chrono.ChBody()
sentinel_marker.SetFixed(True)
sentinel_marker.SetName("sentinel_marker")
sentinel_sph = chrono.ChVisualShapeSphere(BALL_RADIUS)
sentinel_sph.SetColor(chrono.ChColor(0.0, 0.0, 1.0))
sentinel_marker.AddVisualShape(sentinel_sph, chrono.ChFramed())
system.AddBody(sentinel_marker)

# Target marker: current steering target on the path (red)
target_marker = chrono.ChBody()
target_marker.SetFixed(True)
target_marker.SetName("target_marker")
target_sph = chrono.ChVisualShapeSphere(BALL_RADIUS)
target_sph.SetColor(chrono.ChColor(1.0, 0.0, 0.0))
target_marker.AddVisualShape(target_sph, chrono.ChFramed())
system.AddBody(target_marker)


# === Irrlicht visualization (vehicle-specific visual system) ===
# Order: configure FIRST, Initialize(), then scene elements AFTER — Irrlicht inverse of VSG
import pychrono.irrlicht as chronoirr  # noqa: E402 — placed here near vis block
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV Circular Path Follower — 200 m terrain, PID steering")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(hmmwv.GetVehicle())
vis.AttachDriver(driver)    # enables input-bar HUD in the Irrlicht window


# === Review-only: CSV setup and frame directory ===


# === Main simulation loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
frame = 0

try:
    while vis.Run() and hmmwv.GetSystem().GetChTime() < SIM_END:
        sim_time = hmmwv.GetSystem().GetChTime()  # cache: fetched once per frame

        # --- Render at RENDER_FPS cadence ---
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        # Update path-point sphere markers each render frame (cheap fixed-body move)
        sentinel_marker.SetPos(steer_ctrl.GetSentinelLocation())
        target_marker.SetPos(steer_ctrl.GetTargetLocation())

        # --- Physics sub-steps between render frames ---
        for _ in range(RENDER_EVERY):
            sim_time = hmmwv.GetSystem().GetChTime()

            driver_inputs = driver.GetInputs()


            # Synchronize in fixed order: driver → terrain → vehicle → vis
            driver.Synchronize(sim_time)
            terrain.Synchronize(sim_time)
            hmmwv.Synchronize(sim_time, driver_inputs, terrain)
            vis.Synchronize(sim_time, driver_inputs)


            driver.Advance(TIME_STEP)
            terrain.Advance(TIME_STEP)
            hmmwv.Advance(TIME_STEP)   # steps the wrapper-owned ChSystem — no DoStepDynamics
            vis.Advance(TIME_STEP)

            if hmmwv.GetSystem().GetChTime() >= SIM_END:
                break

        realtime_timer.Spin(TIME_STEP)

except (RuntimeError, ValueError) as exc:   # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
