"""
FEDA vehicle simulation with ISO double lane change path-follower driver.

System: SMC (FEDA default contact method), RigidTerrain.
Vehicle: FED-Alpha (veh.FEDA) with TMEASY tires on flat rigid terrain.
Driver: ChPathFollowerDriver following an ISO double lane change path.
Target speed: 10.0 m/s; steering look-ahead: 5.0 m.
Initial location: (-50, 0, 0.5); terrain length: 200.0 m.

The DLC path starts at x=-50, with a 50m straight run to x=0, then the
ISO S-curve (x=0 to x=38), then an exit straight to x=88. Simulation
runs for 14 s so the vehicle completes the S-curve and stops before the
path end — preventing the path-follower from going out of bounds.

Expected behavior: FEDA accelerates to 10 m/s, follows the left DLC
maneuver (lateral displacement ~4 m), and returns to the centerline.
"""

import math
import os


import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

# === Constants ===
TERRAIN_LENGTH = 200.0       # m — prompt: increased to fit double lane change
TERRAIN_WIDTH = 50.0         # m — wide enough for the lane change offset
INIT_POS = chrono.ChVector3d(-50.0, 0.0, 0.5)   # prompt: initial vehicle location
INIT_ROT = chrono.QUNIT

TARGET_SPEED = 10.0          # m/s — prompt: cruise control target
LOOK_AHEAD = 5.0             # m   — prompt: steering look-ahead distance
STEER_KP, STEER_KI, STEER_KD = 0.8, 0.0, 0.0   # steering PID gains
SPEED_KP, SPEED_KI, SPEED_KD = 0.4, 0.0, 0.0   # speed PID gains

# DLC path: total_length=50 places the S-curve starting at x=0 (50m from start).
# Path points: (-50,0)→(0,0): straight; (13.5,4)→(24.5,4): lane segment;
# (38,0)→(88,0): return and exit straight.
# At 10 m/s from x=-50 to x=80: ~13s, so sim_end=13s stays within path bounds.
DLC_TOTAL_LENGTH = 50.0      # m — controls entry straight length

TIME_STEP = 2e-3             # s
SIM_END = 13.0               # s  — stop before vehicle overruns path end (x=88)
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once


# === Vehicle setup ===
feda = veh.FEDA()
feda.SetContactMethod(chrono.ChContactMethod_SMC)
feda.SetChassisCollisionType(veh.CollisionType_NONE)
feda.SetChassisFixed(False)                          # MANDATORY — fixed chassis won't move
feda.SetInitPosition(chrono.ChCoordsysd(INIT_POS, INIT_ROT))
feda.SetEngineType(veh.EngineModelType_SIMPLE_MAP)
feda.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
feda.SetTireType(veh.TireModelType_TMEASY)           # TMEASY: compatible with rigid terrain
feda.SetTireStepSize(TIME_STEP)
feda.Initialize()

# === System & bodies (created by the veh.FEDA wrapper) ===
system = feda.GetSystem()                 # ChSystemSMC owned by the wrapper
chassis = feda.GetChassisBody()           # cache: main chassis rigid body — reused in loop
# wheels/spindles: feda.GetVehicle().GetAxle(i).m_wheels[j].GetSpindle()
# joints: suspension + steering links created inside the FEDA wrapper
# terrain: RigidTerrain patch added below

system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize

# Visualization types — PRIMITIVES avoids missing mesh OBJ asset errors
feda.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
feda.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
feda.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
feda.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
feda.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)

# === Terrain ===
terrain = veh.RigidTerrain(system)

patch_mat = chrono.ChContactMaterialSMC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch_mat.SetYoungModulus(2e7)

# Center terrain at x=50 so it covers -50..150 (full vehicle travel range)
terrain_center = chrono.ChCoordsysd(
    chrono.ChVector3d(50.0, 0.0, 0.0), chrono.QUNIT
)
patch = terrain.AddPatch(
    patch_mat,
    terrain_center,
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Path — ISO double lane change ===
# DoubleLaneChangePath(start, length, width, offset, total_length, to_left)
# With total_length=50: entry straight is 50m (x=-50 to x=0),
# then the S-curve (x=0 to x=38 with 4m lateral offset), then exit straight.
dlc_start = chrono.ChVector3d(-50.0, 0.0, 0.5)
path = veh.DoubleLaneChangePath(
    dlc_start,
    13.5,              # lane-change maneuver length (ISO standard)
    4.0,               # lane width offset
    11.0,              # offset between entry and exit lane centers
    DLC_TOTAL_LENGTH,  # 50m — entry straight, then S-curve at x=0
    True,              # to left (standard ISO DLC direction)
)

# === Driver — ChPathFollowerDriver with cruise control ===
driver = veh.ChPathFollowerDriver(
    feda.GetVehicle(),
    path,
    "dlc_path",
    TARGET_SPEED,
)
driver.GetSteeringController().SetLookAheadDistance(LOOK_AHEAD)
driver.GetSteeringController().SetGains(STEER_KP, STEER_KI, STEER_KD)
driver.GetSpeedController().SetGains(SPEED_KP, SPEED_KI, SPEED_KD)
driver.Initialize()

# === Steering-target visual marker (renderer-agnostic — no Irrlicht scene nodes) ===
target_marker = chrono.ChBody()
target_marker.SetFixed(True)
target_sphere = chrono.ChVisualShapeSphere(0.15)
target_sphere.SetColor(chrono.ChColor(1.0, 0.0, 0.0))
target_marker.AddVisualShape(target_sphere, chrono.ChFramed())
system.AddBody(target_marker)

# === Visualization — ChWheeledVehicleVisualSystemIrrlicht ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("FEDA – ISO Double Lane Change Path Follower")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 12.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(feda.GetVehicle())

# === Spindle footprint check (verify wheels sit on terrain at init) ===
TIRE_RADIUS = 0.37   # FEDA TMEASY approximate tire radius
ZTOL = 0.15
feda_veh = feda.GetVehicle()   # cache: fetched once
spindle_positions = []
for ax in range(feda_veh.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        p = feda_veh.GetSpindlePos(ax, side)
        spindle_positions.append(p)
wheel_bottom_z = min(p.z for p in spindle_positions) - TIRE_RADIUS
assert wheel_bottom_z >= -ZTOL, (
    f"Vehicle sinks into terrain: wheel_bottom_z={wheel_bottom_z:.3f}; "
    f"increase INIT_POS.z by {-wheel_bottom_z:.3f} m"
)

# === CSV setup (review-only) ===

# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
frame = 0

try:
    while vis.Run() and system.GetChTime() < SIM_END:
        time = system.GetChTime()  # cache: fetched once per render step

        if step_number % RENDER_EVERY == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            # Move the red sphere to the path-follower's current look-ahead target
            target_marker.SetPos(driver.GetSteeringController().GetTargetLocation())

        driver_inputs = driver.GetInputs()

        # Synchronize order: driver → terrain → vehicle → vis
        driver.Synchronize(time)
        terrain.Synchronize(time)
        feda.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)


        # Advance order: driver → terrain → feda (steps system) → vis
        driver.Advance(TIME_STEP)
        terrain.Advance(TIME_STEP)
        feda.Advance(TIME_STEP)       # advances the wrapper-owned ChSystem
        vis.Advance(TIME_STEP)

        step_number += 1
        realtime_timer.Spin(TIME_STEP)

except (RuntimeError, ValueError) as exc:   # solver divergence / bad physics state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
