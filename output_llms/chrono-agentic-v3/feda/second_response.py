"""
FEDA vehicle simulation with ISO double lane change path-follower driver.

System: ChSystemNSC (owned by FEDA wrapper), NSC rigid terrain.
Vehicle: FED-Alpha (FEDA) with PAC02 tires on a flat rigid terrain patch.
Driver: ChPathFollowerDriver following an ISO double lane change path.
The vehicle starts at (-50, 0, 0.5) and the terrain is 200 m long to
accommodate the full maneuver. Target cruise speed is 10.0 m/s.
Expected behavior: FEDA accelerates along X, performs the double lane
change maneuver (left offset, then back), and continues on the far side.
"""

import os
import math

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as virr

# === Named constants ===
TIME_STEP = 1e-3           # physics timestep [s]
SIM_END = 30.0             # simulation duration [s]
RENDER_FPS = 50.0          # Irrlicht render cadence [Hz]
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

TERRAIN_LENGTH = 200.0     # terrain patch length [m]  (changed from 100 → 200)
TERRAIN_WIDTH = 100.0      # terrain patch width [m]

INIT_X = -50.0             # vehicle initial X position (changed from 0 → -50)
INIT_Y = 0.0
INIT_Z = 0.5               # chassis height above ground at spawn

TARGET_SPEED = 10.0        # m/s cruise speed for path-follower
LOOKAHEAD_DIST = 5.0       # steering look-ahead distance [m]

# Path parameters for DoubleLaneChangePath
DLC_START_X = INIT_X       # start of the lane-change maneuver at vehicle spawn
DLC_LENGTH = 13.5          # maneuver lane-change section length [m]
DLC_WIDTH = 4.0            # lane width [m]
DLC_OFFSET = 11.0          # longitudinal offset to start of lateral shift [m]
DLC_TOTAL = 50.0 * 4       # total path length (long enough for full terrain) [m]

# === Data paths — required truth components ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Vehicle setup ===
feda = veh.FEDA()
feda.SetContactMethod(chrono.ChContactMethod_NSC)
feda.SetChassisCollisionType(veh.CollisionType_NONE)
feda.SetChassisFixed(False)

init_loc = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
feda.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))

feda.SetEngineType(veh.EngineModelType_SIMPLE_MAP)
feda.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
feda.SetTireType(veh.TireModelType_PAC02)
feda.SetTireStepSize(TIME_STEP)
feda.Initialize()

# === System & bodies (created by the veh.FEDA wrapper) ===
system = feda.GetSystem()                  # ChSystemNSC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize

chassis = feda.GetChassisBody()            # cache: main chassis rigid body
# wheels/spindles: feda.GetVehicle().GetAxle(i)... ; terrain: RigidTerrain below
# joints: suspension + steering links created inside the FEDA wrapper

print("VEHICLE MASS: ", feda.GetVehicle().GetMass())

# Set visualization types (VisualizationType lives in veh namespace for 9.0.0)
feda.SetChassisVisualizationType(veh.VisualizationType_MESH)
feda.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
feda.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
feda.SetWheelVisualizationType(veh.VisualizationType_MESH)
feda.SetTireVisualizationType(veh.VisualizationType_MESH)

# === Terrain ===
terrain = veh.RigidTerrain(system)

patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

# Center the patch at X=75 with length 300 so it spans from x=-75 to x=+225 (covers full maneuver)
terrain_center = chrono.ChCoordsysd(chrono.ChVector3d(75.0, 0.0, 0.0), chrono.QUNIT)
patch = terrain.AddPatch(
    patch_mat,
    terrain_center,
    300.0,        # extended length to cover vehicle path from -50 to ~220
    TERRAIN_WIDTH,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Path-follower driver ===
dlc_start = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)
path = veh.DoubleLaneChangePath(
    dlc_start,
    DLC_LENGTH,
    DLC_WIDTH,
    DLC_OFFSET,
    DLC_TOTAL,
    True,   # to left
)

driver = veh.ChPathFollowerDriver(
    feda.GetVehicle(),
    path,
    "double_lane_change",
    TARGET_SPEED,
)
driver.GetSteeringController().SetLookAheadDistance(LOOKAHEAD_DIST)
driver.GetSteeringController().SetGains(0.8, 0.0, 0.0)
driver.GetSpeedController().SetGains(0.4, 0.0, 0.0)
driver.Initialize()

# Marker body to visualize the steering target point (renderer-agnostic)
target_marker = chrono.ChBody()
target_marker.SetFixed(True)
marker_sphere = chrono.ChVisualShapeSphere(0.15)
marker_sphere.SetColor(chrono.ChColor(1.0, 0.0, 0.0))
target_marker.AddVisualShape(marker_sphere, chrono.ChFramed())
system.AddBody(target_marker)

# === Visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("FEDA Double Lane Change")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.AttachVehicle(feda.GetVehicle())
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()

# === Review-only recording setup ===

# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()

try:
    while vis.Run() and system.GetChTime() < SIM_END:
        time = system.GetChTime()

        vis.BeginScene()
        vis.Render()
        vis.EndScene()


        driver_inputs = driver.GetInputs()

        # Update target marker to current steering controller target
        target_marker.SetPos(driver.GetSteeringController().GetTargetLocation())

        driver.Synchronize(time)
        terrain.Synchronize(time)
        feda.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        for _ in range(RENDER_EVERY):
            driver.Advance(TIME_STEP)
            terrain.Advance(TIME_STEP)
            feda.Advance(TIME_STEP)
            vis.Advance(TIME_STEP)
            if system.GetChTime() >= SIM_END:
                break

        realtime_timer.Spin(TIME_STEP)

except (RuntimeError, ValueError) as exc:  # solver divergence / bad physics state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
