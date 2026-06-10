"""
FEDA vehicle simulation with ISO double lane change path-follower driver.

System type: NSC (rigid terrain)
Vehicle: FED-Alpha (veh.FEDA) on RigidTerrain
Driver: ChPathFollowerDriver following an ISO double lane change path
Initial position: (-50, 0, 0.5) — offset to allow the maneuver to fit within the 200 m terrain patch
Expected behavior: vehicle accelerates to target speed of 10 m/s and performs the
double lane change maneuver autonomously using a steering PID controller.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

# === Named constants ===
step_size       = 2e-3          # physics time step (s)
sim_end         = 30.0          # simulation duration (s)
render_fps      = 50.0
render_every    = max(1, round(1.0 / (render_fps * step_size)))  # precomputed once

TERRAIN_LENGTH  = 200.0         # X (m) — extended to fit double lane change
TERRAIN_WIDTH   = 100.0         # Y (m)
INIT_LOC        = chrono.ChVector3d(-50.0, 0.0, 0.5)   # moved to allow maneuver
INIT_ROT        = chrono.ChQuaterniond(1, 0, 0, 0)

TARGET_SPEED    = 10.0          # m/s cruise-control target
LOOK_AHEAD      = 5.0           # m look-ahead for steering controller

# === Data paths (required truth components) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle setup ===
feda = veh.FEDA()
feda.SetContactMethod(chrono.ChContactMethod_NSC)
feda.SetChassisCollisionType(veh.CollisionType_NONE)
feda.SetChassisFixed(False)
feda.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
feda.SetEngineType(veh.EngineModelType_SIMPLE_MAP)
feda.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
feda.SetTireType(veh.TireModelType_PAC02)
feda.SetTireStepSize(step_size)
feda.Initialize()

# === System & bodies (created by veh.FEDA wrapper) ===
system = feda.GetSystem()                 # ChSystemNSC owned by the wrapper
chassis = feda.GetChassisBody()           # cache: fetched once, reused every step
# wheels/spindles: feda.GetVehicle().GetAxle(i)... ; terrain: RigidTerrain patch below
# joints: suspension + steering links created inside the wrapper

system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize

print("VEHICLE MASS: ", feda.GetVehicle().GetMass())

# Set visualization types (after Initialize)
feda.SetChassisVisualizationType(chrono.VisualizationType_MESH)
feda.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
feda.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
feda.SetWheelVisualizationType(chrono.VisualizationType_MESH)
feda.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === Terrain ===
terrain = veh.RigidTerrain(system)

patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    TERRAIN_LENGTH,
    TERRAIN_WIDTH
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Path-follower driver (ISO double lane change) ===
# The path starts at INIT_LOC; the maneuver begins from (-50, 0, 0.5).
path = veh.DoubleLaneChangePath(
    INIT_LOC,       # start position
    13.5,           # length of the lane-change segment (m)
    4.0,            # lateral width (m)
    11.0,           # longitudinal offset (m)
    50.0,           # total path length (m)
    True            # change to the left
)

driver = veh.ChPathFollowerDriver(
    feda.GetVehicle(),
    path,
    "double_lane_change",
    TARGET_SPEED
)
driver.GetSteeringController().SetLookAheadDistance(LOOK_AHEAD)
driver.GetSteeringController().SetGains(0.8, 0.0, 0.0)    # KP, KI, KD
driver.GetSpeedController().SetGains(0.4, 0.0, 0.0)
driver.Initialize()

# === Steering target marker (path-controller visualization) ===
target_marker = chrono.ChBody()
target_marker.SetFixed(True)
sphere_vis = chrono.ChVisualShapeSphere(0.1)
sphere_vis.SetColor(chrono.ChColor(1.0, 0.0, 0.0))
target_marker.AddVisualShape(sphere_vis, chrono.ChFramed())
system.AddBody(target_marker)

# === Visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("FEDA — ISO Double Lane Change")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(feda.GetVehicle())

# === Review-only setup ===


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
frame = 0

try:
    while vis.Run() and system.GetChTime() < sim_end:
        time = system.GetChTime()

        # Render at throttled cadence
        if step_number % render_every == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        # Update steering target marker each render frame
        if step_number % render_every == 0:
            target_marker.SetPos(driver.GetSteeringController().GetTargetLocation())

        driver_inputs = driver.GetInputs()


        driver.Synchronize(time)
        terrain.Synchronize(time)
        feda.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(step_size)
        terrain.Advance(step_size)
        feda.Advance(step_size)
        vis.Advance(step_size)

        step_number += 1
        realtime_timer.Spin(step_size)

except (RuntimeError, ValueError) as exc:   # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
