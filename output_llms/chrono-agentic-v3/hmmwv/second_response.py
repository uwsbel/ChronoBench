"""
HMMWV Circular Path-Following Simulation
=========================================
System type  : NSC (rigid terrain)
Vehicle      : HMMWV_Full with TMEASY tires
Terrain      : RigidTerrain, 200 m × 200 m flat patch
Driver       : ChPathFollowerDriver following a circular path
               - constant throttle = 0.3
               - PID steering controller
               - path visualized with two ball markers
               - sentinel and target point spheres rendered each frame
Expected     : HMMWV completes laps around a circular path; sentinel (blue)
               and target (red) sphere markers track the controller points.
"""

# === Imports ===
import os
import math
import csv

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

# === Named constants ===
STEP_SIZE          = 2e-3           # physics time step (s)
SIM_END            = 30.0           # simulation end time (s)
RENDER_FPS         = 50.0           # frames per second for recording

TERRAIN_LENGTH     = 200.0          # X-extent of terrain patch (m)
TERRAIN_WIDTH      = 200.0          # Y-extent of terrain patch (m)

PATH_RADIUS        = 30.0           # circular path radius (m)
PATH_RUN_IN        = 10.0           # straight run-in before circle (m)
PATH_TURNS         = 3              # number of laps

TARGET_SPEED       = 8.0            # desired cruise speed (m/s)
THROTTLE_FIXED     = 0.3            # constant throttle (path-follower uses speed controller)

INIT_LOC           = chrono.ChVector3d(-PATH_RADIUS, 0.0, 0.5)  # start inside run-in
INIT_ROT           = chrono.QuatFromAngleZ(0.0)

SUSPENSION_REF_H   = 0.5            # chassis origin height above wheel-bottom at rest (HMMWV)
TIRE_RADIUS        = 0.33           # approximate HMMWV tire radius (m) — for footprint check

# Precomputed once before the loop
render_every = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # # physics steps per frame

# === Data paths (truth-required) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Vehicle setup ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                                  # MANDATORY
hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(STEP_SIZE)
hmmwv.Initialize()

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system  = hmmwv.GetSystem()           # ChSystemNSC owned by the wrapper
chassis = hmmwv.GetChassisBody()      # cache: main chassis rigid body
# wheels/spindles: hmmwv.GetVehicle().GetAxle(i); terrain: RigidTerrain patch below
# joints: suspension + steering links created inside the wrapper

system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize

print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

# Footprint assertion — wheel bottom must be at or above z=0 (flat terrain)
veh_obj = hmmwv.GetVehicle()
spindle_world = []
for axle_idx in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        p = veh_obj.GetSpindlePos(axle_idx, side)
        spindle_world.append(p)

wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
TERRAIN_TOP_Z = 0.0
ZTOL = 0.1
assert wheel_bottom_z >= TERRAIN_TOP_Z - ZTOL, (
    f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs terrain top z={TERRAIN_TOP_Z:.3f}; raise SUSPENSION_REF_H by "
    f"{TERRAIN_TOP_Z - wheel_bottom_z:.3f} m"
)

# === Visualization types ===
hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# === Rigid terrain (200 m × 200 m) ===
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Circular path and driver ===
path_start = chrono.ChVector3d(INIT_LOC.x, INIT_LOC.y, INIT_LOC.z)
path = veh.CirclePath(path_start, PATH_RADIUS, PATH_RUN_IN, True, PATH_TURNS)

driver = veh.ChPathFollowerDriver(
    hmmwv.GetVehicle(),
    path,
    "circle_path",
    TARGET_SPEED,
)
driver.GetSteeringController().SetLookAheadDistance(5.0)
driver.GetSteeringController().SetGains(0.8, 0.0, 0.0)    # KP, KI, KD — PID steering
driver.GetSpeedController().SetGains(0.4, 0.0, 0.0)
driver.Initialize()

# === Path visualization: two ball markers at start and 90-degree points ===
# Ball 1 — at the path start
ball1 = chrono.ChBody()
ball1.SetFixed(True)
ball1.SetPos(path_start)
vis_ball1 = chrono.ChVisualShapeSphere(0.5)
vis_ball1.SetColor(chrono.ChColor(1.0, 1.0, 0.0))          # yellow
ball1.AddVisualShape(vis_ball1, chrono.ChFramed())
system.AddBody(ball1)

# Ball 2 — at the opposite side of the circle (180°)
ball2_pos = chrono.ChVector3d(
    INIT_LOC.x + 2.0 * PATH_RADIUS,
    INIT_LOC.y,
    INIT_LOC.z,
)
ball2 = chrono.ChBody()
ball2.SetFixed(True)
ball2.SetPos(ball2_pos)
vis_ball2 = chrono.ChVisualShapeSphere(0.5)
vis_ball2.SetColor(chrono.ChColor(1.0, 1.0, 0.0))          # yellow
ball2.AddVisualShape(vis_ball2, chrono.ChFramed())
system.AddBody(ball2)

# === Sentinel marker (blue sphere) ===
sentinel_marker = chrono.ChBody()
sentinel_marker.SetFixed(True)
sentinel_marker.SetPos(chrono.ChVector3d(0, 0, 0))
vis_sentinel = chrono.ChVisualShapeSphere(0.3)
vis_sentinel.SetColor(chrono.ChColor(0.0, 0.0, 1.0))       # blue
sentinel_marker.AddVisualShape(vis_sentinel, chrono.ChFramed())
system.AddBody(sentinel_marker)

# === Target marker (red sphere) ===
target_marker = chrono.ChBody()
target_marker.SetFixed(True)
target_marker.SetPos(chrono.ChVector3d(0, 0, 0))
vis_target = chrono.ChVisualShapeSphere(0.3)
vis_target.SetColor(chrono.ChColor(1.0, 0.0, 0.0))         # red
target_marker.AddVisualShape(vis_target, chrono.ChFramed())
system.AddBody(target_marker)

# === Visualization (ChWheeledVehicleVisualSystemIrrlicht) ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV Circular Path Follower")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(hmmwv.GetVehicle())

# === Review-only recording setup ===


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()     # cache: real-time pacing object
step_number = 0

try:
    while vis.Run() and system.GetChTime() < SIM_END:
        time = system.GetChTime()

        # Update sentinel / target markers once per frame (before render)
        if step_number % render_every == 0:
            sentinel_marker.SetPos(
                driver.GetSteeringController().GetSentinelLocation()
            )
            target_marker.SetPos(
                driver.GetSteeringController().GetTargetLocation()
            )
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()
        # Path-follower drives speed; apply constant throttle floor from prompt
        if driver_inputs.m_throttle < THROTTLE_FIXED:
            driver_inputs.m_throttle = THROTTLE_FIXED

        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)


        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        hmmwv.Advance(STEP_SIZE)
        vis.Advance(STEP_SIZE)

        step_number += 1
        realtime_timer.Spin(STEP_SIZE)

except (RuntimeError, ValueError) as exc:    # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
