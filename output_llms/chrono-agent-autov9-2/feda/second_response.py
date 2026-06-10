"""FEDA wheeled vehicle performing an ISO double lane change maneuver.

Models the FED-Alpha (FEDA) catalog wheeled vehicle (SMC contact) driving on a
flat rigid terrain patch. The vehicle is controlled by an autonomous
path-follower / cruise-control driver (veh.ChPathFollowerDriver) that tracks an
ISO standard double-lane-change Bezier path at a constant target speed of
10 m/s. The vehicle spawns at world (-50, 0, 0.5) so the full maneuver fits
inside a 200 m long rigid terrain patch.

System type: SMC (the FEDA wrapper owns its ChSystemSMC).
Main bodies: FEDA chassis + 4 wheels/spindles, plus the rigid terrain patch.
Expected behavior: the vehicle accelerates to ~10 m/s, then steers left-then-
right-then-back (the double lane change) while the speed controller holds the
target speed, and finishes upright on the terrain.
"""

import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh

# === Parameters === geometry / physics constants and derived spawn pose
time_step = 1e-3                      # integration step (s)
tire_step = 1e-3                      # tire force-model step (s)
sim_end = 14.0                        # completes the double lane change within the patch
render_fps = 50.0                     # review-video frame rate
render_every = max(1, round(1.0 / (render_fps * time_step)))   # precomputed once

terrain_length = 200.0                # X size (m) — fits the lane-change run
terrain_width = 20.0                  # Y size (m)
terrain_height = 0.0                  # top surface Z (m)

target_speed = 10.0                   # path-follower cruise speed (m/s)
look_ahead = 5.0                      # steering controller look-ahead distance (m)

init_loc = chrono.ChVector3d(-50.0, 0.0, 0.5)   # spawn so the DLC fits the patch
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)     # identity: facing +X


# === Vehicle === FEDA wrapper owns its ChSystemSMC; build + initialize first
vehicle = veh.FEDA()
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
vehicle.SetEngineType(veh.EngineModelType_SIMPLE_MAP)
vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
vehicle.SetTireType(veh.TireModelType_PAC02)
vehicle.SetTireStepSize(tire_step)
vehicle.Initialize()

vehicle.SetChassisVisualizationType(chrono.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(chrono.VisualizationType_MESH)
vehicle.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === System & bodies (created by the veh.FEDA wrapper) ===
system = vehicle.GetSystem()              # ChSystemSMC owned by the wrapper
chassis = vehicle.GetChassisBody()        # cache: main chassis rigid body, reused every step
# wheels/spindles: vehicle.GetVehicle().GetSpindlePos(axle, side); joints:
# suspension + steering links created inside the wrapper; terrain patch added below.

# Collision is REQUIRED for vehicle/terrain contact — set Bullet on the
# wrapper-owned system right after Initialize (the framework contract).
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Terrain === flat rigid patch large enough to contain the lane change
patch_mat = chrono.ChContactMaterialSMC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch_mat.SetYoungModulus(2e7)

terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrain_height), chrono.QUNIT),
    terrain_length,
    terrain_width,
)
patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Validate the spawn footprint: all four wheels must rest on the patch surface.
TIRE_RADIUS = 0.499                       # FEDA PAC02 tire radius (m), from wheel geometry
ZTOL = 0.10                               # allowed wheel-bottom clearance vs patch top
veh_obj = vehicle.GetVehicle()
spindle_world = []
for axle in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_world.append(veh_obj.GetSpindlePos(axle, side))
wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= terrain_height - ZTOL, (
    f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs patch top z={terrain_height:.3f}; raise init_loc.z"
)

# === Driver === autonomous path-follower on an ISO double-lane-change path
# DoubleLaneChangePath(start, length, width, offset, total_length, to_left):
# ISO 3888 double lane change starting at the vehicle spawn, kept inside the patch.
dlc_path = veh.DoubleLaneChangePath(
    init_loc,        # path start at the vehicle spawn
    13.5,            # entry-lane length (m)
    4.0,             # lane offset width (m)
    11.0,            # exit-lane offset (m)
    50.0,            # total maneuver length (m)
    True,            # maneuver to the left
)
driver = veh.ChPathFollowerDriver(
    veh_obj,
    dlc_path,
    "double_lane_change",
    target_speed,
)
driver.GetSteeringController().SetLookAheadDistance(look_ahead)
driver.GetSteeringController().SetGains(0.8, 0.0, 0.0)   # KP, KI, KD
driver.GetSpeedController().SetGains(0.4, 0.0, 0.0)      # KP, KI, KD
driver.Initialize()

steer_ctrl = driver.GetSteeringController()   # cache: controller fetched once, reused every step

# === Visualization === vehicle-aware Irrlicht: window + chase cam + sky + lights
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("FEDA Double Lane Change")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(veh_obj)
vis.AttachDriver(driver)

# === Main loop === render-cadence outer loop; Synchronize/Advance the full stack


frame = 0
try:
    while vis.Run() and system.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sim_time = system.GetChTime()
            driver_inputs = driver.GetInputs()
            driver.Synchronize(sim_time)
            terrain.Synchronize(sim_time)
            vehicle.Synchronize(sim_time, driver_inputs, terrain)
            vis.Synchronize(sim_time, driver_inputs)

            driver.Advance(time_step)
            terrain.Advance(time_step)
            vehicle.Advance(time_step)          # advances the wrapper-owned system
            vis.Advance(time_step)
            if system.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad simulation state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass

# === Post-processing === assemble review media (stripped from the scored core)
