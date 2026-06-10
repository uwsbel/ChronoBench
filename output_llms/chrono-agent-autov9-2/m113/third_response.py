"""M113 tracked vehicle mobility test on rigid terrain with a box obstacle.

Models the M113 armored personnel carrier (a tracked vehicle) driving forward
under constant throttle across a flat rigid-terrain patch toward a long, fixed
box obstacle placed in its path. The goal is to exercise the vehicle's mobility:
the tracks should propel the chassis forward from its spawn point and the vehicle
should encounter / interact with the obstacle.

System type: NSC (non-smooth contact), Bullet collision. The M113 wrapper owns
its ChSystem; terrain, the obstacle box, and visualization attach to that system.
Drivetrain: SHAFTS engine + automatic-shafts transmission + BDS track driveline,
single-pin track shoes. Driver: scripted constant throttle = 0.8, no steering.

Expected behavior: the M113 accelerates forward (toward +X) along the terrain,
its road wheels / sprockets / tracks visibly turning, and drives up to the fixed
box obstacle lying across its path.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh

# === Named constants: timing, spawn, geometry, drivetrain ===
time_step = 2e-3                       # integration step (s) — stable for tracked NSC
sim_end = 10.0                         # simulated duration (s)
render_fps = 50.0                      # review-video frame rate
THROTTLE = 0.8                         # hard-coded constant throttle during the run
STEERING = 0.0                         # straight-line mobility test, no steering

# Vehicle spawn (chassis-frame origin in world coords).
VEH_INIT_X = -5.0
VEH_INIT_Y = 0.0
VEH_INIT_Z = 0.5                       # spawn height above the rigid terrain plane

# Rigid terrain patch dimensions.
TERRAIN_LENGTH = 100.0                 # X extent (m)
TERRAIN_WIDTH = 40.0                   # Y extent (m)
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01

# Long box obstacle placed across the vehicle's forward path.
BOX_LEN_X = 1.0                        # thin in travel direction
BOX_LEN_Y = 8.0                        # long across the path
BOX_LEN_Z = 0.5                        # obstacle height
BOX_POS_X = 8.0                        # ahead of the vehicle (in +X travel direction)
BOX_POS_Y = 0.0
BOX_POS_Z = BOX_LEN_Z / 2.0            # rest the box on the terrain plane (z=0)
BOX_DENSITY = 1000.0
BOX_FRICTION = 0.8

# Derived render cadence (precomputed once — never recomputed in the loop).
render_every = max(1, round(1.0 / (render_fps * time_step)))   # precomputed once


# === Vehicle (M113 tracked APC; the wrapper creates and owns its ChSystem) ===
# The M113 wrapper internally builds: the ChSystem, the chassis rigid body, the
# left/right track assemblies (sprockets, idlers, road wheels, track shoes),
# the SHAFTS engine + transmission and the BDS track driveline, plus all the
# suspension/track joints. We configure it, Initialize, then take the system.
vehicle = veh.M113()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)        # NSC contact for tracked + terrain
vehicle.SetTrackShoeType(veh.TrackShoeType_SINGLE_PIN)      # single-pin track shoes
vehicle.SetDrivelineType(veh.DrivelineTypeTV_BDS)           # BDS track driveline
vehicle.SetEngineType(veh.EngineModelType_SHAFTS)           # SHAFTS engine
vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
vehicle.SetBrakeType(veh.BrakeType_SIMPLE)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(
    chrono.ChCoordsysd(chrono.ChVector3d(VEH_INIT_X, VEH_INIT_Y, VEH_INIT_Z), chrono.QUNIT)
)
vehicle.Initialize()

vehicle.SetChassisVisualizationType(chrono.VisualizationType_PRIMITIVES)
vehicle.SetSprocketVisualizationType(chrono.VisualizationType_PRIMITIVES)
vehicle.SetIdlerVisualizationType(chrono.VisualizationType_PRIMITIVES)
vehicle.SetRoadWheelVisualizationType(chrono.VisualizationType_PRIMITIVES)
vehicle.SetTrackShoeVisualizationType(chrono.VisualizationType_PRIMITIVES)
vehicle.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)

# === System & enumerated handles (created by the veh.M113 wrapper) ===
system = vehicle.GetSystem()                       # ChSystem owned by the M113 wrapper
veh_obj = vehicle.GetVehicle()                     # cache: ChTrackedVehicle handle, reused every step
chassis = vehicle.GetChassisBody()                 # cache: chassis rigid body, reused every step
num_shoes_L = veh_obj.GetNumTrackShoes(veh.LEFT)   # cache: left-track shoe count (sizes terrain forces)
num_shoes_R = veh_obj.GetNumTrackShoes(veh.RIGHT)  # cache: right-track shoe count
assert num_shoes_L > 0 and num_shoes_R > 0, "track assemblies produced no shoes"

# Bullet collision is REQUIRED for this contact scene (tracks + terrain + obstacle).
# For the wrapper-managed vehicle, set it on the owned system after Initialize.
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Terrain (flat rigid patch the tracks ride on) ===
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()          # NSC material to match the NSC system
patch_mat.SetFriction(TERRAIN_FRICTION)
patch_mat.SetRestitution(TERRAIN_RESTITUTION)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Obstacle (long fixed box across the path to test mobility) ===
# A heavy fixed box: ChBodyEasyBox takes FULL extents; SetFixed pins it in place.
box_mat = chrono.ChContactMaterialNSC()
box_mat.SetFriction(BOX_FRICTION)
box_mat.SetRestitution(0.0)
obstacle = chrono.ChBodyEasyBox(
    BOX_LEN_X, BOX_LEN_Y, BOX_LEN_Z, BOX_DENSITY, True, True, box_mat
)
obstacle.SetName("obstacle_box")
obstacle.SetPos(chrono.ChVector3d(BOX_POS_X, BOX_POS_Y, BOX_POS_Z))
obstacle.SetFixed(True)                             # fixed obstacle in the vehicle path
obstacle.EnableCollision(True)
system.AddBody(obstacle)
system.GetCollisionSystem().BindAll()              # rebuild collision models after adding the box

# === Driver (scripted constant throttle; no human-in-the-loop in batch) ===
driver_inputs = veh.DriverInputs()
driver_inputs.m_throttle = THROTTLE
driver_inputs.m_steering = STEERING
driver_inputs.m_braking = 0.0

# === Visualization === tracked-vehicle Irrlicht window + sky + chase cam + lights
vis = veh.ChTrackedVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("M113 Mobility Test")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 12.0, 1.0)   # follow point, distance, height
vis.Initialize()                                               # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()                                                # standard outdoor sky backdrop
vis.AddTypicalLights()                                         # standard lighting
vis.AttachVehicle(veh_obj)                                     # bind chassis/track visual assets

# === Main loop === render-cadence outer loop; advance the tracked subsystem stack
# Tracked Synchronize needs per-side terrain forces sized to the track-shoe counts.
shoe_forces_left = veh.TerrainForces(num_shoes_L)    # precomputed once: per-shoe force buffers
shoe_forces_right = veh.TerrainForces(num_shoes_R)

frame = 0
try:
    while vis.Run() and system.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sim_time = system.GetChTime()
            terrain.Synchronize(sim_time)
            vehicle.Synchronize(sim_time, driver_inputs, shoe_forces_left, shoe_forces_right)
            vis.Synchronize(sim_time, driver_inputs)

            terrain.Advance(time_step)
            vehicle.Advance(time_step)    # advances the wrapper-owned system (no DoStepDynamics)
            vis.Advance(time_step)
            if system.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:    # solver divergence / invalid state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass

# === Post-processing === assemble review videos + plot, then clean frames
