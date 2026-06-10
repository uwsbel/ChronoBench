"""UAZBUS double-lane-change maneuver on flat rigid terrain (PyChrono 9.0.x).

Models a UAZBUS wheeled vehicle (NSC contact, wrapper-owned ChSystem) spawned
at world X = -40 m on a long flat RigidTerrain patch textured with concrete.
A scripted open-loop driver performs an ISO-style double-lane-change weave
(left swerve, return, right swerve, return) followed by a braking phase, so the
bus tracks two lane offsets and then decelerates to a stop. The wrapped vehicle
owns its system; terrain, visualization, and the scripted driver are attached to
that same system. Expected behavior: the bus accelerates forward along +X,
weaves through the lane-change steering profile staying on the terrain patch,
then brakes near the end of the run.

System type: NSC (default UAZBUS contact method). Main bodies: UAZBUS chassis +
four spindle/wheel assemblies (created by the veh.UAZBUS wrapper) and a flat
RigidTerrain patch body. Collision: Bullet (set on the wrapper-owned system).
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Named constants === geometry / timing / driver schedule (no bare literals downstream)
TIME_STEP = 2e-3                       # physics step (s)
TIRE_STEP = 1e-3                       # tire integration step (s)
SIM_END = 16.0                         # total simulated time (s)
RENDER_FPS = 50.0                      # review-video frame cadence
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once

VEH_INIT_X = -40.0                     # spawn X (world) per maneuver layout
VEH_INIT_Y = 0.0
VEH_INIT_Z = 0.5                       # chassis-origin spawn height above flat ground
INIT_POS = chrono.ChVector3d(VEH_INIT_X, VEH_INIT_Y, VEH_INIT_Z)
INIT_ROT = chrono.ChQuaterniond(1, 0, 0, 0)   # facing +X

TERRAIN_LENGTH = 250.0                 # long patch so the weave stays on-terrain
TERRAIN_WIDTH = 40.0
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01
TERRAIN_TOP_Z = 0.0                    # flat patch top plane
TIRE_RADIUS = 0.39                     # UAZBUS tire radius (for the footprint assert)
ZTOL = 0.15                            # allowed wheel-bottom clearance vs support top

CHASE_DIST = 8.0                       # chase camera distance behind the bus
CHASE_HEIGHT = 1.5                     # chase camera height offset

# Double-lane-change schedule: (time_s, steering[-1..1], throttle[0..1], braking[0..1], gear)
# Phase 1 build speed straight; phase 2 swerve left out then back to the home
# lane (balanced steer/counter-steer pair); phase 3 swerve right out then back;
# phase 4 brake to a stop. Each maneuver is a matched steer + equal-opposite
# counter-steer + centering pulse so the heading (and lateral offset) returns to
# near zero — a clean ISO-style double lane change that stays on the patch.
LANE_CHANGE_SCHEDULE = [
    (0.0,   0.00, 0.0, 0.0, 0.0),  # standstill
    (1.0,   0.00, 0.7, 0.0, 0.0),  # accelerate straight
    (4.0,   0.00, 0.6, 0.0, 0.0),  # cruise straight, steady speed
    # --- lane change 1: out to the left, then back to the home lane ---
    (4.5,   0.18, 0.5, 0.0, 0.0),  # steer left (leave home lane)
    (5.1,  -0.18, 0.5, 0.0, 0.0),  # counter-steer (straighten in left lane)
    (5.7,   0.00, 0.5, 0.0, 0.0),  # hold left lane
    (6.5,  -0.18, 0.5, 0.0, 0.0),  # steer right (return toward home lane)
    (7.1,   0.18, 0.5, 0.0, 0.0),  # counter-steer (straighten, back home)
    (7.7,   0.00, 0.5, 0.0, 0.0),  # hold home lane
    # --- lane change 2: out to the right, then back to the home lane ---
    (8.5,  -0.18, 0.5, 0.0, 0.0),  # steer right (leave home lane)
    (9.1,   0.18, 0.5, 0.0, 0.0),  # counter-steer (straighten in right lane)
    (9.7,   0.00, 0.5, 0.0, 0.0),  # hold right lane
    (10.5,  0.18, 0.5, 0.0, 0.0),  # steer left (return toward home lane)
    (11.1, -0.18, 0.5, 0.0, 0.0),  # counter-steer (straighten, back home)
    (11.7,  0.00, 0.5, 0.0, 0.0),  # hold home lane
    # --- braking phase ---
    (12.5,  0.00, 0.0, 0.0, 0.0),  # release throttle
    (13.0,  0.00, 0.0, 0.8, 0.0),  # brake hard
    (SIM_END, 0.00, 0.0, 0.8, 0.0),  # remain braking to the end
]


# === Vehicle === UAZBUS wrapper owns its ChSystem; configure then Initialize.
vehicle = veh.UAZBUS()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(INIT_POS, INIT_ROT))
vehicle.SetTireType(veh.TireModelType_TMEASY)   # slip/grip tire for rigid road traction
vehicle.SetTireStepSize(TIRE_STEP)
vehicle.Initialize()

vehicle.SetChassisVisualizationType(chrono.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(chrono.VisualizationType_MESH)
vehicle.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === System & bodies (created by the veh.UAZBUS wrapper) ===
sys = vehicle.GetSystem()              # cache: ChSystemNSC owned by the wrapper, reused below
chassis = vehicle.GetChassisBody()     # cache: main chassis rigid body, reused every step
veh_obj = vehicle.GetVehicle()         # cache: ChWheeledVehicle handle, reused for spindles
# spindles/wheels: veh_obj.GetSpindlePos(axle, side); joints: suspension + steering links
# created inside the wrapper; terrain patch body added below.

# === Collision system === Bullet, required for the vehicle/terrain contact in this scene.
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Terrain === long flat rigid patch with a concrete texture under the bus.
terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(TERRAIN_FRICTION)
patch_mat.SetRestitution(TERRAIN_RESTITUTION)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/concrete.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))
terrain.Initialize()

# === Footprint assert === confirm the wheels rest on (not through) the flat patch.
spindle_world = [veh_obj.GetSpindlePos(axle, side)
                 for axle in range(veh_obj.GetNumberAxles())
                 for side in (veh.LEFT, veh.RIGHT)]
wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS   # precomputed once
assert wheel_bottom_z >= TERRAIN_TOP_Z - ZTOL, (
    f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs terrain top z={TERRAIN_TOP_Z:.3f}; raise VEH_INIT_Z"
)

# === Driver === scripted open-loop double-lane-change + braking schedule.
driver_data = veh.vector_Entry(
    [veh.DataDriverEntry(t, s, th, br, g) for (t, s, th, br, g) in LANE_CHANGE_SCHEDULE]
)
driver = veh.ChDataDriver(veh_obj, driver_data)
driver.Initialize()

# === Visualization === vehicle-aware Irrlicht window: chase cam + sky + lights + logo.
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("UAZBUS Double Lane Change")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, CHASE_HEIGHT), CHASE_DIST, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVector3d(VEH_INIT_X - 8.0, -8.0, 4.0), INIT_POS)
vis.AddGrid(2.0, 2.0, 60, 20,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.01), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))   # ground reference grid
vis.AttachVehicle(veh_obj)
vis.AttachDriver(driver)


# === Main loop === throttled render outer loop; vehicle subsystem stack advanced inline.
frame = 0
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            sim_time = sys.GetChTime()
            driver_inputs = driver.GetInputs()
            driver.Synchronize(sim_time)
            terrain.Synchronize(sim_time)
            vehicle.Synchronize(sim_time, driver_inputs, terrain)
            vis.Synchronize(sim_time, driver_inputs)
            driver.Advance(TIME_STEP)
            terrain.Advance(TIME_STEP)
            vehicle.Advance(TIME_STEP)     # advances the wrapper-owned ChSystem
            vis.Advance(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    # === Cleanup === release the visual device even if a step diverges mid-run.
    vis.GetDevice().closeDevice()
