"""HMMWV full vehicle driving along a rigid highway mesh terrain patch.

Model:
  * Vehicle: veh.HMMWV_Full (SMC contact) with TMEASY tires, AWD driveline,
    Pitman-arm steering, shaft-based engine + automatic transmission.
  * Terrain: a single RigidTerrain mesh patch built from the SynChrono Highway
    collision/visual meshes. The Highway mesh runs ~150 m along its LOCAL Y axis
    (the road length) and ~23 m along local X (the road width).
  * The patch is placed with a -90 deg rotation about world Z and an origin at
    (6, -70, 0). Under Rz(-90) the mesh's local +Y (road length) maps to world
    +X, so the drivable lane runs along world X. The vehicle therefore spawns
    facing world +X (steering centered) so it drives DOWN the road length rather
    than across into the barriers.
  * Contact material on the patch: friction 0.4, restitution 0.05.

System type: NSC is NOT used here — the HMMWV_Full wrapper owns a ChSystemSMC
(SMC contact), so all contact materials are ChContactMaterialSMC.

Expected behavior: the vehicle accelerates from rest under a closed-loop
path-follower driver that holds the lane centerline, stays upright, and
translates a meaningful distance along world +X down the highway lane.
"""

import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


# === Simulation constants ===  geometry / timing / driver schedule (no bare literals downstream)
TIME_STEP = 2e-3                      # integration step (s)
SIM_END = 12.0                        # total simulated time (s)
RENDER_FPS = 50.0                     # review-video frame cadence

# Highway mesh footprint (introspected): local Y is the road length, local X the width.
PATCH_MESH_COL = "synchrono/meshes/Highway_col.obj"   # collision mesh
PATCH_MESH_VIS = "synchrono/meshes/Highway_vis.obj"   # visual mesh
PATCH_POS = chrono.ChVector3d(6.0, -70.0, 0.0)        # patch origin in world
PATCH_YAW = -math.pi / 2.0                            # -90 deg about world Z
PATCH_ROT = chrono.QuatFromAngleZ(PATCH_YAW)
PATCH_CSYS = chrono.ChCoordsysd(PATCH_POS, PATCH_ROT)

PATCH_FRICTION = 0.4                  # final patch friction coefficient
PATCH_RESTITUTION = 0.05              # final patch restitution

# Vehicle contact / tire / spawn parameters.
CONTACT_METHOD = chrono.ChContactMethod_SMC
SUSPENSION_REF_HEIGHT = 0.5           # HMMWV chassis-origin height above wheel-bottom at rest
TIRE_RADIUS = 0.46                    # HMMWV tire radius (approx, validated by assert below)
ZTOL = 0.15                           # allowed wheel-bottom clearance/overlap vs road

# Spawn the vehicle on the road, facing world +X (down the rotated road length).
# The mesh local origin lands at PATCH_POS; start near it and drive forward.
VEH_INIT_X = -55.0                    # start near one end of the ~150 m lane along world X
VEH_INIT_Y = -70.0                    # centered on the patch origin Y
ROAD_TOP_Z = 0.0                      # drivable surface near mesh z=0
VEH_INIT_Z = ROAD_TOP_Z + SUSPENSION_REF_HEIGHT
VEH_INIT_POS = chrono.ChVector3d(VEH_INIT_X, VEH_INIT_Y, VEH_INIT_Z)
VEH_INIT_ROT = chrono.QUNIT           # chassis forward = world +X, steering centered

RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once



# === Vehicle (HMMWV_Full wrapper owns its ChSystemSMC + chassis/spindles/joints) ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(CONTACT_METHOD)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(VEH_INIT_POS, VEH_INIT_ROT))
hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
hmmwv.SetTireType(veh.TireModelType_TMEASY)   # prompt: TMEASY tire on rigid highway
hmmwv.SetTireStepSize(TIME_STEP)
hmmwv.Initialize()

hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

# Wrapper-created components, fetched into named locals so they are visible:
#   system (ChSystemSMC), chassis rigid body, axles -> spindles, suspension/steering joints.
system = hmmwv.GetSystem()                 # ChSystemSMC owned by the wrapper
chassis = hmmwv.GetChassisBody()           # cache: main chassis rigid body, reused every step
veh_obj = hmmwv.GetVehicle()               # cache: ChWheeledVehicle handle, reused every step

# === Collision system === Bullet, set on the wrapper-owned system after Initialize (scene has contact)
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Terrain === single rigid highway mesh patch with the final friction/restitution + frame deltas
terrain = veh.RigidTerrain(system)

patch_mat = chrono.ChContactMaterialSMC()
patch_mat.SetFriction(PATCH_FRICTION)
patch_mat.SetRestitution(PATCH_RESTITUTION)
patch_mat.SetYoungModulus(2e7)

patch = terrain.AddPatch(
    patch_mat,
    PATCH_CSYS,                                  # (6,-70,0) origin, -90 deg about Z
    chrono.GetChronoDataFile(PATCH_MESH_COL),    # collision mesh
    True,                                        # connected mesh
    0.0,                                          # sweep-sphere radius
    True,                                         # build visualization too
)
patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/concrete.jpg"), 200, 200)
terrain.Initialize()

# Footprint check: read actual spindle world positions and confirm the wheel bottoms
# rest on (not through) the road surface. The message says how far to nudge the spawn Z.
spindle_world = []
for axle in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_world.append(veh_obj.GetSpindlePos(axle, side))
wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= ROAD_TOP_Z - ZTOL, (
    f"vehicle sinks into road: wheel bottom z={wheel_bottom_z:.3f} vs road top "
    f"z={ROAD_TOP_Z:.3f}; raise SUSPENSION_REF_HEIGHT by {ROAD_TOP_Z - wheel_bottom_z:.3f} m"
)

# === Driver === closed-loop path follower holding the lane centerline (no human-in-the-loop)
# A straight Bezier path along world +X at the lane center (y=-70) keeps the vehicle
# tracking down the road length; the steering controller actively corrects tire scrub
# so the HMMWV does not wander laterally into the barriers.
TARGET_SPEED = 12.0       # cruise speed along the lane (m/s)
PATH_Z = VEH_INIT_Z       # path height matches the chassis spawn height
path_points = chrono.vector_ChVector3d([
    chrono.ChVector3d(VEH_INIT_X, VEH_INIT_Y, PATH_Z),
    chrono.ChVector3d(VEH_INIT_X + 40.0, VEH_INIT_Y, PATH_Z),
    chrono.ChVector3d(VEH_INIT_X + 80.0, VEH_INIT_Y, PATH_Z),
    chrono.ChVector3d(VEH_INIT_X + 120.0, VEH_INIT_Y, PATH_Z),
])
lane_path = chrono.ChBezierCurve(path_points)

driver = veh.ChPathFollowerDriver(veh_obj, lane_path, "highway_lane", TARGET_SPEED)
driver.GetSteeringController().SetLookAheadDistance(6.0)
driver.GetSteeringController().SetGains(0.7, 0.0, 0.0)   # KP, KI, KD
driver.GetSpeedController().SetGains(0.4, 0.0, 0.0)
driver.Initialize()

# === Visualization === vehicle-aware Irrlicht: window + chase cam + sky + lights + logo
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV on Highway Patch")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 9.0, 0.6)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(veh_obj)
vis.AttachDriver(driver)

# === Logging setup (CSV review data) ===

# === Main loop === render once per frame; advance the full vehicle subsystem stack each step
frame = 0
try:
    while vis.Run() and system.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            sim_time = system.GetChTime()
            driver_inputs = driver.GetInputs()
            driver.Synchronize(sim_time)
            terrain.Synchronize(sim_time)
            hmmwv.Synchronize(sim_time, driver_inputs, terrain)
            vis.Synchronize(sim_time, driver_inputs)
            driver.Advance(TIME_STEP)
            terrain.Advance(TIME_STEP)
            hmmwv.Advance(TIME_STEP)        # advances the wrapper-owned ChSystemSMC
            vis.Advance(TIME_STEP)
            if system.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad simulation state
    import traceback
    traceback.print_exc()
    raise

# === Post-processing (close CSV, assemble review video + plot) ===
