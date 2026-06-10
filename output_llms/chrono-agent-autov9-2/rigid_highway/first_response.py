"""HMMWV full vehicle driving straight down a custom highway mesh terrain.

Model
-----
A full High-Mobility Multipurpose Wheeled Vehicle (`veh.HMMWV_Full`) initialized
with an SMC (penalty) contact method and TMEASY tires, driven by a scripted
time-based driver that holds the steering centered and ramps the throttle so the
vehicle accelerates in a straight line. The road is a rigid terrain patch built
from the shipped highway meshes: `synchrono/meshes/Highway_col.obj` provides the
collision surface and `Highway_vis.obj` is added as the visual skin. The mesh is
a ~150 m strip whose LENGTH lies along its local Y and whose ~23 m WIDTH lies
along local X, so the patch is rotated -90 deg about Z to lay the road length
along world +X; the vehicle is spawned facing +X (down the road's length, not
across the narrow width into the barriers). If the highway mesh is unavailable a
large flat rigid patch is used as a fallback (noted at the patch-build site).

System
------
SMC system owned internally by the `HMMWV_Full` wrapper. After the wrapper
initializes, the Bullet collision system is selected explicitly on the owned
system. Rendering is Irrlicht via `veh.ChWheeledVehicleVisualSystemIrrlicht`
with a chase camera; the loop advances driver/terrain/vehicle/vis in real time
at 50 rendered frames per second.

Expected behavior
------------------
The HMMWV rolls forward in a straight line along +X down the highway, staying
upright and on the road for the full simulation duration.
"""

import os
import math

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Constants === geometry / physics / timing (no bare literals downstream)
TIME_STEP = 1.0e-3                 # integration step (s)
SIM_END = 12.0                     # simulation duration (s)
RENDER_FPS = 50.0                  # rendered frames per second (prompt: 50 fps)
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once

# Highway mesh: length along local Y (~150 m), width along local X (~23 m).
# Rotate -90 deg about Z so the road LENGTH lies along world +X.
ROAD_YAW = -math.pi / 2.0          # patch orientation about Z
COL_MESH = "synchrono/meshes/Highway_col.obj"   # collision surface
VIS_MESH = "synchrono/meshes/Highway_vis.obj"   # visual skin
FLAT_LEN = 300.0                   # fallback flat patch length (m, world X)
FLAT_WID = 40.0                    # fallback flat patch width (m, world Y)

# Spawn near the start of the road, facing down its length (+X), centered lane.
SUSPENSION_REF_HEIGHT = 0.5        # HMMWV chassis origin above wheel-bottom at rest
ROAD_TOP_Z = 0.0                   # highway driving surface height near the spawn
VEH_INIT_X = -60.0                 # start near one end of the ~150 m strip
VEH_INIT_Y = 0.0                   # centered in the lane
VEH_INIT_Z = ROAD_TOP_Z + SUSPENSION_REF_HEIGHT
INIT_LOC = chrono.ChVector3d(VEH_INIT_X, VEH_INIT_Y, VEH_INIT_Z)
INIT_ROT = chrono.QUNIT            # facing +X, the road's length direction

TIRE_RADIUS = 0.46                 # HMMWV tire radius (m), for footprint assert
ZTOL = 0.10                        # allowed wheel-bottom clearance/overlap vs road

# === Vehicle (HMMWV_Full wrapper owns its system + bodies + joints) ===
# WHAT: build the full HMMWV with TMEASY tires; WHY: drivable on a rigid road.
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
hmmwv.SetTireType(veh.TireModelType_TMEASY)   # prompt: TMEASY tire model
hmmwv.SetTireStepSize(TIME_STEP)
hmmwv.Initialize()

# Mesh visualization for all vehicle components (prompt: mesh visualization).
hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
sys = hmmwv.GetSystem()                  # ChSystemSMC owned by the wrapper
chassis = hmmwv.GetChassisBody()         # cache: main chassis rigid body, reused
veh_obj = hmmwv.GetVehicle()             # cache: underlying ChWheeledVehicle handle
# spindles: veh_obj.GetSpindlePos(axle, side); wheels: veh_obj.GetAxle(i)...
# joints: suspension + Pitman-arm steering links created inside the wrapper.

# Collision system: REQUIRED — the scene has vehicle/terrain contact.
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Terrain === rigid highway patch from the shipped collision + visual meshes
terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChContactMaterialSMC()   # SMC system -> SMC material
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch_mat.SetYoungModulus(2e7)
road_csys = chrono.ChCoordsysd(chrono.ChVector3d(0, 0, ROAD_TOP_Z),
                               chrono.QuatFromAngleZ(ROAD_YAW))
col_file = chrono.GetChronoDataFile(COL_MESH)
vis_file = chrono.GetChronoDataFile(VIS_MESH)
if os.path.isfile(col_file):
    # Highway collision mesh as the road surface (length -> world +X via ROAD_YAW).
    patch = terrain.AddPatch(patch_mat, road_csys, col_file)
    if os.path.isfile(vis_file):
        # Add Highway_vis.obj as the visual skin on the patch ground body.
        vis_mesh = chrono.ChTriangleMeshConnected()
        vis_mesh = vis_mesh.CreateFromWavefrontFile(vis_file, True, True)
        vshape = chrono.ChVisualShapeTriangleMesh()
        vshape.SetMesh(vis_mesh)
        vshape.SetName("highway_vis")
        vshape.SetMutable(False)
        patch.GetGroundBody().AddVisualShape(
            vshape, chrono.ChFramed(chrono.ChVector3d(0, 0, ROAD_TOP_Z),
                                    chrono.QuatFromAngleZ(ROAD_YAW)))
else:
    # Fallback: highway mesh missing -> large flat rigid patch so the run is valid.
    patch = terrain.AddPatch(patch_mat,
                             chrono.ChCoordsysd(chrono.ChVector3d(0, 0, ROAD_TOP_Z),
                                                chrono.QUNIT),
                             FLAT_LEN, FLAT_WID)
    patch.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
terrain.Initialize()

# Footprint check: wheels must rest on (not through) the road at spawn.
spindle_world = [veh_obj.GetSpindlePos(a, s)
                 for a in range(veh_obj.GetNumberAxles())
                 for s in (veh.LEFT, veh.RIGHT)]
wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
road_z = terrain.GetHeight(chrono.ChVector3d(VEH_INIT_X, VEH_INIT_Y, 0))
assert wheel_bottom_z >= road_z - ZTOL, (
    f"vehicle sinks into road: wheel bottom z={wheel_bottom_z:.3f} vs road top "
    f"z={road_z:.3f}; raise SUSPENSION_REF_HEIGHT by {road_z - wheel_bottom_z:.3f} m")


# === Driver === scripted time-based control: centered steering, ramped throttle
class StraightLineDriver(veh.ChDriver):
    """Holds steering centered (straight lane) and ramps throttle to cruise."""

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        # Ramp throttle smoothly over the first 2 s, then hold; no steering.
        self.SetThrottle(min(0.6, 0.3 * time))
        self.SetBraking(0.0)
        self.SetSteering(0.0)          # centered — drive straight down the lane


driver = StraightLineDriver(veh_obj)
driver.Initialize()

# === Visualization === vehicle-aware Irrlicht: window + chase cam + sky + lights
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV on Highway Mesh Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)   # follow the chassis
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVector3d(VEH_INIT_X - 8, -8, 4),
              chrono.ChVector3d(VEH_INIT_X, 0, 0.5))   # static establishing view
vis.AttachVehicle(veh_obj)
vis.AttachDriver(driver)


# === Main loop === real-time render-cadence loop; physics in inner batch
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
            hmmwv.Synchronize(sim_time, driver_inputs, terrain)
            vis.Synchronize(sim_time, driver_inputs)
            driver.Advance(TIME_STEP)
            terrain.Advance(TIME_STEP)
            hmmwv.Advance(TIME_STEP)        # advances the wrapper-owned system
            vis.Advance(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad simulation state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
