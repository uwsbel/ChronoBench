"""HMMWV driving on a single rigid mesh-patch terrain.

Model: a full HMMWV (veh.HMMWV_Full, TMEASY tires) spawned on a single
RigidTerrain patch whose collision surface is a Wavefront mesh (the shipped
Highway road mesh) with a thin contact-material sweep thickness. A separate
high-resolution visual mesh is attached to the terrain ground body via
ChVisualShapeTriangleMesh so the road renders richly while contact uses the
collision mesh. The vehicle is contact-driven on a Bullet collision system
(SMC contact method), so SetCollisionSystemType(Type_BULLET) is set on the
wrapper-owned system after Initialize.

System type: SMC (HMMWV wrapper owns a ChSystemSMC). Main bodies: HMMWV chassis
+ four wheels/spindles (created by the wrapper) and one terrain ground body
carrying the road collision + visual meshes. A scripted ChDriver applies a
brief settle, then steady throttle driving straight along the road.

Expected behavior: the HMMWV settles onto the road mesh at the spawn point,
then accelerates forward and stays on the drivable surface, upright, for the
whole run.

Note on assets: the requested road mesh files are resolved to the shipped
Chrono Highway meshes (data/synchrono/meshes/Highway_col.obj for collision and
Highway_vis.obj for the visual surface), since those are the drivable road
meshes present in this Chrono data tree.
"""

import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Constants === geometry / physics parameters and derived render cadence
TIME_STEP = 2e-3                     # integration step (s)
SIM_END = 12.0                       # total simulated time (s)
RENDER_FPS = 50.0                    # review-video frame rate
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

VEH_INIT_POS = chrono.ChVector3d(6, -70, 0.5)   # spawn on the road mesh
# The road strip runs along +Y, so face the vehicle +Y (yaw +90 deg about Z)
# to keep it on the drivable surface as it accelerates.
VEH_INIT_ROT = chrono.QuatFromAngleZ(math.pi / 2)
TERRAIN_FRICTION = 0.9               # road contact friction
TERRAIN_RESTITUTION = 0.01           # road contact restitution
PATCH_THICKNESS = 0.01               # mesh-patch contact sweep thickness (m)
TIRE_RADIUS = 0.46                   # HMMWV tire radius (m) for footprint check
ZTOL = 0.20                          # allowed wheel-bottom clearance vs road

COL_MESH = chrono.GetChronoDataFile("synchrono/meshes/Highway_col.obj")  # collision road
VIS_MESH = chrono.GetChronoDataFile("synchrono/meshes/Highway_vis.obj")  # visual road


# === Vehicle (HMMWV_Full wrapper owns its ChSystemSMC) ===
# The wrapper creates the system, chassis, four spindle bodies, suspension +
# steering joints, powertrain and tires internally; we enumerate the real
# handles after Initialize so the essentials are visible.
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(VEH_INIT_POS, VEH_INIT_ROT))
hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
hmmwv.SetTireType(veh.TireModelType_TMEASY)     # deformable tire force model
hmmwv.SetTireStepSize(TIME_STEP)
hmmwv.Initialize()

hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

system = hmmwv.GetSystem()                # ChSystemSMC owned by the wrapper
chassis = hmmwv.GetChassisBody()          # cache: main chassis body, reused every step
veh_obj = hmmwv.GetVehicle()              # cache: vehicle subsystem handle, reused below

# === Collision system === Bullet narrow-phase for vehicle/terrain contact
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Terrain === single rigid patch with a road COLLISION mesh + a VISUAL mesh
terrain = veh.RigidTerrain(system)

patch_mat = chrono.ChContactMaterialSMC()
patch_mat.SetFriction(TERRAIN_FRICTION)
patch_mat.SetRestitution(TERRAIN_RESTITUTION)
patch_mat.SetYoungModulus(2e7)

# Mesh patch: collision surface from the road collision OBJ, thin sweep thickness.
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    COL_MESH,
    True,               # connected_mesh
    PATCH_THICKNESS,    # sweep-sphere radius (contact thickness)
)
patch.SetColor(chrono.ChColor(0.6, 0.6, 0.6))

# Attach the high-resolution VISUAL road mesh to the terrain ground body so the
# rendered road is the detailed surface while contact uses the collision mesh.
vis_trimesh = chrono.ChTriangleMeshConnected()
vis_trimesh = vis_trimesh.CreateFromWavefrontFile(VIS_MESH, True, True)
vis_shape = chrono.ChVisualShapeTriangleMesh()
vis_shape.SetMesh(vis_trimesh)
vis_shape.SetName("highway_visual")
vis_shape.SetMutable(False)
ground = patch.GetGroundBody()            # terrain rigid ground body
ground.AddVisualShape(vis_shape, chrono.ChFramed(chrono.VNULL, chrono.QUNIT))

terrain.Initialize()

# === Footprint check === ensure wheels rest on (not through) the road mesh
spindle_world = [
    veh_obj.GetSpindlePos(axle, side)
    for axle in range(veh_obj.GetNumberAxles())
    for side in (veh.LEFT, veh.RIGHT)
]
road_z = terrain.GetHeight(VEH_INIT_POS)  # road surface height under spawn
wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= road_z - ZTOL, (
    f"vehicle sinks into road: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs road top z={road_z:.3f}; raise VEH_INIT_POS.z"
)

# === Driver === scripted time-based control (settle, accelerate, gentle steer)
class ScriptedDriver(veh.ChDriver):
    """Open-loop driver: brief brake to settle, then steady straight throttle."""

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        if time < 0.5:
            self.SetThrottle(0.0)
            self.SetBraking(1.0)
        else:
            self.SetThrottle(0.5)
            self.SetBraking(0.0)
        self.SetSteering(0.0)   # drive straight to stay on the road strip

driver = ScriptedDriver(veh_obj)
driver.Initialize()

# === Visualization === vehicle-aware Irrlicht window + chase cam + sky/lights
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV on rigid mesh terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(veh_obj)
vis.AttachDriver(driver)

# === Main loop === render-cadence outer loop; vehicle Synchronize/Advance inner
try:

    frame = 0
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
            hmmwv.Advance(TIME_STEP)          # internally steps the owned system
            vis.Advance(TIME_STEP)
            if system.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:     # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass

# === Post-processing === assemble review video + plot, then clean frame dirs
