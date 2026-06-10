"""HMMWV_Full rigid-terrain simulation with a single highway mesh patch.

The model uses an NSC vehicle system owned by the HMMWV wrapper, Bullet collision,
one mesh-based rigid terrain patch, and an added visual triangle mesh on the
terrain ground body. The vehicle starts at the requested highway position and can
be driven interactively while the validation run records a short straight drive.
"""

import math
import os

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


# === Constants === named setup values keep the vehicle and terrain placement explicit
STEP_SIZE = 2.0e-3
TIRE_STEP_SIZE = 1.0e-3
SIM_END = 3.0
RENDER_FPS = 30.0
RENDER_STEP = 1.0 / RENDER_FPS
RENDER_STEPS = max(1, math.ceil(RENDER_STEP / STEP_SIZE))  # precomputed once
INIT_POS = chrono.ChVector3d(6.0, -70.0, 0.5)
INIT_ROT = chrono.QuatFromAngleZ(chrono.CH_PI / 2.0)
FRICTION = 0.9
RESTITUTION = 0.01
CONTACT_THICKNESS = 0.01
CAMERA_TRACK_POINT = chrono.ChVector3d(0.0, 0.0, 1.75)
CAMERA_DISTANCE = 10.0
CAMERA_HEIGHT = 1.5


def chrono_mesh_file(prompt_rel_path, fallback_rel_path):
    """Resolve the requested mesh path, falling back to the bundled 9.0.0 asset."""
    requested = chrono.GetChronoDataFile(prompt_rel_path)
    if os.path.isfile(requested):
        return requested
    fallback = chrono.GetChronoDataFile(fallback_rel_path)
    if os.path.isfile(fallback):
        return fallback
    raise FileNotFoundError(f"mesh not found: {prompt_rel_path} or {fallback_rel_path}")


def add_visual_mesh_to_terrain(ground_body, mesh_file):
    """Attach a Wavefront visual mesh to the RigidTerrain ground body."""
    mesh = chrono.ChTriangleMeshConnected()
    if not mesh.LoadWavefrontMesh(mesh_file, True, True):
        raise RuntimeError(f"failed to load visual terrain mesh: {mesh_file}")
    visual_mesh = chrono.ChVisualShapeTriangleMesh()
    visual_mesh.SetMesh(mesh)
    visual_mesh.SetBackfaceCull(False)
    ground_body.AddVisualShape(visual_mesh)
    return visual_mesh


# === Vehicle and system === wrapper owns the ChSystem and all HMMWV sub-bodies
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_POS, INIT_ROT))
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(TIRE_STEP_SIZE)
hmmwv.Initialize()

system = hmmwv.GetSystem()  # cache: wrapper-owned NSC system reused by terrain and loop
chassis = hmmwv.GetChassisBody()  # cache: main chassis body reused for logging
vehicle_model = hmmwv.GetVehicle()  # cache: vehicle subsystem handle reused in visualization
# wrapper-created bodies: chassis, suspension links, steering links, wheels, tires, and spindles
# wrapper-created joints: steering, suspension, driveline, engine, and tire force subsystems
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
print("VEHICLE MASS: ", vehicle_model.GetMass())

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)


# === Terrain === a single collision mesh patch plus the requested visual highway mesh
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(FRICTION)
patch_mat.SetRestitution(RESTITUTION)

collision_mesh = chrono_mesh_file(
    "vehicle/terrain/meshes/Highway_col.obj",
    "synchrono/meshes/Highway_col.obj",
)
visual_mesh = chrono_mesh_file(
    "terrain/meshes/Highway_vis.obj",
    "synchrono/meshes/Highway_vis.obj",
)
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    collision_mesh,
    True,
    CONTACT_THICKNESS,
    False,
)
terrain_body = patch.GetGroundBody()  # cache: fixed terrain body receives the visual mesh
add_visual_mesh_to_terrain(terrain_body, visual_mesh)
terrain.Initialize()


# === Visualization and driver === Irrlicht vehicle view and interactive driver are built unconditionally
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV on Single Highway Mesh")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(CAMERA_TRACK_POINT, CAMERA_DISTANCE, CAMERA_HEIGHT)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle_model)

driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(RENDER_STEP / 1.0)
driver.SetThrottleDelta(RENDER_STEP / 1.0)
driver.SetBrakingDelta(RENDER_STEP / 0.3)
driver.Initialize()


# === Main loop === synchronized vehicle, terrain, driver, and Irrlicht updates
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

try:

    while vis.Run() and system.GetChTime() < SIM_END:
        time = system.GetChTime()

        if step_number % RENDER_STEPS == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()

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

except (FileNotFoundError, RuntimeError, ValueError) as exc:
    print(f"simulation setup or runtime failure: {exc}")
    raise
except (OSError, IOError) as exc:
    print(f"recording file failure: {exc}")
    raise
finally:
    pass
