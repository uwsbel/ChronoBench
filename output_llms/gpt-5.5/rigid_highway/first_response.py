# hmmwv_highway_mesh.py
#
# PyChrono simulation: full HMMWV on custom mesh terrain with Irrlicht visualization.
#
# Required mesh files:
#   Highway_col.obj   collision mesh
#   Highway_vis.obj   visual mesh
#
# Put these OBJ files in the same directory as this script, or update COL_MESH_FILE
# and VIS_MESH_FILE below.

import os
import math

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


# ---------------------------------------------------------------------
# Chrono / vehicle data paths
# ---------------------------------------------------------------------

# If your Chrono data path is not configured globally, set it here.
# Example:
# chrono.SetChronoDataPath("/path/to/chrono/data/")
#
# The vehicle module expects the vehicle data directory.
if "CHRONO_DATA_DIR" in os.environ:
    chrono.SetChronoDataPath(os.environ["CHRONO_DATA_DIR"])

veh.SetDataPath(os.path.join(chrono.GetChronoDataPath(), "vehicle") + "/")


# ---------------------------------------------------------------------
# User parameters
# ---------------------------------------------------------------------

CONTACT_METHOD = chrono.ChContactMethod_NSC

# Initial vehicle state
INIT_LOC = chrono.ChVector3d(0.0, 0.0, 0.75)
INIT_ROT = chrono.QUNIT

# Mesh terrain files
SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
COL_MESH_FILE = os.path.join(SCRIPT_DIR, "Highway_col.obj")
VIS_MESH_FILE = os.path.join(SCRIPT_DIR, "Highway_vis.obj")

# Simulation parameters
STEP_SIZE = 1.0e-3
TIRE_STEP_SIZE = 1.0e-3
FPS = 50
RENDER_STEP_SIZE = 1.0 / FPS

# Driver response times
STEERING_TIME = 1.0
THROTTLE_TIME = 1.0
BRAKING_TIME = 0.3

# Terrain contact properties
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01
TERRAIN_YOUNG_MODULUS = 2.0e7
TERRAIN_POISSON_RATIO = 0.3
TERRAIN_COLLISION_ENVELOPE = 0.005


# ---------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------

def make_contact_material(contact_method):
    """Create a Chrono contact material compatible with the selected contact method."""
    if contact_method == chrono.ChContactMethod_NSC:
        mat = chrono.ChContactMaterialNSC()
        mat.SetFriction(TERRAIN_FRICTION)
        mat.SetRestitution(TERRAIN_RESTITUTION)
    else:
        mat = chrono.ChContactMaterialSMC()
        mat.SetFriction(TERRAIN_FRICTION)
        mat.SetRestitution(TERRAIN_RESTITUTION)
        mat.SetYoungModulus(TERRAIN_YOUNG_MODULUS)
        mat.SetPoissonRatio(TERRAIN_POISSON_RATIO)

    return mat


def create_mesh_terrain(system, contact_method, col_mesh_file, vis_mesh_file):
    """
    Create a RigidTerrain patch using Highway_col.obj for tire/terrain contact
    and attach Highway_vis.obj as the visible mesh.
    """

    if not os.path.isfile(col_mesh_file):
        raise FileNotFoundError(f"Collision mesh not found: {col_mesh_file}")

    if not os.path.isfile(vis_mesh_file):
        raise FileNotFoundError(f"Visual mesh not found: {vis_mesh_file}")

    terrain = veh.RigidTerrain(system)

    terrain_mat = make_contact_material(contact_method)
    terrain_pos = chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT)

    # Add collision mesh patch.  Visualization is disabled here because we attach
    # a separate visual mesh below.
    try:
        patch = terrain.AddPatch(
            terrain_mat,
            terrain_pos,
            col_mesh_file,
            "Highway_collision_mesh",
            TERRAIN_COLLISION_ENVELOPE,
            False,
            0.0,
        )
    except TypeError:
        # Fallback for older PyChrono bindings with fewer AddPatch arguments.
        patch = terrain.AddPatch(
            terrain_mat,
            terrain_pos,
            col_mesh_file,
            "Highway_collision_mesh",
            TERRAIN_COLLISION_ENVELOPE,
            False,
        )

    terrain.Initialize()

    # Add separate visual mesh to the terrain body.
    vis_mesh = chrono.ChTriangleMeshConnected()
    vis_mesh.LoadWavefrontMesh(vis_mesh_file, True, True)

    vis_shape = chrono.ChVisualShapeTriangleMesh()
    vis_shape.SetMesh(vis_mesh)
    vis_shape.SetName("Highway_visual_mesh")
    vis_shape.SetMutable(False)

    try:
        terrain_body = patch.GetGroundBody()
    except AttributeError:
        terrain_body = patch.GetBody()

    terrain_body.AddVisualShape(vis_shape)

    return terrain


# ---------------------------------------------------------------------
# Vehicle setup
# ---------------------------------------------------------------------

hmmwv = veh.HMMWV_Full()

hmmwv.SetContactMethod(CONTACT_METHOD)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))

# Full HMMWV configuration
hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)

# Tire model requested: TMEASY
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(TIRE_STEP_SIZE)

# Optional chassis collision setting
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)

hmmwv.Initialize()

# Mesh visualization for all major vehicle subsystems
hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

system = hmmwv.GetSystem()

try:
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
except AttributeError:
    system.Set_G_acc(chrono.ChVector3d(0, 0, -9.81))

# Use Bullet collision system when available.
try:
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
except Exception:
    pass


# ---------------------------------------------------------------------
# Terrain setup
# ---------------------------------------------------------------------

terrain = create_mesh_terrain(system, CONTACT_METHOD, COL_MESH_FILE, VIS_MESH_FILE)


# ---------------------------------------------------------------------
# Irrlicht visualization
# ---------------------------------------------------------------------

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("PyChrono HMMWV on Custom Highway Mesh Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 8.0, 0.5)
vis.Initialize()

try:
    vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
except Exception:
    pass

vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(hmmwv.GetVehicle())


# ---------------------------------------------------------------------
# Interactive driver
# ---------------------------------------------------------------------

driver = veh.ChInteractiveDriverIRR(vis)

driver.SetSteeringDelta(RENDER_STEP_SIZE / STEERING_TIME)
driver.SetThrottleDelta(RENDER_STEP_SIZE / THROTTLE_TIME)
driver.SetBrakingDelta(RENDER_STEP_SIZE / BRAKING_TIME)

driver.Initialize()


# ---------------------------------------------------------------------
# Simulation loop
# ---------------------------------------------------------------------

realtime_timer = chrono.ChRealtimeStepTimer()

render_steps = max(1, int(math.ceil(RENDER_STEP_SIZE / STEP_SIZE)))
step_number = 0

while vis.Run():
    time = system.GetChTime()

    # Render at 50 FPS
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    # Driver inputs
    driver_inputs = driver.GetInputs()

    # Synchronize modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    hmmwv.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance modules
    driver.Advance(STEP_SIZE)
    terrain.Advance(STEP_SIZE)
    hmmwv.Advance(STEP_SIZE)
    vis.Advance(STEP_SIZE)

    # Real-time pacing
    realtime_timer.Spin(STEP_SIZE)

    step_number += 1