import os
import math

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr











if "CHRONO_DATA_DIR" in os.environ:
    chrono.SetChronoDataPath(os.environ["CHRONO_DATA_DIR"])

veh.SetDataPath(os.path.join(chrono.GetChronoDataPath(), "vehicle") + "/")






CONTACT_METHOD = chrono.ChContactMethod_NSC


INIT_LOC = chrono.ChVector3d(0.0, 0.0, 0.75)
INIT_ROT = chrono.QUNIT


SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
COL_MESH_FILE = os.path.join(SCRIPT_DIR, "Highway_col.obj")
VIS_MESH_FILE = os.path.join(SCRIPT_DIR, "Highway_vis.obj")


STEP_SIZE = 1.0e-3
TIRE_STEP_SIZE = 1.0e-3
FPS = 50
RENDER_STEP_SIZE = 1.0 / FPS


STEERING_TIME = 1.0
THROTTLE_TIME = 1.0
BRAKING_TIME = 0.3


TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01
TERRAIN_YOUNG_MODULUS = 2.0e7
TERRAIN_POISSON_RATIO = 0.3
TERRAIN_COLLISION_ENVELOPE = 0.005






def make_contact_material(contact_method):
    
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
    

    if not os.path.isfile(col_mesh_file):
        raise FileNotFoundError(f"Collision mesh not found: {col_mesh_file}")

    if not os.path.isfile(vis_mesh_file):
        raise FileNotFoundError(f"Visual mesh not found: {vis_mesh_file}")

    terrain = veh.RigidTerrain(system)

    terrain_mat = make_contact_material(contact_method)
    terrain_pos = chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT)

    
    
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
        
        patch = terrain.AddPatch(
            terrain_mat,
            terrain_pos,
            col_mesh_file,
            "Highway_collision_mesh",
            TERRAIN_COLLISION_ENVELOPE,
            False,
        )

    terrain.Initialize()

    
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






hmmwv = veh.HMMWV_Full()

hmmwv.SetContactMethod(CONTACT_METHOD)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))


hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)


hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(TIRE_STEP_SIZE)


hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)

hmmwv.Initialize()


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


try:
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
except Exception:
    pass






terrain = create_mesh_terrain(system, CONTACT_METHOD, COL_MESH_FILE, VIS_MESH_FILE)






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






driver = veh.ChInteractiveDriverIRR(vis)

driver.SetSteeringDelta(RENDER_STEP_SIZE / STEERING_TIME)
driver.SetThrottleDelta(RENDER_STEP_SIZE / THROTTLE_TIME)
driver.SetBrakingDelta(RENDER_STEP_SIZE / BRAKING_TIME)

driver.Initialize()






realtime_timer = chrono.ChRealtimeStepTimer()

render_steps = max(1, int(math.ceil(RENDER_STEP_SIZE / STEP_SIZE)))
step_number = 0

while vis.Run():
    time = system.GetChTime()

    
    if step_number % render_steps == 0:
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

    
    realtime_timer.Spin(STEP_SIZE)

    step_number += 1