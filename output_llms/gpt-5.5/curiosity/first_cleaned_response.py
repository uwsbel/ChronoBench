import os
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr





Vec = getattr(chrono, "ChVector3d", getattr(chrono, "ChVectorD", None))
Quat = getattr(chrono, "ChQuaterniond", getattr(chrono, "ChQuaternionD", None))
Coordsys = getattr(chrono, "ChCoordsysd", getattr(chrono, "ChCoordsysD", None))


def set_gravity(system, gvec):
    if hasattr(system, "SetGravitationalAcceleration"):
        system.SetGravitationalAcceleration(gvec)
    else:
        system.Set_G_acc(gvec)


def safe_call(obj, method_name, *args):
    if hasattr(obj, method_name):
        return getattr(obj, method_name)(*args)
    return None





step_size = 2.0e-3
render_step_size = 1.0 / 60.0

terrain_length = 80.0
terrain_width = 80.0
terrain_friction = 0.9
terrain_restitution = 0.01

init_loc = Vec(0.0, 0.0, 0.55)
init_rot = Quat(1.0, 0.0, 0.0, 0.0)

camera_track_point = Vec(0.0, 0.0, 0.8)
camera_distance = 6.0
camera_height = 1.5

window_width = 1280
window_height = 720



contact_method = chrono.ChContactMethod_SMC






if "CHRONO_DATA_DIR" in os.environ:
    chrono.SetChronoDataPath(os.environ["CHRONO_DATA_DIR"])

veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")





system = chrono.ChSystemSMC()


set_gravity(system, Vec(0.0, 0.0, -9.81))


if hasattr(system, "SetCollisionSystemType") and hasattr(chrono, "ChCollisionSystem"):
    try:
        system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    except Exception:
        pass


if hasattr(system, "SetSolverType") and hasattr(chrono, "ChSolver"):
    try:
        system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    except Exception:
        pass






try:
    rover = veh.Curiosity(system)
except TypeError:
    rover = veh.Curiosity()

safe_call(rover, "SetContactMethod", contact_method)
safe_call(rover, "SetChassisFixed", False)
safe_call(rover, "SetInitPosition", Coordsys(init_loc, init_rot))


VIS_MESH = getattr(veh, "VisualizationType_MESH", None)
VIS_PRIMITIVES = getattr(veh, "VisualizationType_PRIMITIVES", None)

if VIS_MESH is not None:
    safe_call(rover, "SetChassisVisualizationType", VIS_MESH)
    safe_call(rover, "SetWheelVisualizationType", VIS_MESH)
    safe_call(rover, "SetTireVisualizationType", VIS_MESH)

if VIS_PRIMITIVES is not None:
    safe_call(rover, "SetSuspensionVisualizationType", VIS_PRIMITIVES)
    safe_call(rover, "SetSteeringVisualizationType", VIS_PRIMITIVES)


rover.Initialize()


vehicle = rover.GetVehicle() if hasattr(rover, "GetVehicle") else rover





terrain = veh.RigidTerrain(system)

if contact_method == chrono.ChContactMethod_SMC:
    terrain_mat = chrono.ChContactMaterialSMC()
else:
    terrain_mat = chrono.ChContactMaterialNSC()

terrain_mat.SetFriction(terrain_friction)
terrain_mat.SetRestitution(terrain_restitution)


safe_call(terrain_mat, "SetYoungModulus", 2.0e7)
safe_call(terrain_mat, "SetPoissonRatio", 0.3)
safe_call(terrain_mat, "SetAdhesion", 0.0)


patch = terrain.AddPatch(
    terrain_mat,
    chrono.CSYSNORM,
    terrain_length,
    terrain_width
)

patch.SetColor(chrono.ChColor(0.55, 0.50, 0.42))


try:
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 160, 160)
except Exception:
    pass

terrain.Initialize()





VisualSystemClass = getattr(
    veh,
    "ChWheeledVehicleVisualSystemIrrlicht",
    getattr(veh, "ChVehicleVisualSystemIrrlicht", None)
)

vis = VisualSystemClass()
vis.SetWindowTitle("PyChrono Curiosity Rover - Rigid Terrain")
vis.SetWindowSize(window_width, window_height)
vis.SetChaseCamera(camera_track_point, camera_distance, camera_height)
vis.AttachVehicle(vehicle)


safe_call(vis, "SetSymbolScale", 1.0)
safe_call(vis, "EnableShadows")

vis.Initialize()


safe_call(vis, "AddSkyBox")
safe_call(vis, "AddLightDirectional")

try:
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
except Exception:
    pass








driver = veh.ChInteractiveDriverIRR(vis)


steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3

driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()





realtime_timer = chrono.ChRealtimeStepTimer()
render_steps = max(1, int(render_step_size / step_size))
step_number = 0

while vis.Run():
    time = system.GetChTime()

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    rover.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    rover.Advance(step_size)
    vis.Advance(step_size)

    
    realtime_timer.Spin(step_size)

    step_number += 1