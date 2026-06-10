import os
import math

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr  






Vec = chrono.ChVector3d if hasattr(chrono, "ChVector3d") else chrono.ChVectorD
Coordsys = chrono.ChCoordsysd if hasattr(chrono, "ChCoordsysd") else chrono.ChCoordsysD

if hasattr(chrono, "ChQuaterniond"):
    Quat = chrono.ChQuaterniond
else:
    Quat = chrono.ChQuaternionD


def yaw_quaternion(yaw):
    if hasattr(chrono, "QuatFromAngleZ"):
        return chrono.QuatFromAngleZ(yaw)
    if hasattr(chrono, "Q_from_AngZ"):
        return chrono.Q_from_AngZ(yaw)
    return Quat(math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0))


def make_contact_material_nsc():
    if hasattr(chrono, "ChContactMaterialNSC"):
        return chrono.ChContactMaterialNSC()
    return chrono.ChMaterialSurfaceNSC()






if "CHRONO_DATA_DIR" in os.environ:
    chrono.SetChronoDataPath(os.environ["CHRONO_DATA_DIR"])

if "CHRONO_VEHICLE_DATA_DIR" in os.environ:
    veh.SetDataPath(os.environ["CHRONO_VEHICLE_DATA_DIR"])
else:
    veh.SetDataPath(os.path.join(chrono.GetChronoDataPath(), "vehicle") + os.sep)






contact_method = chrono.ChContactMethod_NSC


init_location = Vec(0.0, 0.0, 0.5)
init_yaw = 0.0
init_orientation = yaw_quaternion(init_yaw)
init_pose = Coordsys(init_location, init_orientation)


tire_model = veh.TireModelType_TMEASY
tire_step_size = 1.0e-3


terrain_length = 300.0
terrain_width = 300.0
terrain_friction = 0.9
terrain_restitution = 0.01
terrain_texture = veh.GetDataFile("terrain/textures/tile4.jpg")


render_fps = 50
render_step_size = 1.0 / render_fps
step_size = 2.0e-3


steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3






bus = veh.CityBus()
bus.SetContactMethod(contact_method)
bus.SetChassisCollisionType(veh.CollisionType_NONE)
bus.SetChassisFixed(False)
bus.SetInitPosition(init_pose)
bus.SetTireType(tire_model)
bus.SetTireStepSize(tire_step_size)
bus.Initialize()




bus.SetChassisVisualizationType(veh.VisualizationType_MESH)
bus.SetWheelVisualizationType(veh.VisualizationType_MESH)
bus.SetTireVisualizationType(veh.VisualizationType_MESH)
bus.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
bus.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)

system = bus.GetSystem()

if hasattr(system, "SetCollisionSystemType"):
    try:
        system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    except Exception:
        pass






terrain = veh.RigidTerrain(system)

patch_mat = make_contact_material_nsc()
patch_mat.SetFriction(terrain_friction)
patch_mat.SetRestitution(terrain_restitution)

terrain_pose = Coordsys(Vec(0.0, 0.0, 0.0), yaw_quaternion(0.0))
patch = terrain.AddPatch(patch_mat, terrain_pose, terrain_length, terrain_width)
patch.SetTexture(terrain_texture, 80.0, 80.0)
patch.SetColor(chrono.ChColor(0.6, 0.6, 0.6))

terrain.Initialize()






vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("PyChrono CityBus on Rigid Terrain")
vis.SetWindowSize(1280, 720)





vis.SetChaseCamera(Vec(0.0, 0.0, 1.75), 10.0, 1.0)

vis.Initialize()
vis.AddSkyBox()
vis.AddLightDirectional()

try:
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
except Exception:
    pass

vis.AttachVehicle(bus.GetVehicle())






driver = veh.ChInteractiveDriverIRR(vis)

driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()






realtime_timer = chrono.ChRealtimeStepTimer()

while vis.Run():
    time = system.GetChTime()

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    bus.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    frame_time = 0.0
    while frame_time < render_step_size:
        h = min(step_size, render_step_size - frame_time)

        driver.Advance(h)
        terrain.Advance(h)
        bus.Advance(h)
        vis.Advance(h)

        frame_time += h

    
    realtime_timer.Spin(render_step_size)