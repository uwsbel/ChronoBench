import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


kraz_initLoc = chrono.ChVector3d(0, 0, 0.5)
kraz_initRot = chrono.ChQuaterniond(1, 0, 0, 0)
sedan_initLoc = chrono.ChVector3d(5, 0, 0.5)
sedan_initRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


kraz_tire_model = veh.TireModelType_RIGID
sedan_tire_model = veh.TireModelType_TMEASY


terrain_mesh = veh.GetDataFile("terrain/meshes/highway.obj")


contact_method = chrono.ChContactMethod_NSC
contact_vis = False


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 50  


kraz = veh.Kraz()
kraz.SetContactMethod(contact_method)
kraz.SetChassisCollisionType(chassis_collision_type)
kraz.SetChassisFixed(False)
kraz.SetInitPosition(chrono.ChCoordsysd(kraz_initLoc, kraz_initRot))
kraz.Initialize()

kraz.SetChassisVisualizationType(vis_type, vis_type)
kraz.SetSteeringVisualizationType(vis_type)
kraz.SetSuspensionVisualizationType(vis_type, vis_type)
kraz.SetWheelVisualizationType(vis_type, vis_type)
kraz.SetTireVisualizationType(vis_type, vis_type)


sedan = veh.Sedan()
sedan.SetContactMethod(contact_method)
sedan.SetChassisCollisionType(chassis_collision_type)
sedan.SetChassisFixed(False)
sedan.SetInitPosition(chrono.ChCoordsysd(sedan_initLoc, sedan_initRot))
sedan.Initialize()

sedan.SetChassisVisualizationType(vis_type, vis_type)
sedan.SetSteeringVisualizationType(vis_type)
sedan.SetSuspensionVisualizationType(vis_type, vis_type)
sedan.SetWheelVisualizationType(vis_type, vis_type)
sedan.SetTireVisualizationType(vis_type, vis_type)


patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(kraz.GetSystem())
patch = terrain.AddPatch(patch_mat, terrain_mesh)

patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Kraz and Sedan Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(kraz_initLoc, 25.0, 1.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(kraz.GetTractor())
vis.AttachVehicle(sedan.GetTractor())


kraz_driver = veh.ChInteractiveDriverIRR(vis)
sedan_driver = veh.ChPathFollowerDriver(sedan.GetVehicle(), "my_path.txt", "my_path_pts.txt", 1.0)


steering_time = 1.0  
throttle_time = 1.0  
braking_time = 0.3   
kraz_driver.SetSteeringDelta(render_step_size / steering_time)
kraz_driver.SetThrottleDelta(render_step_size / throttle_time)
kraz_driver.SetBrakingDelta(render_step_size / braking_time)

kraz_driver.Initialize()
sedan_driver.Initialize()


print("KRAZ MASS: ", kraz.GetTractor().GetMass())
print("SEDAN MASS: ", sedan.GetTractor().GetMass())


render_steps = math.ceil(render_step_size / step_size)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0


tractor_states = []
trailer_states = []

while vis.Run():
    time = kraz.GetSystem().GetChTime()

    
    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    kraz_driver_inputs = kraz_driver.GetInputs()
    sedan_driver_inputs = sedan_driver.GetInputs()

    
    kraz_driver.Synchronize(time)
    sedan_driver.Synchronize(time)
    terrain.Synchronize(time)
    kraz.Synchronize(time, kraz_driver_inputs, terrain)
    sedan.Synchronize(time, sedan_driver_inputs, terrain)
    vis.Synchronize(time, kraz_driver_inputs)

    
    kraz_driver.Advance(step_size)
    sedan_driver.Advance(step_size)
    terrain.Advance(step_size)
    kraz.Advance(step_size)
    sedan.Advance(step_size)
    vis.Advance(step_size)

    
    tractor_states.append(kraz.GetTractor().GetPos())
    trailer_states.append(kraz.GetTrailer().GetPos())

    
    step_number += 1

    
    realtime_timer.Spin(step_size)