import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


kraz_initLoc = chrono.ChVector3d(-50, 0, 0.5)
kraz_initRot = chrono.ChQuaterniond(1, 0, 0, 0)


sedan_initLoc = chrono.ChVector3d(50, 0, 0.5)
sedan_initRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


kraz_tire_model = veh.TireModelType_RIGID
sedan_tire_model = veh.TireModelType_TMEASY


terrain_model = veh.RigidTerrain.BOX
terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0


trackPoint = chrono.ChVector3d(0, 0, 2.1)


contact_method = chrono.ChContactMethod_NSC
contact_vis = False


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 50  


kraz_vehicle = veh.Kraz()
kraz_vehicle.SetContactMethod(contact_method)
kraz_vehicle.SetChassisCollisionType(chassis_collision_type)
kraz_vehicle.SetChassisFixed(False)
kraz_vehicle.SetInitPosition(chrono.ChCoordsysd(kraz_initLoc, kraz_initRot))
kraz_vehicle.Initialize()
kraz_vehicle.SetChassisVisualizationType(vis_type, vis_type)
kraz_vehicle.SetSteeringVisualizationType(vis_type)
kraz_vehicle.SetSuspensionVisualizationType(vis_type, vis_type)
kraz_vehicle.SetWheelVisualizationType(vis_type, vis_type)
kraz_vehicle.SetTireVisualizationType(vis_type, vis_type)
kraz_vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
kraz_vehicle.SetTireModel(kraz_tire_model)


sedan_vehicle = veh.Sedan()
sedan_vehicle.SetContactMethod(contact_method)
sedan_vehicle.SetChassisCollisionType(chassis_collision_type)
sedan_vehicle.SetChassisFixed(False)
sedan_vehicle.SetInitPosition(chrono.ChCoordsysd(sedan_initLoc, sedan_initRot))
sedan_vehicle.Initialize()
sedan_vehicle.SetChassisVisualizationType(vis_type, vis_type)
sedan_vehicle.SetSteeringVisualizationType(vis_type)
sedan_vehicle.SetSuspensionVisualizationType(vis_type, vis_type)
sedan_vehicle.SetWheelVisualizationType(vis_type, vis_type)
sedan_vehicle.SetTireVisualizationType(vis_type, vis_type)
sedan_vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sedan_vehicle.SetTireModel(sedan_tire_model)


patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(kraz_vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), terrainLength, terrainWidth)

patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Kraz and Sedan Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 25.0, 1.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(kraz_vehicle.GetTractor())
vis.AttachVehicle(sedan_vehicle.GetTractor())


kraz_driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
kraz_driver.SetSteeringDelta(render_step_size / steering_time)
kraz_driver.SetThrottleDelta(render_step_size / throttle_time)
kraz_driver.SetBrakingDelta(render_step_size / braking_time)
kraz_driver.Initialize()


sedan_driver = veh.ChInteractiveDriverIRR(vis)
sedan_driver.SetSteeringDelta(render_step_size / steering_time)
sedan_driver.SetThrottleDelta(render_step_size / throttle_time)
sedan_driver.SetBrakingDelta(render_step_size / braking_time)
sedan_driver.Initialize()


sedan_driver.SetSteering(0.2)
sedan_driver.SetThrottle(1.0)


kraz_tractor = kraz_vehicle.GetTractor()
if hasattr(kraz_vehicle, 'GetTrailer'):
    kraz_trailer = kraz_vehicle.GetTrailer()
else:
    kraz_trailer = None


print("KRAZ VEHICLE MASS: ", kraz_vehicle.GetTractor().GetMass())
print("SEDAN VEHICLE MASS: ", sedan_vehicle.GetTractor().GetMass())


render_steps = math.ceil(render_step_size / step_size)


s = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = kraz_vehicle.GetSystem().GetChTime()

    
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
    kraz_vehicle.Synchronize(time, kraz_driver_inputs, terrain)
    sedan_vehicle.Synchronize(time, sedan_driver_inputs, terrain)
    vis.Synchronize(time, kraz_driver_inputs)

    
    kraz_driver.Advance(step_size)
    sedan_driver.Advance(step_size)
    terrain.Advance(step_size)
    kraz_vehicle.Advance(step_size)
    sedan_vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1

    
    s.Spin(step_size)