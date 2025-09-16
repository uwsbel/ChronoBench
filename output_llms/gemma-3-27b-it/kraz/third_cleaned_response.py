import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc_kraz = chrono.ChVector3d(-10, 0, 0.5)
initRot_kraz = chrono.ChQuaterniond(1, 0, 0, 0)


initLoc_sedan = chrono.ChVector3d(10, 0, 0.5)
initRot_sedan = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model_kraz = veh.TireModelType_RIGID
tire_model_sedan = veh.TireModelType_TMEASY



terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0


trackPoint = chrono.ChVector3d(0, 0, 2.1)


contact_method = chrono.ChContactMethod_NSC
contact_vis = False


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 50  


vehicle_kraz = veh.Kraz()
vehicle_kraz.SetContactMethod(contact_method)
vehicle_kraz.SetChassisCollisionType(chassis_collision_type)
vehicle_kraz.SetChassisFixed(False)
vehicle_kraz.SetInitPosition(chrono.ChCoordsysd(initLoc_kraz, initRot_kraz))
vehicle_kraz.Initialize()
vehicle_kraz.SetChassisVisualizationType(vis_type, vis_type)
vehicle_kraz.SetSteeringVisualizationType(vis_type)
vehicle_kraz.SetSuspensionVisualizationType(vis_type, vis_type)
vehicle_kraz.SetWheelVisualizationType(vis_type, vis_type)
vehicle_kraz.SetTireVisualizationType(vis_type, vis_type)
vehicle_kraz.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
vehicle_kraz.SetTireModel(tire_model_kraz)


vehicle_sedan = veh.Sedan()
vehicle_sedan.SetContactMethod(contact_method)
vehicle_sedan.SetChassisCollisionType(chassis_collision_type)
vehicle_sedan.SetChassisFixed(False)
vehicle_sedan.SetInitPosition(chrono.ChCoordsysd(initLoc_sedan, initRot_sedan))
vehicle_sedan.Initialize()
vehicle_sedan.SetChassisVisualizationType(vis_type, vis_type)
vehicle_sedan.SetSteeringVisualizationType(vis_type)
vehicle_sedan.SetSuspensionVisualizationType(vis_type, vis_type)
vehicle_sedan.SetWheelVisualizationType(vis_type, vis_type)
vehicle_sedan.SetTireVisualizationType(vis_type, vis_type)
vehicle_sedan.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
vehicle_sedan.SetTireModel(tire_model_sedan)


patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle_kraz.GetSystem())
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), terrainLength, terrainWidth)


terrain.ImportFromObj(chrono.GetChronoDataFile("terrain/highway/highway.obj"))
terrain.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Kraz and Sedan Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 25.0, 1.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle_kraz.GetTractor())
vis.AttachVehicle(vehicle_sedan.GetTractor())


driver_kraz = veh.ChInteractiveDriverIRR(vis)
steering_time_kraz = 1.0
throttle_time_kraz = 1.0
braking_time_kraz = 0.3
driver_kraz.SetSteeringDelta(render_step_size / steering_time_kraz)
driver_kraz.SetThrottleDelta(render_step_size / throttle_time_kraz)
driver_kraz.SetBrakingDelta(render_step_size / braking_time_kraz)
driver_kraz.Initialize()


driver_sedan = veh.ChInteractiveDriverIRR(vis)
steering_time_sedan = 1.0
throttle_time_sedan = 1.0
braking_time_sedan = 0.3
driver_sedan.SetSteeringDelta(render_step_size / steering_time_sedan)
driver_sedan.SetThrottleDelta(render_step_size / throttle_time_sedan)
driver_sedan.SetBrakingDelta(render_step_size / braking_time_sedan)
driver_sedan.Initialize()


driver_sedan.SetSteering(0.2)
driver_sedan.SetThrottle(1.0)
driver_sedan.SetBraking(0.0)


tractor_kraz = vehicle_kraz.GetTractor()
trailer_kraz = vehicle_kraz.GetTrailer()


print("VEHICLE KRAZ MASS: ", vehicle_kraz.GetTractor().GetMass())
print("VEHICLE SEDAN MASS: ", vehicle_sedan.GetTractor().GetMass())


render_steps = math.ceil(render_step_size / step_size)


s = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle_kraz.GetSystem().GetChTime()

    
    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    driver_inputs_kraz = driver_kraz.GetInputs()
    driver_inputs_sedan = driver_sedan.GetInputs()

    
    driver_kraz.Synchronize(time)
    driver_sedan.Synchronize(time)
    terrain.Synchronize(time)
    vehicle_kraz.Synchronize(time, driver_inputs_kraz, terrain)
    vehicle_sedan.Synchronize(time, driver_inputs_sedan, terrain)
    vis.Synchronize(time, driver_inputs_kraz)
    vis.Synchronize(time, driver_inputs_sedan)

    
    driver_kraz.Advance(step_size)
    driver_sedan.Advance(step_size)
    terrain.Advance(step_size)
    vehicle_kraz.Advance(step_size)
    vehicle_sedan.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1

    
    s.Spin(step_size)