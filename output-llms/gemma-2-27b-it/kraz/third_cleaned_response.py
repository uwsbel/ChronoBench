import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc_truck = chrono.ChVector3d(0, 0, 0.5)
initRot_truck = chrono.ChQuaterniond(1, 0, 0, 0)

initLoc_sedan = chrono.ChVector3d(5, 0, 0.5)
initRot_sedan = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model_truck = veh.TireModelType_RIGID
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


vehicle_truck = veh.Kraz()
vehicle_truck.SetContactMethod(contact_method)
vehicle_truck.SetChassisCollisionType(chassis_collision_type)
vehicle_truck.SetChassisFixed(False)
vehicle_truck.SetInitPosition(chrono.ChCoordsysd(initLoc_truck, initRot_truck))
vehicle_truck.Initialize()

vehicle_truck.SetChassisVisualizationType(vis_type, vis_type)
vehicle_truck.SetSteeringVisualizationType(vis_type)
vehicle_truck.SetSuspensionVisualizationType(vis_type, vis_type)
vehicle_truck.SetWheelVisualizationType(vis_type, vis_type)
vehicle_truck.SetTireVisualizationType(vis_type, vis_type)

vehicle_truck.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


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


patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle_truck.GetSystem())
patch = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    terrainLength, terrainWidth)

patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
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
vis.AttachVehicle(vehicle_truck.GetTractor())
vis.AttachVehicle(vehicle_sedan.GetChassis())


driver_truck = veh.ChInteractiveDriverIRR(vis)


steering_time_truck = 1.0  
throttle_time_truck = 1.0  
braking_time_truck = 0.3   
driver_truck.SetSteeringDelta(render_step_size / steering_time_truck)
driver_truck.SetThrottleDelta(render_step_size / throttle_time_truck)
driver_truck.SetBrakingDelta(render_step_size / braking_time_truck)

driver_truck.Initialize()


driver_sedan = veh.ChDriver(vehicle_sedan.GetChassis())
driver_sedan.SetSteering(0.2)
driver_sedan.SetThrottle(0.5)


tractor_states = []
trailer_states = []


render_steps = math.ceil(render_step_size / step_size)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle_truck.GetSystem().GetChTime()

    
    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    driver_inputs_truck = driver_truck.GetInputs()

    
    driver_truck.Synchronize(time)
    driver_sedan.Synchronize(time)
    terrain.Synchronize(time)
    vehicle_truck.Synchronize(time, driver_inputs_truck, terrain)
    vehicle_sedan.Synchronize(time, driver_sedan.GetInputs(), terrain)
    vis.Synchronize(time, driver_inputs_truck)

    
    driver_truck.Advance(step_size)
    driver_sedan.Advance(step_size)
    terrain.Advance(step_size)
    vehicle_truck.Advance(step_size)
    vehicle_sedan.Advance(step_size)
    vis.Advance(step_size)

    
    tractor_states.append(vehicle_truck.GetTractor().GetPos())
    trailer_states.append(vehicle_truck.GetTrailer().GetPos())

    
    step_number += 1

    
    realtime_timer.Spin(step_size)