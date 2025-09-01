import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLocTruck = chrono.ChVector3d(0, -5, 0.5)
initRotTruck = chrono.ChQuaterniond(chrono.Q_from_AngZ(math.pi/4))


initLocSedan = chrono.ChVector3d(10, 5, 0.5)
initRotSedan = chrono.ChQuaterniond(1, 0, 0, 0)


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


vehicle = veh.Kraz()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLocTruck, initRotTruck))
vehicle.SetTireType(tire_model_truck)
vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)


sedan = veh.HMMWV_Full()
sedan.SetContactMethod(contact_method)
sedan.SetChassisCollisionType(chassis_collision_type)
sedan.SetChassisFixed(False)
sedan.SetInitPosition(chrono.ChCoordsysd(initLocSedan, initRotSedan))
sedan.SetTireType(tire_model_sedan)
sedan.Initialize()

sedan.SetChassisVisualizationType(vis_type)
sedan.SetSuspensionVisualizationType(vis_type)
sedan.SetSteeringVisualizationType(vis_type)
sedan.SetWheelVisualizationType(vis_type)
sedan.SetTireVisualizationType(vis_type)


terrain = veh.RigidTerrain(vehicle.GetSystem())
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain.LoadPatch(veh.GetDataFile("highway/Highway.obj"), "highway", patch_mat)
terrain.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Vehicle Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 25.0, 1.5)
vis.Initialize()
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetTractor())


driver_truck = veh.ChInteractiveDriverIRR(vis)


steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver_truck.SetSteeringDelta(render_step_size / steering_time)
driver_truck.SetThrottleDelta(render_step_size / throttle_time)
driver_truck.SetBrakingDelta(render_step_size / braking_time)

driver_truck.Initialize()


driver_sedan = veh.ChDriver(sedan.GetVehicle())
driver_sedan.Initialize()


render_steps = math.ceil(render_step_size / step_size)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

tractor_states = []
trailer_states = []

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    driver_inputs_truck = driver_truck.GetInputs()

    
    driver_inputs_sedan = veh.DriverInputs()
    driver_inputs_sedan.m_steering = 0
    driver_inputs_sedan.m_throttle = 0.5
    driver_inputs_sedan.m_braking = 0

    
    driver_truck.Synchronize(time)
    driver_sedan.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs_truck, terrain)
    sedan.Synchronize(time, driver_inputs_sedan, terrain)
    vis.Synchronize(time, driver_inputs_truck)

    
    driver_truck.Advance(step_size)
    driver_sedan.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    sedan.Advance(step_size)
    vis.Advance(step_size)

    
    tractor_states.append(vehicle.GetTractor().GetState())
    trailer_states.append(vehicle.GetTrailer().GetState())

    
    step_number += 1

    
    realtime_timer.Spin(step_size)