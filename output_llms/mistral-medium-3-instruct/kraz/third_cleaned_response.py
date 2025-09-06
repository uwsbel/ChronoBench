import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


truck_initLoc = chrono.ChVector3d(0, 0, 0.5)
truck_initRot = chrono.ChQuaterniond(1, 0, 0, 0)  

sedan_initLoc = chrono.ChVector3d(5, 0, 0.5)      
sedan_initRot = chrono.ChQuaterniond(1, 0, 0, 0)  


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


truck_tire_model = veh.TireModelType_RIGID  
sedan_tire_model = veh.TireModelType_TMEASY


terrainHeight = 0      
terrainLength = 100.0  
terrainWidth = 100.0   


trackPoint = chrono.ChVector3d(0, 0, 2.1)


contact_method = chrono.ChContactMethod_NSC
contact_vis = False


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 50  


truck = veh.Kraz()
truck.SetContactMethod(contact_method)
truck.SetChassisCollisionType(chassis_collision_type)
truck.SetChassisFixed(False)
truck.SetInitPosition(chrono.ChCoordsysd(truck_initLoc, truck_initRot))
truck.Initialize()

truck.SetChassisVisualizationType(vis_type, vis_type)
truck.SetSteeringVisualizationType(vis_type)
truck.SetSuspensionVisualizationType(vis_type, vis_type)
truck.SetWheelVisualizationType(vis_type, vis_type)
truck.SetTireVisualizationType(vis_type, vis_type)


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


truck.SetTireTypeFront(truck_tire_model)
truck.SetTireTypeRear(truck_tire_model)
sedan.SetTireTypeFront(sedan_tire_model)
sedan.SetTireTypeRear(sedan_tire_model)


patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)


terrain = veh.RigidTerrain(vehicle.GetSystem())
highway_mesh = veh.GetDataFile("terrain/meshes/highway.obj")
terrain.AddVisualizationMesh(highway_mesh, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                           chrono.ChVector2d(terrainLength, terrainWidth))
terrain.AddContactMesh(highway_mesh, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                      chrono.ChVector2d(terrainLength, terrainWidth))

terrain.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Vehicle Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 25.0, 1.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(truck.GetTractor())
vis.AttachVehicle(sedan.GetVehicle())


truck_driver = veh.ChInteractiveDriverIRR(vis)
sedan_driver = veh.ChDataDriver()


steering_time = 1.0  
throttle_time = 1.0  
braking_time = 0.3   
truck_driver.SetSteeringDelta(render_step_size / steering_time)
truck_driver.SetThrottleDelta(render_step_size / throttle_time)
truck_driver.SetBrakingDelta(render_step_size / braking_time)
truck_driver.Initialize()


sedan_throttle = 0.5  
sedan_steering = 0.0  
sedan_driver.SetSteering(sedan_steering)
sedan_driver.SetThrottle(sedan_throttle)
sedan_driver.SetBraking(0.0)
sedan_driver.Initialize()


print("TRUCK MASS: ", truck.GetTractor().GetMass())
print("SEDAN MASS: ", sedan.GetVehicle().GetMass())


render_steps = math.ceil(render_step_size / step_size)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0


truck_states = []

while vis.Run():
    time = truck.GetSystem().GetChTime()

    
    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    truck_driver_inputs = truck_driver.GetInputs()

    
    sedan_driver_inputs = veh.ChDriverInputs()
    sedan_driver_inputs.m_throttle = sedan_throttle
    sedan_driver_inputs.m_steering = sedan_steering

    
    truck_driver.Synchronize(time)
    sedan_driver.Synchronize(time)
    terrain.Synchronize(time)
    truck.Synchronize(time, truck_driver_inputs, terrain)
    sedan.Synchronize(time, sedan_driver_inputs, terrain)
    vis.Synchronize(time, truck_driver_inputs)

    
    truck_driver.Advance(step_size)
    sedan_driver.Advance(step_size)
    terrain.Advance(step_size)
    truck.Advance(step_size)
    sedan.Advance(step_size)
    vis.Advance(step_size)

    
    tractor_pos = truck.GetTractor().GetPos()
    trailer_pos = truck.GetTrailer().GetPos() if truck.GetTrailer() else None
    truck_states.append({
        'time': time,
        'tractor_pos': (tractor_pos.x, tractor_pos.y, tractor_pos.z),
        'trailer_pos': (trailer_pos.x, trailer_pos.y, trailer_pos.z) if trailer_pos else None
    })

    
    step_number += 1

    
    realtime_timer.Spin(step_size)