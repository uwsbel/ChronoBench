import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


truck_initLoc = chrono.ChVector3d(0, 0, 0.5)
truck_initRot = chrono.ChQuaterniond(1, 0, 0, 0)
sedan_initLoc = chrono.ChVector3d(10, 0, 0.5)
sedan_initRot = chrono.ChQuaterniond(1, 0, 0, 0)


truck_vis_type = veh.VisualizationType_MESH
sedan_vis_type = veh.VisualizationType_MESH


truck_chassis_collision_type = veh.CollisionType_MESH
sedan_chassis_collision_type = veh.CollisionType_MESH


truck_tire_model = veh.TireModelType_RIGID
sedan_tire_model = veh.TireModelType_TMEASY


terrain_model = veh.PredefinedTerrain.HIGHWAY
terrainHeight = 0      
terrainLength = 100.0  
terrainWidth = 100.0   


truck_trackPoint = chrono.ChVector3d(0,0, 2.1)
sedan_trackPoint = chrono.ChVector3d(10,0, 2.1)


contact_method = chrono.ChContactMethod_NSC
contact_vis = False


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 50  


truck = veh.Kraz()
truck.SetContactMethod(contact_method)
truck.SetChassisCollisionType(truck_chassis_collision_type)
truck.SetChassisFixed(False)
truck.SetInitPosition(chrono.ChCoordsysd(truck_initLoc, truck_initRot))
truck.Initialize()

truck.SetChassisVisualizationType(truck_vis_type, truck_vis_type)
truck.SetSteeringVisualizationType(truck_vis_type)
truck.SetSuspensionVisualizationType(truck_vis_type, truck_vis_type)
truck.SetWheelVisualizationType(truck_vis_type, truck_vis_type)
truck.SetTireVisualizationType(truck_vis_type, truck_vis_type)

truck.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


sedan = veh.Sedan()
sedan.SetContactMethod(contact_method)
sedan.SetChassisCollisionType(sedan_chassis_collision_type)
sedan.SetChassisFixed(False)
sedan.SetInitPosition(chrono.ChCoordsysd(sedan_initLoc, sedan_initRot))
sedan.Initialize()

sedan.SetChassisVisualizationType(sedan_vis_type, sedan_vis_type)
sedan.SetSteeringVisualizationType(sedan_vis_type)
sedan.SetSuspensionVisualizationType(sedan_vis_type, sedan_vis_type)
sedan.SetWheelVisualizationType(sedan_vis_type, sedan_vis_type)
sedan.SetTireVisualizationType(sedan_vis_type, sedan_vis_type)

sedan.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.PredefinedTerrain(sedan.GetSystem(), terrain_model)
terrain.Initialize()


truck_vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
truck_vis.SetWindowTitle('Truck Demo')
truck_vis.SetWindowSize(1280, 1024)
truck_vis.SetChaseCamera(truck_trackPoint, 25.0, 1.5)
truck_vis.Initialize()
truck_vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
truck_vis.AddLightDirectional()
truck_vis.AddSkyBox()
truck_vis.AttachVehicle(truck.GetTractor())

sedan_vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
sedan_vis.SetWindowTitle('Sedan Demo')
sedan_vis.SetWindowSize(1280, 1024)
sedan_vis.SetChaseCamera(sedan_trackPoint, 25.0, 1.5)
sedan_vis.Initialize()
sedan_vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
sedan_vis.AddLightDirectional()
sedan_vis.AddSkyBox()
sedan_vis.AttachVehicle(sedan.GetTractor())


truck_driver = veh.ChInteractiveDriverIRR(truck_vis)
sedan_driver = veh.ChInteractiveDriverIRR(sedan_vis)


steering_time = 1.0  
throttle_time = 1.0  
braking_time = 0.3   
truck_driver.SetSteeringDelta(render_step_size / steering_time)
truck_driver.SetThrottleDelta(render_step_size / throttle_time)
truck_driver.SetBrakingDelta(render_step_size / braking_time)
sedan_driver.SetSteeringDelta(render_step_size / steering_time)
sedan_driver.SetThrottleDelta(render_step_size / throttle_time)
sedan_driver.SetBrakingDelta(render_step_size / braking_time)

truck_driver.Initialize()
sedan_driver.Initialize()


print( "TRUCK MASS: ",  truck.GetTractor().GetMass())
print( "SEDAN MASS: ",  sedan.GetTractor().GetMass())


render_steps = math.ceil(render_step_size / step_size)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while truck_vis.Run() and sedan_vis.Run() :
    time = truck.GetSystem().GetChTime()

    
    if (step_number % render_steps == 0) :
        truck_vis.BeginScene()
        truck_vis.Render()
        truck_vis.EndScene()
        sedan_vis.BeginScene()
        sedan_vis.Render()
        sedan_vis.EndScene()
        render_frame += 1

    
    truck_driver_inputs = truck_driver.GetInputs()
    sedan_driver_inputs = sedan_driver.GetInputs()

    
    truck_driver.Synchronize(time)
    sedan_driver.Synchronize(time)
    terrain.Synchronize(time)
    truck.Synchronize(time, truck_driver_inputs, terrain)
    sedan.Synchronize(time, sedan_driver_inputs, terrain)
    truck_vis.Synchronize(time, truck_driver_inputs)
    sedan_vis.Synchronize(time, sedan_driver_inputs)

    
    truck_driver.Advance(step_size)
    sedan_driver.Advance(step_size)
    terrain.Advance(step_size)
    truck.Advance(step_size)
    sedan.Advance(step_size)
    truck_vis.Advance(step_size)
    sedan_vis.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)


truck_state = truck.GetTractor().GetState()
truck_trailer_state = truck.GetTrailer().GetState()
print("TRUCK TRACTOR STATE: ", truck_state)
print("TRUCK TRAILER STATE: ", truck_trailer_state)