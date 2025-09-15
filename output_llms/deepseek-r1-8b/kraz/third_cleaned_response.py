import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc_truck = chrono.ChVector3d(0, 0, 0.5)
initRot_truck = chrono.ChQuaterniond(1, 0, 0, 0)


initLoc_sedan = chrono.ChVector3d(-50, 0, 0.5)
initRot_sedan = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model_truck = veh.TireModelType_RIGID
tire_model_sedan = veh.TireModelType_RIGID  


terrain_model = veh.RigidTerrain.Type_HIGHWAY_MESH
terrainHeight = 0      
terrainLength = 1000.0  
terrainWidth = 300.0   


trackPoint = chrono.ChVector3d(0,0, 2.1)


contact_method = chrono.ChContactMethod_NSC
contact_vis = False


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 50  


truck = veh.Kraz()
truck.SetContactMethod(contact_method)
truck.SetChassisCollisionType(chassis_collision_type)
truck.SetChassisFixed(False)
truck.SetInitPosition(chrono.ChCoordsysd(initLoc_truck, initRot_truck))
truck.Initialize()

truck.SetChassisVisualizationType(vis_type, vis_type)
truck.SetSteeringVisualizationType(vis_type)
truck.SetSuspensionVisualizationType(vis_type, vis_type)
truck.SetWheelVisualizationType(vis_type, vis_type)
truck.SetTireVisualizationType(vis_type, vis_type)

truck.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


terrain = veh.RigidTerrain(truck.GetSystem())
terrain.InitializeTerrain(terrain_model, terrainLength, terrainWidth, terrainHeight)


vis_truck = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis_truck.SetWindowTitle('Kraz Truck Demo')
vis_truck.SetWindowSize(1280, 1024)
vis_truck.SetChaseCamera(trackPoint, 25.0, 1.5)
vis_truck.Initialize()
vis_truck.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis_truck.AddLightDirectional()
vis_truck.AddSkyBox()
vis_truck.AttachVehicle(truck.GetTractor())


sedan_vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
sedan_vis.SetWindowTitle('Sedan Demo')
sedan_vis.SetWindowSize(1280, 1024)
sedan_vis.SetChaseCamera(trackPoint, 25.0, 1.5)
sedan_vis.Initialize()
sedan_vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
sedan_vis.AddLightDirectional()
sedan_vis.AddSkyBox()
sedan_vis.AttachVehicle(sedan.GetTractor())


driver_truck = veh.ChInteractiveDriverIRR(vis_truck)


driver_sedan = veh.ChInteractiveDriverIRR(sedan_vis)


steering_time = 1.0  
throttle_time = 1.0  
braking_time = 0.3   
driver_truck.SetSteeringDelta(render_step_size / steering_time)
driver_truck.SetThrottleDelta(render_step_size / throttle_time)
driver_truck.SetBrakingDelta(render_step_size / braking_time)

driver_truck.Initialize()


print("VEHICLE MASS: ", truck.GetTractor().GetMass())


render_steps = math.ceil(render_step_size / step_size)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0


truck_state = None

while vis_truck.Run() and vis_sedan.Run():
    time = truck.GetSystem().GetChTime()

    
    if (step_number % render_steps == 0):
        vis_truck.BeginScene()
        vis_truck.Render()
        vis_truck.EndScene()
        render_frame += 1

    
    driver_inputs = driver_truck.GetInputs()

    
    truck.Synchronize(time)
    terrain.Synchronize(time)
    truck.Advance(step_size)
    vis_truck.Synchronize(time, driver_inputs)

    
    driver_truck.Advance(step_size)
    terrain.Advance(step_size)
    truck.Advance(step_size)
    vis_truck.Advance(step_size)

    
    if not truck.GetTractor().GetState().IsBraking():
        sedan_control = sedan.GetTractor().GetControl()
        sedan_control.SetThrottle(0.5)  
        sedan_control.SetSteering(0.0)  

    
    step_number += 1

    
    realtime_timer.Spin(step_size)

    
    if truck_state is None:
        truck_state = truck.GetTractor().GetState().Copy()


if truck_state is not None:
    truck.GetTractor().GetState().Load(truck_state)