import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVectorD(0, 0, 0.5)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_RIGID


terrain_height = 0      
terrain_length = 100.0  
terrain_width = 100.0   


track_point = chrono.ChVectorD(0, 0, 2.1)


contact_method = chrono.ChContactMethod_NSC
contact_vis = False


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 50  


vehicle = veh.Kraz()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type, vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type, vis_type)
vehicle.SetWheelVisualizationType(vis_type, vis_type)
vehicle.SetTireVisualizationType(vis_type, vis_type)

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


terrain_material = chrono.ChContactMaterialNSC()
terrain_material.SetFriction(0.9)
terrain_material.SetRestitution(0.01)
terrain = veh.HighwayTerrain(vehicle.GetSystem())
terrain_mesh = chrono.ChTriangleMeshConnected()
terrain_mesh.LoadWavefrontMesh(chrono.GetChronoDataFile('vehicle/highway.mesh'), True, True)
terrain.SetMesh(terrain_mesh)
terrain.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Kraz Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(track_point, 25.0, 1.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetTractor())


driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0  
throttle_time = 1.0  
braking_time = 0.3   
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()


sedan = veh.Sedan()
sedan.SetContactMethod(contact_method)
sedan.SetChassisCollisionType(chassis_collision_type)
sedan.SetChassisFixed(False)
sedan.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(50, 0, 0.5), initRot))
sedan.Initialize()

sedan.SetChassisVisualizationType(vis_type, vis_type)
sedan.SetSteeringVisualizationType(vis_type)
sedan.SetSuspensionVisualizationType(vis_type, vis_type)
sedan.SetWheelVisualizationType(vis_type, vis_type)
sedan.SetTireVisualizationType(vis_type, vis_type)

sedan.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


sedan_driver = veh.ChInteractiveDriverIRR(vis)
sedan_driver.SetSteeringDelta(render_step_size / steering_time)
sedan_driver.SetThrottleDelta(render_step_size / throttle_time)
sedan_driver.SetBrakingDelta(render_step_size / braking_time)
sedan_driver.Initialize()


sedan_driver.SetSteeringDelta(render_step_size / steering_time)
sedan_driver.SetThrottleDelta(render_step_size / throttle_time)
sedan_driver.SetBrakingDelta(render_step_size / braking_time)


print("VEHICLE MASS: ", vehicle.GetTractor().GetMass())


render_steps = math.ceil(render_step_size / step_size)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    truck_driver_inputs = driver.GetInputs()
    sedan_driver_inputs = sedan_driver.GetInputs()

    
    driver.Synchronize(time)
    sedan_driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, truck_driver_inputs, terrain)
    sedan.Synchronize(time, sedan_driver_inputs, terrain)
    vis.Synchronize(time, truck_driver_inputs)

    
    driver.Advance(step_size)
    sedan_driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    sedan.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)