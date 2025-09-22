import pychrono.core as chrono
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc_truck = chrono.ChVector3d(0, 0, 0.5)
initRot_truck = chrono.ChQuaterniond(1, 0, 0, 0)
initLoc_sedan = chrono.ChVector3d(5, 0, 0)
initRot_sedan = chrono.ChQuaterniond(0, 0, 1, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type_truck = veh.CollisionType_NONE
chassis_collision_type_sedan = veh.CollisionType_NONE


tire_model_truck = veh.TireModelType_RIGID
tire_model_sedan = veh.TireModelType_TMEASY



terrainHeight = 0      
terrainLength = 100.0  
terrainWidth = 100.0   


trackPoint = chrono.ChVector3d(0,0, 2.1)


contact_method = chrono.ChContactMethod_NSC
contact_vis = False


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 50  



truck = veh.Kraz()
truck.SetContactMethod(contact_method)
truck.SetChassisCollisionType(chassis_collision_type_truck)
truck.SetChassisFixed(False)
truck.SetInitPosition(chrono.ChCoordsysd(initLoc_truck, initRot_truck))
truck.Initialize()
truck.SetChassisVisualizationType(vis_type, vis_type)
truck.SetSteeringVisualizationType(vis_type, vis_type)
truck.SetSuspensionVisualizationType(vis_type, vis_type)
truck.SetWheelVisualizationType(vis_type, vis_type)
truck.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)



sedan = veh.Kraz()
sedan.SetContactMethod(contact_method)
sedan.SetChassisCollisionType(chassis_collision_type_sedan)
sedan.SetChassisFixed(False)
sedan.SetInitPosition(chrono.ChCoordsysd(initLoc_sedan, initRot_sedan))
sedan.Initialize()
sedan.SetChassisVisualizationType(vis_type, vis_type)
sedan.SetSteeringVisualizationType(vis_type, vis_type)
sedan.SetSuspensionVisualizationType(vis_type, vis_type)
sedan.SetWheelVisualizationType(vis_type, vis_type)
sedan.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(truck.GetSystem())
patch = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 
    terrainLength, terrainWidth)

patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


vis_truck = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis_sedan = veh.ChWheeledVehicleVisualSystemIrrlicht()

vis_truck.SetWindowTitle('Kraz Truck Demo')
vis_sedan.SetWindowTitle('Kraz Sedan Demo')
vis_truck.SetWindowSize(1280, 1024)
vis_sedan.SetWindowSize(1280, 1024)
vis_truck.SetChaseCamera(trackPoint, 25.0, 1.5)
vis_sedan.SetChaseCamera(trackPoint, 25.0, 1.5)
vis_truck.Initialize()
vis_sedan.Initialize()
vis_truck.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis_sedan.AddLightDirectional()
vis_truck.AddSkyBox()
vis_sedan.AddSkyBox()
vis_truck.AttachVehicle(truck.GetTractor())
vis_sedan.AttachVehicle(sedan.GetTractor())


driver_truck = veh.ChInteractiveDriverIRR(vis_truck)
driver_sedan = veh.ChInteractiveDriverIRR(vis_sedan)


steering_time = 1.0  
throttle_time = 1.0  
braking_time = 0.3   
driver_truck.SetSteeringDelta(render_step_size / steering_time)
driver_truck.SetThrottleDelta(render_step_size / throttle_time)
driver_truck.SetBrakingDelta(render_step_size / braking_time)
driver_sedan.SetSteeringDelta(render_step_size / steering_time)
driver_sedan.SetThrottleDelta(render_step_size / throttle_time)
driver_sedan.SetBrakingDelta(render_step_size / braking_time)

driver_truck.Initialize()
driver_sedan.Initialize()


print(f"Truck MASS: {truck.GetTractor().GetMass()}")
print(f"Sedan MASS: {sedan.GetTractor().GetMass()}")


render_steps = math.ceil(render_step_size / step_size)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis_truck.Run() :
    time = truck.GetSystem().GetChTime()

    
    if (step_number % render_steps == 0) :
        vis_truck.BeginScene()
        vis_truck.Render()
        vis_truck.EndScene()
        render_frame += 1

    
    driver_inputs_truck = driver_truck.GetInputs()

    
    driver_truck.Synchronize(time)
    terrain.Synchronize(time)
    truck.Synchronize(time, driver_inputs_truck, terrain)
    vis_truck.Synchronize(time, driver_inputs_truck)

    
    driver_truck.Advance(step_size)
    terrain.Advance(step_size)
    truck.Advance(step_size)
    vis_truck.Advance(step_size)

    
    realtime_timer.Spin(step_size)

    
    if (step_number % render_steps == 0) :
        vis_sedan.BeginScene()
        vis_sedan.Render()
        render_frame += 1

    
    driver_inputs_sedan = driver_sedan.GetInputs()

    
    driver_sedan.Synchronize(time)
    terrain.Synchronize(time)
    sedan.Synchronize(time, driver_inputs_sedan, terrain)
    vis_sedan.Synchronize(time, driver_inputs_sedan)

    
    driver_sedan.Advance(step_size)
    terrain.Advance(step_size)
    sedan.Advance(step_size)
    vis_sedan.Advance(step_size)

    
    realtime_timer.Spin(step_size)

    
    step_number += 1