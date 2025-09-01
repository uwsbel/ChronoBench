import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


system = chrono.ChSystem()


initLoc_truck = chrono.ChVector3d(0, 2, 0.5)  
initRot_truck = chrono.Q_ROTATE_Y_TO_X  
initLoc_sedan = chrono.ChVector3d(-10, 0, 0.5)  
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


vehicle = veh.Kraz(system)  
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc_truck, initRot_truck))
vehicle.SetTireType(tire_model_truck)  
vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)


sedan = veh.Sedan(system)  
sedan.SetContactMethod(contact_method)
sedan.SetChassisCollisionType(chassis_collision_type)
sedan.SetChassisFixed(False)
sedan.SetInitPosition(chrono.ChCoordsysd(initLoc_sedan, initRot_sedan))
sedan.Initialize()

sedan.SetChassisVisualizationType(vis_type)
sedan.SetSteeringVisualizationType(vis_type)
sedan.SetSuspensionVisualizationType(vis_type)
sedan.SetWheelVisualizationType(vis_type)
sedan.SetTireVisualizationType(vis_type)

system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrainHeight), chrono.QUNIT),
    veh.GetDataFile("terrain/meshes/highway.obj"))  

patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Modified Kraz and Sedan Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 25.0, 1.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetTractor())


driver_truck = veh.ChInteractiveDriverIRR(vis)
driver_sedan = veh.ChDriver(sedan.GetVehicle())  


steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver_truck.SetSteeringDelta(render_step_size / steering_time)
driver_truck.SetThrottleDelta(render_step_size / throttle_time)
driver_truck.SetBrakingDelta(render_step_size / braking_time)
driver_truck.Initialize()


driver_sedan.SetThrottle(0.5)  
driver_sedan.SetSteering(0.0)  
driver_sedan.SetBraking(0.0)


render_steps = math.ceil(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0


tractor_states = []
trailer_states = []

while vis.Run():
    time = system.GetChTime()

    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    inputs_truck = driver_truck.GetInputs()
    inputs_sedan = driver_sedan.GetInputs()  

    
    driver_truck.Synchronize(time)
    driver_sedan.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, inputs_truck, terrain)
    sedan.Synchronize(time, inputs_sedan, terrain)  
    vis.Synchronize(time, inputs_truck)

    
    driver_truck.Advance(step_size)
    driver_sedan.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    sedan.Advance(step_size)  
    vis.Advance(step_size)

    
    tractor_states.append(vehicle.GetTractor().GetPos())
    if vehicle.GetNumberTrailers() > 0:  
        trailer_states.append(vehicle.GetTrailer(0).GetPos())

    step_number += 1
    realtime_timer.Spin(step_size)