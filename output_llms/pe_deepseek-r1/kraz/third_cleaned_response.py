import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(2, 0, 0.5)  
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


sedan_initLoc = chrono.ChVector3d(-5, 0, 0.5)  
sedan_initRot = chrono.ChQuaterniond(1, 0, 0, 0)


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
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model_truck)  
vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type, vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type, vis_type)
vehicle.SetWheelVisualizationType(vis_type, vis_type)
vehicle.SetTireVisualizationType(vis_type, vis_type)

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


sedan = veh.Sedan()  
sedan.SetContactMethod(contact_method)
sedan.SetChassisCollisionType(chassis_collision_type)
sedan.SetChassisFixed(False)
sedan.SetInitPosition(chrono.ChCoordsysd(sedan_initLoc, sedan_initRot))
sedan.SetTireType(tire_model_sedan)
sedan.Initialize()

sedan.SetChassisVisualizationType(vis_type, vis_type)
sedan.SetSteeringVisualizationType(vis_type)
sedan.SetSuspensionVisualizationType(vis_type, vis_type)
sedan.SetWheelVisualizationType(vis_type, vis_type)
sedan.SetTireVisualizationType(vis_type, vis_type)


patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())


mesh_file = "terrain/meshes/highway.obj"  
patch = terrain.AddMesh(
    patch_mat, 
    veh.GetDataFile(mesh_file),
    chrono.ChVector3d(0, 0, 0),
    chrono.QUNIT
)
terrain.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Kraz and Sedan Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 25.0, 1.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetTractor())


driver_truck = veh.ChInteractiveDriverIRR(vis)  
driver_sedan = veh.ChDriver(sedan.GetVehicle())  


driver_sedan.SetThrottle(0.5)  
driver_sedan.SetSteering(0.0)   
driver_sedan.SetBraking(0.0)    


steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver_truck.SetSteeringDelta(render_step_size / steering_time)
driver_truck.SetThrottleDelta(render_step_size / throttle_time)
driver_truck.SetBrakingDelta(render_step_size / braking_time)

driver_truck.Initialize()


print("TRUCK MASS: ", vehicle.GetTractor().GetMass())
print("SEDAN MASS: ", sedan.GetVehicle().GetMass())


render_steps = math.ceil(render_step_size / step_size)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0


state_file = open("truck_state.csv", "w")
state_file.write("Time,Tractor_X,Tractor_Y,Tractor_Z,Trailer_X,Trailer_Y,Trailer_Z\n")

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    driver_inputs_truck = driver_truck.GetInputs()
    
    
    driver_truck.Synchronize(time)
    driver_sedan.Synchronize(time)  
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs_truck, terrain)
    sedan.Synchronize(time, driver_sedan.GetInputs(), terrain)  
    vis.Synchronize(time, driver_inputs_truck)

    
    tractor_pos = vehicle.GetTractor().GetPos()
    trailer_pos = vehicle.GetTrailer().GetPos()
    state_file.write(f"{time:.4f},{tractor_pos.x:.4f},{tractor_pos.y:.4f},{tractor_pos.z:.4f},")
    state_file.write(f"{trailer_pos.x:.4f},{trailer_pos.y:.4f},{trailer_pos.z:.4f}\n")

    
    driver_truck.Advance(step_size)
    driver_sedan.Advance(step_size)  
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    sedan.Advance(step_size)  
    vis.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)


state_file.close()

mesh_file = "terrain/meshes/highway.obj"
     patch = terrain.AddMesh(patch_mat, veh.GetDataFile(mesh_file), ...)

driver_sedan = veh.ChDriver(sedan.GetVehicle())
     driver_sedan.SetThrottle(0.5)  

state_file = open("truck_state.csv", "w")
     state_file.write("Time,Tractor_X,Tractor_Y,Tractor_Z,Trailer_X,Trailer_Y,Trailer_Z\n")