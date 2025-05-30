import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math
import os 


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')




initLoc_truck = chrono.ChVector3d(-10, -2, 1.0) 
initRot_truck = chrono.ChQuaterniond(1, 0, 0, 0) 


initLoc_sedan = chrono.ChVector3d(0, 2, 0.5)
initRot_sedan = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH



truck_chassis_collision_type = veh.CollisionType_PRIMITIVES 

sedan_chassis_collision_type = veh.CollisionType_PRIMITIVES


tire_model_truck = veh.TireModelType_RIGID  
tire_model_sedan = veh.TireModelType_TMEASY 



terrain_initial_height = 0.0 


trackPoint = chrono.ChVector3d(0.0, 0.0, 1.75) 


contact_method = chrono.ChContactMethod_NSC
contact_vis = False 


step_size = 2e-3 
tire_step_size = step_size 


render_step_size = 1.0 / 50  


out_dir = "KRAZ_SEDAN_SIMULATION"
if not os.path.exists(out_dir):
    os.makedirs(out_dir)


truck = veh.Kraz()
truck.SetContactMethod(contact_method)
truck.SetChassisCollisionType(truck_chassis_collision_type)
truck.SetChassisFixed(False)
truck.SetInitPosition(chrono.ChCoordsysd(initLoc_truck, initRot_truck))
truck.SetTireType(tire_model_truck) 
truck.Initialize()


truck.SetChassisVisualizationType(vis_type) 
truck.SetSteeringVisualizationType(vis_type) 
truck.SetSuspensionVisualizationType(vis_type, vis_type) 
truck.SetWheelVisualizationType(vis_type, vis_type) 
truck.SetTireVisualizationType(vis_type) 


sedan = veh.Sedan()
sedan.SetContactMethod(contact_method)
sedan.SetChassisCollisionType(sedan_chassis_collision_type)
sedan.SetChassisFixed(False)
sedan.SetInitPosition(chrono.ChCoordsysd(initLoc_sedan, initRot_sedan))
sedan.SetTireType(tire_model_sedan) 
sedan.Initialize()


sedan.SetChassisVisualizationType(vis_type)
sedan.SetSteeringVisualizationType(vis_type)
sedan.SetSuspensionVisualizationType(vis_type, vis_type)
sedan.SetWheelVisualizationType(vis_type, vis_type)
sedan.SetTireVisualizationType(vis_type)


system = truck.GetSystem()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)



patch_mat = chrono.ChMaterialSurfaceNSC() 
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(system)



highway_mesh_file = veh.GetDataFile("terrain/meshes/long_road.obj")
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0,0,terrain_initial_height), chrono.QUNIT), highway_mesh_file, "highway", 0.01) 



patch.GetGroundBody().GetVisualShape(0).SetColor(chrono.ChColor(0.4, 0.4, 0.5)) 

terrain.Initialize()



vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Kraz and Sedan on Highway Demo')
vis.SetWindowSize(1280, 1024)

vis.SetChaseCamera(trackPoint, 12.0, 0.5) 
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(truck.GetVehicle()) 
vis.AttachVehicle(sedan.GetVehicle()) 




driver_truck = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver_truck.SetSteeringDelta(render_step_size / steering_time)
driver_truck.SetThrottleDelta(render_step_size / throttle_time)
driver_truck.SetBrakingDelta(render_step_size / braking_time)
driver_truck.Initialize()


driver_sedan = veh.ChDriver(sedan.GetVehicle())




print("TRUCK MASS: ", truck.GetMass())
print("SEDAN MASS: ", sedan.GetMass())


render_steps = math.ceil(render_step_size / step_size)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0


truck_time_history = []
truck_pos_history = []
truck_rot_history = []
truck_chassis_body = truck.GetChassisBody() 


while vis.Run():
    time = system.GetChTime()

    
    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        
        
        current_pos = truck_chassis_body.GetPos()
        current_rot = truck_chassis_body.GetRot()
        truck_time_history.append(time)
        truck_pos_history.append([current_pos.x, current_pos.y, current_pos.z])
        truck_rot_history.append([current_rot.e0, current_rot.e1, current_rot.e2, current_rot.e3])

    
    driver_inputs_truck = driver_truck.GetInputs()

    
    driver_sedan.SetThrottle(0.5) 
    driver_sedan.SetSteering(0.0) 
    driver_sedan.SetBraking(0.0)   
    driver_inputs_sedan = driver_sedan.GetInputs()


    
    driver_truck.Synchronize(time)
    driver_sedan.Synchronize(time) 
    
    terrain.Synchronize(time)
    
    truck.Synchronize(time, driver_inputs_truck, terrain)
    sedan.Synchronize(time, driver_inputs_sedan, terrain)
    
    vis.Synchronize(time, driver_inputs_truck) 

    
    driver_truck.Advance(step_size)
    driver_sedan.Advance(step_size)
    
    terrain.Advance(step_size)
    
    truck.Advance(step_size)
    sedan.Advance(step_size)
    
    system.DoStepDynamics(step_size) 

    vis.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)




with open(os.path.join(out_dir, "truck_state.csv"), "w") as f:
    f.write("Time,PosX,PosY,PosZ,RotE0,RotE1,RotE2,RotE3\n")
    for i in range(len(truck_time_history)):
        pos = truck_pos_history[i]
        rot = truck_rot_history[i]
        f.write(f"{truck_time_history[i]},{pos[0]},{pos[1]},{pos[2]},{rot[0]},{rot[1]},{rot[2]},{rot[3]}\n")

print(f"Truck state data saved to {os.path.join(out_dir, 'truck_state.csv')}")