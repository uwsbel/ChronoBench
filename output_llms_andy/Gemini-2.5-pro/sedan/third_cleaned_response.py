import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math
import os 






chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')





initLoc = chrono.ChVector3d(5, 1.75, 0.5)  
initRot = chrono.ChQuaterniond(1, 0, 0, 0)  


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE 


tire_model = veh.TireModelType_TMEASY



terrain_mesh_file = veh.GetDataFile("terrain/meshes/StraightMeshed.obj")




trackPoint = chrono.ChVector3d(-5.0, 0.0, 1.8)


contact_method = chrono.ChContactMethod_NSC



step_size = 0.0005  
tire_step_size = step_size  


render_step_size = 1.0 / 100.0  


target_speed_mps = 15.0  



Kp = 0.8  
Ki = 0.3  
Kd = 0.1  


integral_error = 0.0
previous_error = 0.0






vehicle = veh.BMW_E90()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)  
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize() 


vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)


vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)      
patch_mat.SetRestitution(0.01)  

terrain = veh.RigidTerrain(vehicle.GetSystem())


patch_cs = chrono.ChCoordsysd(chrono.ChVector3d(0, 0, -0.1), chrono.QUNIT)
patch = terrain.AddPatch(patch_mat, patch_cs, terrain_mesh_file)



patch.SetColor(chrono.ChColor(0.5, 0.5, 0.5)) 

terrain.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('BMW E90 on Highway - PID Speed Control') 
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5) 
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png')) 
vis.AddLightDirectional() 
vis.AddSkyBox() 
vis.AttachVehicle(vehicle.GetVehicle()) 


driver = veh.ChInteractiveDriverIRR(vis)



steering_time = 5.0  
throttle_time = 1.0  
braking_time = 0.3   


driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time) 
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()





print(f"VEHICLE MASS: {vehicle.GetVehicle().GetMass()} kg")
print(f"Target speed set to: {target_speed_mps:.2f} m/s ({target_speed_mps * 3.6:.2f} km/h)")
print(f"PID Parameters: Kp={Kp}, Ki={Ki}, Kd={Kd}")
print("Controls: Steering (A/D keys), Braking (S key). Throttle is PID controlled.")
print("Close the Irrlicht window or press 'Q' (if bound by Irrlicht) to exit.")



render_steps = math.ceil(render_step_size / step_size)

realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    
    driver_inputs = driver.GetInputs()

    
    current_speed = vehicle.GetVehicle().GetSpeed() 
    error = target_speed_mps - current_speed        

    
    integral_error += error * step_size
    
    
    
    max_integral_contribution = 0.5 
    if Ki != 0: 
        integral_limit = abs(max_integral_contribution / Ki)
        integral_error = max(-integral_limit, min(integral_error, integral_limit))
    
    
    if step_size > 1e-9: 
        derivative_error = (error - previous_error) / step_size
    else:
        derivative_error = 0.0
    previous_error = error 

    
    throttle_signal = Kp * error + Ki * integral_error + Kd * derivative_error
    
    
    
    
    if target_speed_mps < 0.1:  
        applied_throttle = 0.0
    else:
        applied_throttle = max(0.0, min(throttle_signal, 1.0))
    
    
    driver_inputs.m_throttle = applied_throttle
    
    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        
        
        
        
        
        
        
        
        vis.EndScene()
        
        
        


    
    driver.Synchronize(time) 
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain) 
    vis.Synchronize(time, driver_inputs) 

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)
    
    
    step_number += 1

    
    realtime_timer.Spin(step_size)