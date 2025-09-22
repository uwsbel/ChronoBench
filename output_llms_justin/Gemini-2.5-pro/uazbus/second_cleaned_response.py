import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math
import os 


chrono.SetChronoDataPath(os.environ.get('CHRONO_DATA_DIR', '')) 
veh.SetDataPath(os.path.join(chrono.GetChronoDataPath(), 'vehicle/'))



initLoc = chrono.ChVector3d(-40, 0, 0.5) 
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE 


tire_model = veh.TireModelType_TMEASY


terrainHeight = 0      
terrainLength = 200.0  
terrainWidth = 100.0   


trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1) 


contact_method = chrono.ChContactMethod_NSC
contact_vis = False 


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 50  



vehicle = veh.UAZBUS() 
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
patch = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrainHeight - 0.1), chrono.QUNIT), 
    terrainLength, terrainWidth)



patch.SetTexture(chrono.GetChronoDataFile("terrain/textures/concrete.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('UAZBUS Double Lane Change Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())



driver_inputs = veh.DriverInputs()


print( "VEHICLE MASS: ",  vehicle.GetVehicle().GetMass())


render_steps = math.ceil(render_step_size / step_size)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0


throttle_value = 0.7    

t_accelerate_end = 3.0  


steer_angle_1 = 0.4     
steer_1_start_time = t_accelerate_end
steer_1_peak_time = steer_1_start_time + 0.75
steer_1_end_time = steer_1_peak_time + 0.75 


straight_1_duration = 1.0
straight_1_end_time = steer_1_end_time + straight_1_duration


steer_angle_2 = -0.4    
steer_2_start_time = straight_1_end_time
steer_2_peak_time = steer_2_start_time + 0.75
steer_2_end_time = steer_2_peak_time + 0.75 


straight_2_duration = 1.0
straight_2_end_time = steer_2_end_time + straight_2_duration


t_brake_start = straight_2_end_time
braking_value = 0.8
t_brake_end = t_brake_start + 2.0


t_simulation_end = t_brake_end + 1.0



while vis.Run() :
    time = vehicle.GetSystem().GetChTime()

    
    current_steering = 0.0
    current_throttle = 0.0
    current_braking = 0.0

    if time < t_accelerate_end: 
        current_throttle = throttle_value
    elif time < steer_1_peak_time: 
        current_throttle = throttle_value
        current_steering = steer_angle_1 * (time - steer_1_start_time) / (steer_1_peak_time - steer_1_start_time)
    elif time < steer_1_end_time: 
        current_throttle = throttle_value
        current_steering = steer_angle_1 * (steer_1_end_time - time) / (steer_1_end_time - steer_1_peak_time)
    elif time < straight_1_end_time: 
        current_throttle = throttle_value
        current_steering = 0.0
    elif time < steer_2_peak_time: 
        current_throttle = throttle_value
        current_steering = steer_angle_2 * (time - steer_2_start_time) / (steer_2_peak_time - steer_2_start_time)
    elif time < steer_2_end_time: 
        current_throttle = throttle_value
        current_steering = steer_angle_2 * (steer_2_end_time - time) / (steer_2_end_time - steer_2_peak_time)
    elif time < t_brake_start: 
        current_throttle = throttle_value 
        current_steering = 0.0
    elif time < t_brake_end: 
        current_throttle = 0.0
        current_braking = braking_value
    else: 
        current_throttle = 0.0
        current_braking = 0.1 
        if time > t_simulation_end:
            vis.Quit() 

    
    driver_inputs.m_steering = chrono.ChClamp(current_steering, -1.0, 1.0)
    driver_inputs.m_throttle = chrono.ChClamp(current_throttle, 0.0, 1.0)
    driver_inputs.m_braking = chrono.ChClamp(current_braking, 0.0, 1.0)
    
    
    if (step_number % render_steps == 0) :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    
    
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs) 

    
    
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1
    
    realtime_timer.Spin(step_size)