import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

print(veh) 




chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')






initLoc1 = chrono.ChVector3d(0, -2, 0.5) 
initRot1 = chrono.ChQuaterniond(1, 0, 0, 0)

initLoc2 = chrono.ChVector3d(0, 2, 0.5)  
initRot2 = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE 


tire_model = veh.TireModelType_TMEASY


terrainHeight = 0      
terrainLength = 100.0  
terrainWidth = 100.0   


trackPoint = chrono.ChVector3d(-5.0, 0.0, 1.8)


contact_method = chrono.ChContactMethod_NSC



step_size = 1e-3
tire_step_size = step_size 


render_step_size = 1.0 / 50  


steer_amplitude = 0.6  
steer_frequency = 0.3  
throttle_value = 0.3   
braking_value = 0.0    






if contact_method == chrono.ChContactMethod_NSC:
    my_system = chrono.ChSystemNSC()
elif contact_method == chrono.ChContactMethod_SMC:
    my_system = chrono.ChSystemSMC()
    
else:
    print("Error: Unknown contact method.")
    exit(1)

my_system.Set_G_acc(chrono.ChVector3d(0, 0, -9.81)) 
my_system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET) 






vehicle1 = veh.Sedan(system=my_system,
                     fixed=False,
                     tire_model=tire_model,
                     contact_method=contact_method, 
                     chassis_collision_type=chassis_collision_type)

vehicle1.SetInitPosition(chrono.ChCoordsysd(initLoc1, initRot1))
vehicle1.SetTireStepSize(tire_step_size)
vehicle1.Initialize()

vehicle1.SetChassisVisualizationType(vis_type)
vehicle1.SetSuspensionVisualizationType(vis_type)
vehicle1.SetSteeringVisualizationType(vis_type)
vehicle1.SetWheelVisualizationType(vis_type)
vehicle1.SetTireVisualizationType(vis_type)




vehicle2 = veh.Sedan(system=my_system,
                     fixed=False,
                     tire_model=tire_model,
                     contact_method=contact_method,
                     chassis_collision_type=chassis_collision_type)

vehicle2.SetInitPosition(chrono.ChCoordsysd(initLoc2, initRot2))
vehicle2.SetTireStepSize(tire_step_size)
vehicle2.Initialize()

vehicle2.SetChassisVisualizationType(vis_type)
vehicle2.SetSuspensionVisualizationType(vis_type)
vehicle2.SetSteeringVisualizationType(vis_type)
vehicle2.SetWheelVisualizationType(vis_type)
vehicle2.SetTireVisualizationType(vis_type)






patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(my_system) 
patch = terrain.AddPatch(patch_mat,
                         chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrainHeight - 0.1), chrono.QUNIT), 
                         terrainLength, terrainWidth, 0.2) 


patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()




vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Sedan Demo - Dual Vehicle Sinusoidal Steering')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5) 
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()


vis.AttachVehicle(vehicle1.GetVehicle())
vis.AttachVehicle(vehicle2.GetVehicle())







print("VEHICLE 1 MASS: ", vehicle1.GetVehicle().GetMass())
print("VEHICLE 2 MASS: ", vehicle2.GetVehicle().GetMass())


render_steps = math.ceil(render_step_size / step_size)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0


driver_inputs_veh1 = veh.DriverInputs()
driver_inputs_veh2 = veh.DriverInputs()

while vis.Run():
    time = my_system.GetChTime()

    
    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    
    
    driver_inputs_veh1.m_steering = steer_amplitude * math.sin(2 * math.pi * steer_frequency * time)
    driver_inputs_veh1.m_throttle = throttle_value
    driver_inputs_veh1.m_braking = braking_value

    
    driver_inputs_veh2.m_steering = steer_amplitude * math.sin(2 * math.pi * steer_frequency * time + math.pi) 
    
    driver_inputs_veh2.m_throttle = throttle_value
    driver_inputs_veh2.m_braking = braking_value

    
    terrain.Synchronize(time) 
    vehicle1.Synchronize(time, driver_inputs_veh1, terrain) 
    vehicle2.Synchronize(time, driver_inputs_veh2, terrain) 
    vis.Synchronize(time, driver_inputs_veh1) 

    
    
    vehicle1.GetVehicle().Advance(step_size)  
    vehicle2.GetVehicle().Advance(step_size)  
    terrain.Advance(step_size)                
    my_system.DoStepDynamics(step_size)       
    vis.Advance(step_size)                    

    
    step_number += 1

    
    realtime_timer.Spin(step_size)

del my_system