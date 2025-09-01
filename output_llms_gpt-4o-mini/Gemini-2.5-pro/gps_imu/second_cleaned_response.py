import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math
import os 




chrono.SetChronoDataPath(chrono.GetChronoDataPath()) 
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_TMEASY


terrainHeight = 0      
terrainLength = 100.0  
terrainWidth = 100.0   


trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)


contact_method = chrono.ChContactMethod_NSC
contact_vis = False 


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 50  


log_step_size = 0.1  


vehicle = veh.HMMWV_Full() 
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
                         chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrainHeight - 0.05), chrono.QUNIT), 
                         terrainLength, terrainWidth, 0.1) 
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo with Programmatic Driver and GPS Logging')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())












driver_inputs = veh.DriverInputs() 


manager = sens.ChSensorManager(vehicle.GetSystem())




offset_pose = chrono.ChFrameD(chrono.ChVector3d(-8, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
imu = sens.ChAccelerometerSensor(vehicle.GetChassisBody(), 
                                 100,        
                                 offset_pose,          
                                 sens.ChNoiseNone())   
imu.SetName("Accelerometer Sensor") 
imu.SetLag(0)
imu.SetCollectionWindow(0)

imu.PushFilter(sens.ChFilterAccelAccess()) 

manager.AddSensor(imu)


gps_update_rate = 10 
gps = sens.ChGPSSensor(vehicle.GetChassisBody(),                     
                       gps_update_rate,        
                       offset_pose,          
                       chrono.ChVector3d(-89.400, 43.070, 260.0),  
                       sens.ChNoiseNone())   
gps.SetName("GPS Sensor")
gps.SetLag(0)
gps.SetCollectionWindow(0) 

gps.PushFilter(sens.ChFilterGPSAccess()) 

manager.AddSensor(gps)






print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())


render_steps = math.ceil(render_step_size / step_size)

log_steps = math.ceil(log_step_size / step_size)


gps_data = []


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0



while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        

    
    
    if time < 6.0:
        driver_inputs.m_throttle = 0.5  
        driver_inputs.m_steering = 0.2  
        driver_inputs.m_braking = 0.0   
    else:
        driver_inputs.m_throttle = 0.0   
        driver_inputs.m_steering = 0.0   
        driver_inputs.m_braking = 1.0    

    
    
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain) 
    vis.Synchronize(time, driver_inputs) 

    
    
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size) 

    
    manager.Update()

    
    if step_number % log_steps == 0:
        gps_buffer = gps.GetMostRecentGPSBuffer() 
        if gps_buffer and gps_buffer.HasData(): 
            
            current_gps_coords_list = gps_buffer.GetGPSData()
            if current_gps_coords_list: 
                
                gps_data.append(current_gps_coords_list[0])
                
                


    
    step_number += 1

    
    realtime_timer.Spin(step_size)


print("\nSimulation Ended.")
print(f"Collected {len(gps_data)} GPS data points.")

print("GPS Data (Lat, Long, Alt):")
for i, data_point in enumerate(gps_data):
    print(f"  Point {i}: Lat={data_point.x:.6f}, Long={data_point.y:.6f}, Alt={data_point.z:.2f}")