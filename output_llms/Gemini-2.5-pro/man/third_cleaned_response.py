import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

import pychrono.sensor as sens
import numpy as np
import random




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


terrain_top_surface_z = terrainHeight + 0.05 


trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1) 


contact_method = chrono.ChContactMethod_NSC


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 50  


vehicle = veh.MAN_10t()
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



patch_coordsys = chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrainHeight), chrono.QUNIT)
patch = terrain.AddPatch(patch_mat, patch_coordsys, terrainLength, terrainWidth)


patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200) 

patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()



num_boxes = 20 
box_density = 1000 
box_texture_file = chrono.GetChronoDataFile("textures/concrete.jpg")

for i in range(num_boxes):
    s_x = random.uniform(0.5, 1.5) 
    s_y = random.uniform(0.5, 1.5) 
    s_z = random.uniform(0.5, 2.0) 
    
    
    
    p_x = random.uniform(-terrainLength / 3, terrainLength / 3)
    p_y = random.uniform(-terrainWidth / 3, terrainWidth / 3)
    
    
    if abs(p_x) < 5.0 and abs(p_y) < 5.0:
        p_x = (np.sign(p_x) if p_x != 0 else 1.0) * (abs(p_x) + 5.0)
        p_y = (np.sign(p_y) if p_y != 0 else 1.0) * (abs(p_y) + 5.0)
        
    
    p_z = terrain_top_surface_z + s_z / 2.0 + 0.02 

    box_body = chrono.ChBodyEasyBox(s_x, s_y, s_z,
                                    box_density,    
                                    True,           
                                    True)           
    box_body.SetPos(chrono.ChVector3d(p_x, p_y, p_z))
    
    
    
    
    

    box_body.GetMaterialSurfaceNSC().SetFriction(0.7)
    box_body.GetMaterialSurfaceNSC().SetRestitution(0.1)
    
    
    if box_texture_file:
        box_ch_texture = chrono.ChTexture()
        box_ch_texture.SetTextureFilename(box_texture_file)
        
        vis_shape = box_body.GetVisualShape(0)
        if vis_shape:
             vis_shape.SetTexture(box_ch_texture)

    vehicle.GetSystem().Add(box_body)



sensor_manager = sens.ChSensorManager(vehicle.GetSystem())


sensor_manager.SetVerbose(False)


chassis_body = vehicle.GetChassisBody()


lidar_update_rate = 10.0  




lidar_pos_chassis = chrono.ChVector3d(3.5, 0, 2.0) 
lidar_rot_chassis = chrono.Q_from_AngX(0)         
lidar_offset_pose = chrono.ChFrameD(lidar_pos_chassis, lidar_rot_chassis)

horizontal_samples = 360      
vertical_samples = 32         
horizontal_fov = 2 * math.pi  
max_vert_angle = math.pi / 12.0  
min_vert_angle = -math.pi / 6.0 
max_distance = 100.0          

lidar = sens.ChLidarSensor(
    chassis_body,         
    lidar_update_rate,    
    lidar_offset_pose,    
    horizontal_samples,   
    vertical_samples,     
    horizontal_fov,       
    min_vert_angle,       
    max_vert_angle,       
    max_distance,         
    show_debug_points=True 
)
lidar.SetName("LidarSensor")





sensor_manager.AddSensor(lidar)




vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('MAN 10t Lidar Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 8.0, 0.7) 
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())



driver = veh.ChInteractiveDriverIRR(vis)


steering_time = 1.0  
throttle_time = 1.0  
braking_time = 0.3   
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()


print( "VEHICLE MASS: ",  vehicle.GetVehicle().GetMass())


render_steps = math.ceil(render_step_size / step_size)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0


while vis.Run() :
    time = vehicle.GetSystem().GetChTime()

    
    if (step_number % render_steps == 0) :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    
    
    sensor_manager.Update(time) 
    
    
    vis.Synchronize(time, driver_inputs) 

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size) 

    
    step_number += 1

    
    realtime_timer.Spin(step_size)