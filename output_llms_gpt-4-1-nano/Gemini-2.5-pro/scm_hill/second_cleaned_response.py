import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens 
import math
import numpy as np 


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(-15, 0, 1.2)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_RIGID


terrainHeight = 0      
terrainLength = 100.0  
terrainWidth = 100.0   


trackPoint = chrono.ChVector3d(0.0, 0.0, 1.71)


contact_method = chrono.ChContactMethod_SMC
contact_vis = False 


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 50  


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


terrain = veh.SCMTerrain(vehicle.GetSystem())
terrain.SetSoilParameters(2e6,   
                            0,     
                            1.1,   
                            0,     
                            30,    
                            0.01,  
                            2e8,   
                            3e4    
)



terrain.AddMovingPatch(vehicle.GetChassisBody(), chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(5, 3, 1))


terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)



scm_mesh_resolution = 0.1
terrain.Initialize(veh.GetDataFile("terrain/height_maps/bump64.bmp"), 40.0, 40.0, -1.0, 1.0, scm_mesh_resolution)

terrain.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 60.0, 60.0) 


num_obstacles = 5
obstacle_size_xyz = chrono.ChVector3d(0.5, 2.0, 1.0) 
obstacle_density = 400 



obstacle_area_min_x = -10.0
obstacle_area_max_x = 20.0
obstacle_area_min_y = -15.0
obstacle_area_max_y = 15.0

for i in range(num_obstacles):
    rand_x = np.random.uniform(obstacle_area_min_x, obstacle_area_max_x)
    rand_y = np.random.uniform(obstacle_area_min_y, obstacle_area_max_y)
    
    
    base_z = 0.0 + obstacle_size_xyz.z / 2.0
    
    obstacle = chrono.ChBodyEasyBox(obstacle_size_xyz.x, obstacle_size_xyz.y, obstacle_size_xyz.z,
                                    obstacle_density, True, True) 
    obstacle.SetPos(chrono.ChVector3d(rand_x, rand_y, base_z))
    obstacle.SetRot(chrono.QuatFromAngleZ(np.random.uniform(0, math.pi))) 
    obstacle.SetFixed(True) 
    
    
    color = chrono.ChColor(np.random.uniform(0.4,0.7), np.random.uniform(0.4,0.7), np.random.uniform(0.4,0.7))
    if obstacle.GetVisualShape(0):
        obstacle.GetVisualShape(0).SetColor(color)

    vehicle.GetSystem().Add(obstacle)


sensor_manager = sens.ChSensorManager(vehicle.GetSystem())
sensor_manager.SetVerbose(False)





lidar_update_rate = 20  

lidar_offset_pose = chrono.ChFramed(chrono.ChVector3d(0.8, 0, 1.5), 
                                   chrono.QuatFromAngleZ(0)) 
horizontal_fov = 2 * math.pi  
max_vertical_angle = chrono.CH_PI / 6.0  
min_vertical_angle = -chrono.CH_PI / 6.0 

horizontal_resolution = 1080 
vertical_channels = 32     
max_distance = 150.0       
lag = 0                    
collection_window = 0      


lidar = sens.ChLidarSensor(
    vehicle.GetChassisBody(),    
    lidar_update_rate,           
    lidar_offset_pose,           
    horizontal_resolution,       
    vertical_channels,           
    horizontal_fov,              
    max_vertical_angle,          
    min_vertical_angle,          
    max_distance,                
    lag,                         
    collection_window,           
    sens.LidarBeamShape_RECTANGULAR, 
    0.003,                       
    0.003                        
)
lidar.SetName("LidarSensor")


lidar.PushFilter(sens.ChFilterPCfromDepth()) 

lidar_vis_filter = sens.ChFilterVisualizingPointCloud(2.0, 1280, 720, "Lidar Point Cloud") 
lidar.PushFilter(lidar_vis_filter)


sensor_manager.AddSensor(lidar)



vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV SCM Lidar Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5) 
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
    vis.Synchronize(time, driver_inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    sensor_manager.Update()

    
    step_number += 1

    
    realtime_timer.Spin(step_size)

del sensor_manager