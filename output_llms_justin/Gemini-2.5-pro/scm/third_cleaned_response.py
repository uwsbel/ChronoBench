import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens 
import math
import random 








chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(-8, 0, 0.6)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE





tire_model = veh.TireModelType_RIGID


trackPoint = chrono.ChVector3d(0.0, 0.0, 1.71)


contact_method = chrono.ChContactMethod_SMC


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


my_system = vehicle.GetSystem()
my_system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


terrain = veh.SCMTerrain(my_system)
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





terrain.Initialize(20, 20, 0.02)


num_boxes = 15
box_density = 1000  
box_min_size = 0.3
box_max_size = 0.7


box_material = chrono.ChMaterialSurfaceSMC()
box_material.SetYoungModulus(2e5) 
box_material.SetPoissonRatio(0.3)
box_material.SetStaticFriction(0.6)
box_material.SetSlidingFriction(0.5)
box_material.SetRestitution(0.1)
box_material.SetAdhesion(0) 


veh_length_approx = 4.6  
veh_width_approx = 2.2   
spawn_buffer = 2.0       

no_spawn_x_min = initLoc.x - veh_length_approx / 2 - spawn_buffer
no_spawn_x_max = initLoc.x + veh_length_approx / 2 + spawn_buffer
no_spawn_y_min = initLoc.y - veh_width_approx / 2 - spawn_buffer
no_spawn_y_max = initLoc.y + veh_width_approx / 2 + spawn_buffer


terrain_x_min, terrain_x_max = -9.5, 9.5 
terrain_y_min, terrain_y_max = -9.5, 9.5 

for i in range(num_boxes):
    while True:
        
        size_x = random.uniform(box_min_size, box_max_size)
        size_y = random.uniform(box_min_size, box_max_size)
        size_z = random.uniform(box_min_size, box_max_size)
        
        
        pos_x = random.uniform(terrain_x_min, terrain_x_max)
        pos_y = random.uniform(terrain_y_min, terrain_y_max)
        
        
        if not (no_spawn_x_min < pos_x < no_spawn_x_max and \
                no_spawn_y_min < pos_y < no_spawn_y_max):
            break 

    
    pos_z = size_z / 2.0 
    
    box_body = chrono.ChBodyEasyBox(size_x, size_y, size_z,
                                    box_density,
                                    box_material, 
                                    chrono.ChVisualMaterial.Default()) 
    box_body.SetCoords(chrono.ChCoordsysd(chrono.ChVector3d(pos_x, pos_y, pos_z), chrono.QUNIT))
    
    
    color_asset = chrono.ChColorAsset()
    color_asset.SetColor(chrono.ChColor(random.random(), random.random(), random.random())) 
    box_body.AddAsset(color_asset)
    
    my_system.Add(box_body)


sensor_manager = sens.ChSensorManager(my_system)
sensor_manager.SetVerbose(False) 


light_color_vec = chrono.ChVector3f(1.0, 1.0, 1.0) 
light_max_range = 50.0  
light_intensity_val = 1.5 


point_light1 = sens.ChPointLight(
    chrono.ChVector3d(8, 8, 5),    
    light_color_vec,               
    light_max_range                
)
point_light1.SetName("PointLightSensor1")
point_light1.SetIntensity(light_intensity_val) 
sensor_manager.AddSensor(point_light1)


point_light2 = sens.ChPointLight(
    chrono.ChVector3d(-8, -8, 5),  
    light_color_vec,               
    light_max_range
)
point_light2.SetName("PointLightSensor2")
point_light2.SetIntensity(light_intensity_val) 
sensor_manager.AddSensor(point_light2)


camera_update_rate = 30 
image_width = 1280
image_height = 720
hfov = math.pi / 2.8  



camera_offset_pos = chrono.ChVector3d(0.8, 0, 0.9) 
camera_offset_rot = chrono.QUNIT 

camera = sens.ChCameraSensor(
    vehicle.GetChassisBody(),  
    camera_update_rate,        
    chrono.ChFramed(camera_offset_pos, camera_offset_rot), 
    image_width,               
    image_height,              
    hfov                       
)
camera.SetName("VehicleChassisCamera")


camera_vis_filter = sens.ChFilterVisualize(image_width, image_height, "Camera Feed")
camera.PushFilter(camera_vis_filter)

sensor_manager.AddSensor(camera)



vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV SCM Demo with Sensors and Objects')
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
    time = my_system.GetChTime()

    
    if (step_number % render_steps == 0) :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs) 

    
    sensor_manager.Update(time)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size) 

    
    step_number += 1

    
    realtime_timer.Spin(step_size)