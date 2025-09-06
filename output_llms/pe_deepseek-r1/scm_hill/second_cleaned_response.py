import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens  
import numpy as np  
import math


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


render_step_size = 1.0 / 20  


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


terrain.Initialize(veh.GetDataFile("terrain/height_maps/bump64.bmp"), 40, 40, -1, 1, 0.02)

terrain.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)




obstacle_material = chrono.ChContactMaterialSMC()
obstacle_material.SetFriction(0.9)

for i in range(5):
    
    x = np.random.uniform(-terrainLength/2 + 5, terrainLength/2 - 5)
    y = np.random.uniform(-terrainWidth/2 + 5, terrainWidth/2 - 5)
    z = 0.5  
    
    
    obstacle = chrono.ChBody()
    obstacle.SetPos(chrono.ChVector3d(x, y, z))
    obstacle.SetMass(200)
    obstacle.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
    
    
    obstacle.EnableCollision(True)
    obstacle.GetCollisionModel().ClearModel()
    obstacle.GetCollisionModel().AddBox(obstacle_material, 1.0, 1.0, 0.5)  
    obstacle.GetCollisionModel().BuildModel()
    
    
    box_shape = chrono.ChVisualShapeBox(1.0, 1.0, 0.5)
    box_shape.SetColor(chrono.ChColor(0.8, 0.2, 0.2))  
    obstacle.AddVisualShape(box_shape)
    
    
    vehicle.GetSystem().Add(obstacle)


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo with Lidar')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())




manager = sens.ChSensorManager(vehicle.GetSystem())
manager.scene.AddPointLight(chrono.ChVector3d(0, 0, 100), chrono.ChColor(1, 1, 1), 500)


lidar_offset_pose = chrono.ChFramed(chrono.ChVector3d(0, 0, 1.75), 
                                    chrono.Q_from_AngAxis(0, chrono.ChVector3d(0, 1, 0)))
lidar_h_samples = 2250
lidar_v_samples = 32
lidar_hfov = 2 * chrono.CH_PI  
lidar_vfov = chrono.CH_PI / 6  
lidar_max_dist = 100
lidar_lag = 0
lidar_update_rate = 10


lidar = sens.ChLidarSensor(
    vehicle.GetChassisBody(),     
    lidar_update_rate,            
    lidar_offset_pose,            
    lidar_h_samples,              
    lidar_v_samples,              
    lidar_hfov,                   
    lidar_vfov / 2,               
    -lidar_vfov / 2,              
    lidar_max_dist,               
    sens.LidarBeamShape_RECTANGULAR,
    0.003,                        
    0.003                         
)


lidar.SetName("Vehicle Lidar")
lidar.SetLag(lidar_lag)
lidar.SetCollectionWindow(1/lidar_update_rate)


lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterVisualize(lidar_h_samples, lidar_v_samples, "Lidar Depth Map"))


manager.AddSensor(lidar)


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
render_frame = 0

while vis.Run() :
    time = vehicle.GetSystem().GetChTime()

    
    if (step_number % render_steps == 0) :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)
    
    
    
    
    manager.Update()

    
    step_number += 1

    
    realtime_timer.Spin(step_size)

import pychrono.sensor as sens  
import numpy as np  

manager.Update()