import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sensor
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


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo')
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




num_obstacles = 5
for i in range(num_obstacles):
    obstacle_pos = chrono.ChVector3d(np.random.uniform(-10, 10), np.random.uniform(-10, 10), 0.5)
    obstacle_size = chrono.ChVector3d(1, 1, 1)
    obstacle = chrono.ChBody()
    obstacle.SetPos(obstacle_pos)
    obstacle.SetMass(100)
    obstacle.SetBodyFixed(True)
    obstacle.GetCollisionModel().ClearModel()
    obstacle.GetCollisionModel().AddBox(obstacle_size.x, obstacle_size.y, obstacle_size.z)
    obstacle.GetCollisionModel().BuildModel()
    vehicle.GetSystem().Add(obstacle)




sensor_manager = sensor.ChSensorManager(vehicle.GetSystem())

lidar = sensor.ChLidarSensor(
    vehicle.GetChassisBody(),                    
    20,                                          
    chrono.ChFrameD(chrono.ChVectorD(0, 0, 1.8), chrono.Q_from_AngZ(90 * chrono.CH_C_DEG_TO_RAD)))  

lidar.SetFov(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1), chrono.Q_from_AngY(90 * chrono.CH_C_DEG_TO_RAD)), 120 * chrono.CH_C_DEG_TO_RAD, 180 * chrono.CH_C_DEG_TO_RAD)  
lidar.PushFilter(sensor.ChFilterVisualize(1200, 800))
lidar.PushFilter(sensor.ChFilterSave("lidar_data.csv"))

sensor_manager.AddSensor(lidar)






print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())


render_steps = math.ceil(render_step_size / step_size)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    
    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    sensor_manager.Update()

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)