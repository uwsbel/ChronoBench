import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math
import numpy as np
import pychrono.sensor as sensors


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(-15, 0, 1.2)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)


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
vehicle.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
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


obstacles = []
np.random.seed(42)  
for i in range(5):
    size = np.random.uniform(0.5, 2.0, size=3)  
    position = np.random.uniform(-20, 20, size=3)
    
    obstacle_shape = chrono.ChBoxShape()
    obstacle_shape.GetBoxGeometry().SetLengths(size)
    
    obstacle_body = chrono.ChBody()
    obstacle_body.SetBodyFixed(True)
    obstacle_body.SetPos(chrono.ChVectorD(*position))
    obstacle_body.GetCollisionModel().ClearModel()
    obstacle_body.GetCollisionModel().AddShape(obstacle_shape)
    obstacle_body.GetCollisionModel().BuildModel()
    obstacle_body.SetCollide(True)
    
    vehicle.GetSystem().Add(obstacle_body)
    obstacles.append(obstacle_body)


sensor_manager = sensors.ChSensorManager(vehicle.GetSystem())


lidar_position = chrono.ChVectorD(0.0, 0.0, 1.5)  
lidar_dir = chrono.ChVectorD(1, 0, 0)  

lidar = sensors.ChLidarSensor(
    body=vehicle.GetChassisBody(),
    position=lidar_position,
    direction=lidar_dir,
    updates_time_step=render_step_size,
    lidar_resolution=chrono.ChVector2D(640, 480),
    lidar_fov=chrono.ChVector2D(math.radians(30), math.radians(20)),
    max_distance=50,
    vertical_fov=math.radians(20),
    name="Lidar_front"
)


lidar.SetPointCloudFilter(sensors.PointCloudFilter_POINT_CLOUD_FILTER_TYPE::POINT_CLOUD_FILTER_TYPE_NO_FILTER)


sensor_manager.AddSensor(lidar)


sensor_manager.Initialize()






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
    sensor_manager.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)