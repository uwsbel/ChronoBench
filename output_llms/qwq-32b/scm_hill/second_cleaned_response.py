import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import numpy as np
import pychrono.sensor as sens
import math


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVectorD(-15, 0, 1.2)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_RIGID


terrainHeight = 0      
terrainLength = 100.0  
terrainWidth = 100.0   


trackPoint = chrono.ChVectorD(0.0, 0.0, 1.71)


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


obstacle_size = chrono.ChVectorD(1.0, 1.0, 1.0)
obstacle_mass = 100.0
for _ in range(5):
    x = np.random.uniform(-terrainLength/2 + 2, terrainLength/2 - 2)
    y = np.random.uniform(-terrainWidth/2 + 2, terrainWidth/2 - 2)
    z = terrainHeight + obstacle_size.z() / 2
    body = chrono.ChBodyEasyBox(
        obstacle_size.x(), obstacle_size.y(), obstacle_size.z(),
        obstacle_mass, True, True)
    body.SetPos(chrono.ChVectorD(x, y, z))
    vehicle.GetSystem().Add(body)


terrain = veh.SCMTerrain(vehicle.GetSystem())
terrain.SetSoilParameters(
    2e6,    
    0,      
    1.1,    
    0,      
    30,     
    0.01,   
    2e8,    
    3e4     
)
terrain.AddMovingPatch(vehicle.GetChassisBody(), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(5, 3, 1))
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)


nx, ny = 40, 40
dx = terrainLength / (nx - 1)
dy = terrainWidth / (ny - 1)
terrain.Initialize(
    veh.GetDataFile("terrain/height_maps/bump64.bmp"),
    nx, ny,
    -terrainLength/2, -terrainWidth/2, terrainHeight,
    dx, dy
)
terrain.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)


sensor_manager = sens.ChSensorManager(vehicle.GetSystem())
lidar = sens.ChLidar()
lidar.SetName("lidar")
lidar.SetPosition(chrono.ChVectorD(0, 0, 1.5))
lidar.SetDirection(chrono.ChVectorD(0, 0, 1))
lidar.SetRange(50.0)
lidar.SetHorizontalResolution(0.5)
lidar.SetVerticalResolution(0.5)
lidar.SetFovHorizontal(180)
lidar.SetFovVertical(30)
lidar.SetSamplingPeriod(0.1)

point_cloud = sens.ChPointcloudVisualizer()
point_cloud.SetName("point_cloud")
lidar.AddFilter(point_cloud)
sensor_manager.AddSensor(lidar)


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


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    if step_number % math.ceil(render_step_size / step_size) == 0:
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

    sensor_manager.Update()  

    step_number += 1
    realtime_timer.Spin(step_size)

import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import numpy as np
import pychrono.sensor as sens
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

initLoc = chrono.ChVectorD(-15, 0, 1.2)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)

vis_type = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_NONE
tire_model = veh.TireModelType_RIGID

terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0

trackPoint = chrono.ChVectorD(0.0, 0.0, 1.71)
contact_method = chrono.ChContactMethod_SMC
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


obstacle_size = chrono.ChVectorD(1.0, 1.0, 1.0)
obstacle_mass = 100.0
for _ in range(5):
    x = np.random.uniform(-terrainLength/2 + 2, terrainLength/2 - 2)
    y = np.random.uniform(-terrainWidth/2 + 2, terrainWidth/2 - 2)
    z = terrainHeight + obstacle_size.z() / 2
    body = chrono.ChBodyEasyBox(
        obstacle_size.x(), obstacle_size.y(), obstacle_size.z(),
        obstacle_mass, True, True)
    body.SetPos(chrono.ChVectorD(x, y, z))
    vehicle.GetSystem().Add(body)

terrain = veh.SCMTerrain(vehicle.GetSystem())
terrain.SetSoilParameters(
    2e6, 0, 1.1, 0, 30, 0.01, 2e8, 3e4
)
terrain.AddMovingPatch(vehicle.GetChassisBody(), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(5, 3, 1))
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)

nx, ny = 40, 40
dx = terrainLength / (nx - 1)
dy = terrainWidth / (ny - 1)
terrain.Initialize(
    veh.GetDataFile("terrain/height_maps/bump64.bmp"),
    nx, ny,
    -terrainLength/2, -terrainWidth/2, terrainHeight,
    dx, dy
)
terrain.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)

sensor_manager = sens.ChSensorManager(vehicle.GetSystem())
lidar = sens.ChLidar()
lidar.SetName("lidar")
lidar.SetPosition(chrono.ChVectorD(0, 0, 1.5))
lidar.SetDirection(chrono.ChVectorD(0, 0, 1))
lidar.SetRange(50.0)
lidar.SetHorizontalResolution(0.5)
lidar.SetVerticalResolution(0.5)
lidar.SetFovHorizontal(180)
lidar.SetFovVertical(30)
lidar.SetSamplingPeriod(0.1)

point_cloud = sens.ChPointcloudVisualizer()
point_cloud.SetName("point_cloud")
lidar.AddFilter(point_cloud)
sensor_manager.AddSensor(lidar)

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

realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    if step_number % math.ceil(render_step_size / step_size) == 0:
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

    sensor_manager.Update()  

    step_number += 1
    realtime_timer.Spin(step_size)