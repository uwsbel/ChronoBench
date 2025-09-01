import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math
import random

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(-8, 0, 0.6)
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


terrain.Initialize(20, 20, 0.02)


system = vehicle.GetSystem()
box_size = 1.0
for _ in range(10):
    x = random.uniform(-terrainLength / 2, terrainLength / 2)
    y = random.uniform(-terrainWidth / 2, terrainWidth / 2)
    while math.hypot(x - initLoc.x, y - initLoc.y) < 5:  
        x = random.uniform(-terrainLength / 2, terrainLength / 2)
        y = random.uniform(-terrainWidth / 2, terrainWidth / 2)
    body = chrono.ChBodyEasyBox(box_size, box_size, box_size, 1000, True, True)
    body.SetPos(chrono.ChVector3d(x, y, terrainHeight + box_size / 2))
    system.Add(body)


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())


sensor_manager = veh.ChSensorManager()
for i in range(5):
    light_pos = chrono.ChVector3d(random.uniform(-terrainLength / 2, terrainLength / 2),
                                 random.uniform(-terrainWidth / 2, terrainWidth / 2),
                                 terrainHeight + 2)
    point_light = chrono.ChLightPoint(light_pos, chrono.ChColor(1, 1, 1))
    sensor_manager.AddPointLight(point_light)


camera_sensor = chrono.ChCameraSensor()
camera_sensor.SetResolution(640, 480)
camera_sensor.SetFieldOfView(90)
camera_sensor.SetPos(chrono.ChVector3d(0, 0, 1.71))
camera_sensor.SetDir(chrono.ChVector3d(0, 0, -1))
sensor_manager.AddCamera(camera_sensor)


filter = veh.ChSensorFilter()
filter.SetCameraSensor(camera_sensor)
filter.SetWindowTitle('Camera Feed')
filter.SetWindowSize(640, 480)
filter.Initialize()


driver = veh.ChInteractiveDriverIRR(vis)


steering_time = 1.0  
throttle_time = 1.0  
braking_time = 0.3   
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()






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
        filter.Render()  
        vis.EndScene()
        render_frame += 1

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    sensor_manager.Synchronize(time)  
    vis.Synchronize(time, driver_inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    sensor_manager.Advance(step_size)  
    vis.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)