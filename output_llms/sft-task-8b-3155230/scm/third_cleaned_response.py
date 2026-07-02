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


def add_boxes(system, count=5, min_dist=5.0):
    for _ in range(count):
        while True:
            x = random.uniform(-10, 10)
            y = random.uniform(-5, 5)
            z = 0.5
            pos = chrono.ChVector3d(x, y, z)
            if pos.Dist(initLoc) > min_dist:
                box = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 1000, True, False)
                box.SetName('box')
                box.SetPos(pos)
                box.GetVisualShape(0).SetColor(chrono.ChColor(0.5, 0.5, 0.5))
                box.GetVisualShape(0).SetOpacity(0.7)
                system.Add(box)
                break

add_boxes(vehicle.GetSystem())


terrain = veh.SCMTerrain(vehicle.GetSystem())
terrain.SetSoilParameters(2e6, 0, 1.1, 0, 30, 0.01, 2e8, 3e4)
terrain.AddMovingPatch(vehicle.GetChassisBody(), chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(5, 3, 1))
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)
terrain.Initialize(20, 20, 0.02)


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())


sensor_manager = veh.ChSensorManager(vehicle.GetSystem())
sensor_manager.SetAmbientLight(chrono.ChColor(0.1, 0.1, 0.1))

light1 = chrono.ChLightPoint()
light1.SetPos(chrono.ChVector3d(10, 5, 2))
light1.SetColor(chrono.ChColor(1, 0.8, 0.2))
light1.SetRadius(10)
sensor_manager.AddLight(light1)

light2 = chrono.ChLightPoint()
light2.SetPos(chrono.ChVector3d(-10, -5, 2))
light2.SetColor(chrono.ChColor(0.2, 0.6, 1))
light2.SetRadius(10)
sensor_manager.AddLight(light2)


camera = veh.ChCameraSensor()
camera.SetUpdateRate(30)
camera.SetResolution(640, 480)
camera.SetFOV(60 * chrono.CH_PI / 180)
camera.SetPosition(chrono.ChVector3d(0, 0, 2))
camera.SetDirection(chrono.ChVector3d(0, 0, -1))
camera.SetUp(chrono.ChVector3d(0, 1, 0))
camera.SetSensorType(veh.ChCameraSensor.SENSOR_TYPE_COLOR)
camera.SetSensorFilter(veh.ChCameraSensor.FILTER_TYPE_GRAYSCALE)
camera.AttachToBody(vehicle.GetVehicle().GetChassisBody())
sensor_manager.AddSensor(camera)


filter = veh.ChVisualSystemIrrlichtFilter()
filter.SetResolution(640, 480)
filter.SetPosition(chrono.ChVector3d(12, 0, 2))
filter.SetSize(0.5)
sensor_manager.AddFilter(filter)


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

    
    step_number += 1

    
    realtime_timer.Spin(step_size)