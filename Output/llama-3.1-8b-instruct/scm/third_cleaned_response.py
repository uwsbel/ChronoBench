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
sensor_manager.AddSensor(vis)


camera_sensor = veh.ChCameraSensor()
camera_sensor.SetResolution(640, 480)
camera_sensor.SetFieldOfView(math.radians(60))
camera_sensor.SetPosition(vehicle.GetChassisBody().GetPos())
camera_sensor.SetOrientation(vehicle.GetChassisBody().GetRot())
sensor_manager.AddSensor(camera_sensor)


light1 = veh.ChPointLight()
light1.SetPosition(chrono.ChVector3d(-10, 0, 0))
light1.SetIntensity(10)
sensor_manager.AddSensor(light1)

light2 = veh.ChPointLight()
light2.SetPosition(chrono.ChVector3d(10, 0, 0))
light2.SetIntensity(10)
sensor_manager.AddSensor(light2)

light3 = veh.ChPointLight()
light3.SetPosition(chrono.ChVector3d(0, 10, 0))
light3.SetIntensity(10)
sensor_manager.AddSensor(light3)

light4 = veh.ChPointLight()
light4.SetPosition(chrono.ChVector3d(0, -10, 0))
light4.SetIntensity(10)
sensor_manager.AddSensor(light4)


filter = veh.ChImageFilter()
filter.SetSize(640, 480)
filter.SetFilterType(veh.ImageFilterType_BILINEAR)
camera_sensor.SetFilter(filter)


driver = veh.ChInteractiveDriverIRR(vis)


steering_time = 1.0  
throttle_time = 1.0  
braking_time = 0.3   
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()


for i in range(10):
    box = veh.ChBodyEasyBox()
    box.SetPos(chrono.ChVector3d(random.uniform(-10, 10), random.uniform(-10, 10), random.uniform(0, 1)))
    box.SetSize(chrono.ChVector3d(1, 1, 1))
    box.SetMass(1)
    box.SetCollisionShapeType(chrono.ChCollisionShapeType_BOX)
    vehicle.GetSystem().Add(box)






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

    
    step_number += 1

    
    realtime_timer.Spin(step_size)