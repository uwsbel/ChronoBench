import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sensor
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


def add_random_boxes(system, num_boxes=10, vehicle_pos=initLoc, min_distance=5.0):
    for i in range(num_boxes):
        
        while True:
            x = random.uniform(-terrainLength/2, terrainLength/2)
            y = random.uniform(-terrainWidth/2, terrainWidth/2)
            z = random.uniform(0.5, 2.0)

            box_pos = chrono.ChVector3d(x, y, z)
            distance = (box_pos - vehicle_pos).Length()

            if distance > min_distance:
                break

        
        size_x = random.uniform(0.5, 2.0)
        size_y = random.uniform(0.5, 2.0)
        size_z = random.uniform(0.5, 2.0)

        box = chrono.ChBodyEasyBox(size_x, size_y, size_z, 1000, True, True)
        box.SetPos(box_pos)
        box.SetBodyFixed(False)
        box.GetVisualModel().SetColor(chrono.ChColor(random.random(), random.random(), random.random()))
        system.Add(box)

add_random_boxes(vehicle.GetSystem())


driver = veh.ChInteractiveDriverIRR(vis)


steering_time = 1.0  
throttle_time = 1.0  
braking_time = 0.3   
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()


sensor_manager = sensor.ChSensorManager(vehicle.GetSystem())


def add_point_lights(system, num_lights=5):
    for i in range(num_lights):
        x = random.uniform(-terrainLength/2, terrainLength/2)
        y = random.uniform(-terrainWidth/2, terrainWidth/2)
        z = random.uniform(3.0, 8.0)

        light = chrono.ChPointPointLight()
        light.SetPos(chrono.ChVector3d(x, y, z))
        light.SetColor(chrono.ChColor(random.random(), random.random(), random.random()))
        light.SetIntensity(1.0)
        system.Add(light)

add_point_lights(vehicle.GetSystem())


camera = sensor.ChCameraSensor(
    vehicle.GetChassisBody(),  
    10,                        
    True,                      
    "camera_sensor"            
)


camera.SetName("vehicle_camera")
camera.SetOffsetPose(chrono.ChFrameD(chrono.ChVector3d(0, 0, 2.0), chrono.ChQuaternionD(1, 0, 0, 0)))
camera.SetLensModel(sensor.ChLensModelType.PERSPECTIVE)
camera.SetImageWidth(640)
camera.SetImageHeight(480)
camera.SetFocalLength(0.035)  
camera.SetHorizontalFOV(chrono.ChMath.PI / 3)  


filter = sensor.ChFilterVisualize(640, 480, "Camera Feed")
camera.AddFilter(filter)


sensor_manager.AddSensor(camera)






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
    sensor_manager.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)