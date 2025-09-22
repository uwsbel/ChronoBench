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


terrain.AddMovingPatch(vehicle.GetChassisBody(),
                       chrono.ChVector3d(0, 0, 0),
                       chrono.ChVector3d(5, 3, 1))


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


random.seed(42)


num_boxes = 20  
boxes = []

for _ in range(num_boxes):
    
    while True:
        x = random.uniform(-terrainLength/2 + 5, terrainLength/2 - 5)
        y = random.uniform(-terrainWidth/2 + 5, terrainWidth/2 - 5)
        
        if abs(x - initLoc.x) > 2 and abs(y - initLoc.y) > 2:
            break
    size_x = random.uniform(0.2, 1.0)
    size_y = random.uniform(0.2, 1.0)
    size_z = random.uniform(0.2, 1.0)
    box = chrono.ChBodyEasyBox(size_x, size_y, size_z,  
                                100,  
                                True,  
                                True)  
    box.SetPosition(chrono.ChVector3d(x, y, terrainHeight + size_z/2))
    
    
    
    vehicle.GetSystem().Add(box)
    boxes.append(box)




import pychrono.sensor as sensitivity


sensor_manager = sensitivity.ChSensorManager()
sensor_manager_scene = sensor_manager.GetScene()
sensor_manager.SetScene(sensor_manager_scene)


for i in range(3):
    light_pos = chrono.ChVector3d(
        random.uniform(-10, 10),
        random.uniform(-10, 10),
        10
    )
    
    
    
    vis_scene_mgr = vis.GetSceneManager()
    light = vis_scene_mgr.AddLightSceneNode(
        None,  
        chrono.ChVector3d(light_pos.x, light_pos.y, light_pos.z),
        chrono.ChColor(1, 1, 1),  
        10.0  
    )



camera = sensitivity.ChCameraSensor(
    vehicle.GetChassisBody(),  
    30,  
    chrono.ChFrameFct(
        chrono.ChVectorF(0.0, 0.0, 1.5),  
        chrono.ChQuaternionF(1, 0, 0, 0)  
    )
)
camera.SetName("FrontCamera")
camera.SetResolution(640, 480)
camera.SetFocalLength(35)  
camera.SetFieldOfView(45.0)  
sensor_manager.AddSensor(camera)




def visualize_camera_feed(sensor):
    
    
    pass







print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())


step_number = 0
render_frame = 0
realtime_timer = chrono.ChRealtimeStepTimer()

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
    
    
    sensor_manager.Process()
    sensor_manager.Update()
    sensor_manager.Render()
    
    

    vis.Advance(step_size)

    step_number += 1

    
    realtime_timer.Spin(step_size)