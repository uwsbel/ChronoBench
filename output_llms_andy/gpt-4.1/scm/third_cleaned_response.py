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


vis_type = veh.VisualizationType.MESH


chassis_collision_type = veh.ChassisCollisionType_NONE


tire_model = veh.TireModelType.RIGID


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
terrain.SetSoilParameters(2e6, 0, 1.1, 0, 30, 0.01, 2e8, 3e4)
terrain.AddMovingPatch(vehicle.GetChassisBody(), chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(5, 3, 1))
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)
terrain.Initialize(20, 20, 0.02)


system = vehicle.GetSystem()
num_boxes = 10
box_size = chrono.ChVector3d(1, 1, 1)
min_dist_from_vehicle = 4.0  

vehicle_pos = initLoc

for i in range(num_boxes):
    while True:
        x = random.uniform(-terrainLength/2, terrainLength/2)
        y = random.uniform(-terrainWidth/2, terrainWidth/2)
        z = terrainHeight + box_size.z / 2
        
        dist = math.sqrt((x - vehicle_pos.x)**2 + (y - vehicle_pos.y)**2)
        if dist > min_dist_from_vehicle:
            break
    box = chrono.ChBodyEasyBox(box_size.x, box_size.y, box_size.z, 1000, True, True)
    box.SetPos(chrono.ChVector3d(x, y, z))
    box.SetBodyFixed(False)
    box.SetCollide(True)
    box.SetMaterialSurface(chrono.ChMaterialSurfaceSMC())
    
    color = chrono.ChColorAsset()
    color.SetColor(chrono.ChColor(random.random(), random.random(), random.random()))
    box.AddAsset(color)
    system.Add(box)


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





manager = sens.ChSensorManager(system)


light_positions = [
    chrono.ChVector3d(0, 0, 10),
    chrono.ChVector3d(10, 10, 8),
    chrono.ChVector3d(-10, -10, 8),
    chrono.ChVector3d(10, -10, 8),
    chrono.ChVector3d(-10, 10, 8)
]
for pos in light_positions:
    light = sens.ChPointLight(pos, chrono.ChColor(1.0, 1.0, 1.0), 500.0)
    manager.AddSensor(light)


cam_update_rate = 30.0  
cam_width = 640
cam_height = 480
cam_fov = math.radians(60)  


cam_offset = chrono.ChFramed(chrono.ChVector3d(0.5, 0, 1.5))

camera = sens.ChCameraSensor(
    vehicle.GetChassisBody(),    
    cam_update_rate,             
    cam_offset,                  
    cam_width,                   
    cam_height,                  
    cam_fov                      
)
camera.SetName("Camera Sensor")
camera.SetLag(0)
camera.SetCollectionWindow(0)


camera.PushFilter(sens.ChFilterVisualize(cam_width, cam_height, "Camera Sensor"))





manager.AddSensor(camera)





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

    
    manager.Update()

    
    step_number += 1

    
    realtime_timer.Spin(step_size)