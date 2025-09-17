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


terrain.AddMovingPatch(vehicle.GetChassisBody(), chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(5, 3, 1))


terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)


terrain.Initialize(20, 20, 0.02)


num_boxes = 20
box_size = chrono.ChVector3d(0.5, 0.5, 0.5)  
density = 1000  


safe_radius = 3.0
safe_radius_sq = safe_radius**2

for i in range(num_boxes):
    while True:
        
        x = random.uniform(-40, 40)
        y = random.uniform(-40, 40)
        
        dx = x - initLoc.x
        dy = y - initLoc.y
        if dx*dx + dy*dy >= safe_radius_sq:
            break
    
    z = box_size.z / 2  
    pos = chrono.ChVector3d(x, y, z)
    
    
    box_body = chrono.ChBodyEasyBox(box_size.x, box_size.y, box_size.z, density, True, True)
    box_body.SetPos(pos)
    box_body.SetFixed(False)
    box_body.GetVisualShape(0).SetColor(chrono.ChColor(
        random.random(), 
        random.random(), 
        random.random()
    ))
    vehicle.GetSystem().Add(box_body)


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo with Sensors')
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


manager = sens.ChSensorManager(vehicle.GetSystem())
manager.scene.AddPointLight(chrono.ChVector3d(0, 0, 100), chrono.ChColor(1, 1, 1), 1000.0)


camera_body = vehicle.GetChassisBody()
camera_pos = chrono.ChVector3d(-2, 0, 1)  
camera_rot = chrono.Q_from_AngZ(math.pi) * chrono.Q_from_AngY(-math.pi/2)  
camera_pose = chrono.ChFramed(camera_pos, camera_rot)

camera = sens.ChCameraSensor(
    camera_body,            
    30,                     
    camera_pose,            
    1280,                   
    720,                    
    math.radians(70)        
)
camera.SetName("Vehicle Camera")
camera.SetLag(0)
camera.SetCollectionWindow(0)


camera.PushFilter(sens.ChFilterRGBA8Access())
camera.PushFilter(sens.ChFilterVisualize(1280, 720, "Camera Feed"))
manager.AddSensor(camera)




print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())


render_steps = math.ceil(render_step_size / step_size)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    
    if step_number % render_steps == 0:
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
    
    
    manager.Update()
    
    vis.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)