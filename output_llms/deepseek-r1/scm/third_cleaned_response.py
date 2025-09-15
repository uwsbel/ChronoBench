import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math
import random


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')
sens.SetDataPath(chrono.GetChronoDataPath())  


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




system = vehicle.GetSystem()
random.seed(42)  


for _ in range(20):
    while True:
        
        x = random.uniform(-terrainLength/2, terrainLength/2)
        y = random.uniform(-terrainWidth/2, terrainWidth/2)
        
        if math.sqrt((x - initLoc.x)**2 + (y - initLoc.y)**2) > 3.0:
            break
    
    
    box = chrono.ChBodyEasyBox(1, 1, 1, 1000)
    box.SetPos(chrono.ChVector3d(x, y, 0.5))  
    box.SetFixed(True)
    box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
    system.Add(box)





manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(chrono.ChVector3d(10, 10, 10), chrono.ChColor(1, 1, 1), 1000.0)
manager.scene.AddPointLight(chrono.ChVector3d(-10, 10, 10), chrono.ChColor(1, 1, 1), 1000.0)
manager.scene.SetAmbientLight(chrono.ChColor(0.1, 0.1, 0.1))


camera_update_rate = 30  
image_width = 1280
image_height = 720
fov = 70  
exposure_time = 0.02  


offset_pos = chrono.ChVector3d(-1, 0, 1)  

R = chrono.ChMatrix33d()
R.SetRow(0, chrono.ChVector3d(0, -1, 0))   
R.SetRow(1, chrono.ChVector3d(0, 0, -1))   
R.SetRow(2, chrono.ChVector3d(1, 0, 0))    
offset_rot = chrono.ChQuaterniond(R)
offset_pose = chrono.ChFramed(offset_pos, offset_rot)


camera = sens.ChCameraSensor(
    vehicle.GetChassisBody(),     
    camera_update_rate,           
    offset_pose,                  
    image_width,                  
    image_height,                 
    fov,                          
    1,                            
    sens.CameraLensModelType_PINHOLE,
    False                         
)
camera.SetName("Vehicle Camera")
camera.SetLag(0)                 
camera.SetCollectionWindow(exposure_time)  
camera.PushFilter(sens.ChFilterVisualize(640, 360, "Camera Feed"))  
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
    vis.Advance(step_size)
    
    
    manager.Update()

    
    step_number += 1
    realtime_timer.Spin(step_size)