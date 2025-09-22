import math
import random

import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens     




chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

random.seed(1)         


initLoc = chrono.ChVector3d(-8, 0, 0.6)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type               = veh.VisualizationType_MESH
chassis_collision_type = veh.ChassisCollisionType_NONE   
tire_model             = veh.TireModelType_RIGID


terrainHeight = 0
terrainLength = 100.0
terrainWidth  = 100.0


trackPoint = chrono.ChVector3d(0.0, 0.0, 1.71)


contact_method = chrono.ChContactMethod_SMC


step_size       = 1e-3
tire_step_size  = step_size
render_step_size = 1.0 / 50.0          




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

system = vehicle.GetSystem()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)




terrain = veh.SCMTerrain(system)
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
vis.SetWindowTitle("HMMWV with Sensors")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())




driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / 1.0)   
driver.SetThrottleDelta(render_step_size / 1.0)   
driver.SetBrakingDelta (render_step_size / 0.3)   
driver.Initialize()




box_density  = 800      
box_size     = (0.6, 0.6, 0.6)
num_boxes    = 20
min_distance = 5.0      

for _ in range(num_boxes):
    while True:
        x = random.uniform(-terrainLength / 2, terrainLength / 2)
        y = random.uniform(-terrainWidth  / 2, terrainWidth  / 2)
        if math.hypot(x - initLoc.x, y - initLoc.y) > min_distance:
            break
    z = terrainHeight + box_size[2] / 2.0
    body = chrono.ChBodyEasyBox(*box_size,       
                                box_density,
                                True, True)      
    body.SetPos(chrono.ChVector3d(x, y, z))
    body.SetRot(chrono.ChQuaterniond(1, 0, 0, 0))
    system.Add(body)




smanager = sens.ChSensorManager(system)


cam_width  = 640
cam_height = 480


light_positions = [
    chrono.ChVectorF(0,   0, 10),
    chrono.ChVectorF(-5, -5, 5),
    chrono.ChVectorF(10,  0, 6)
]
for lp in light_positions:
    smanager.scene.AddPointLight(lp, chrono.ChColor(1, 1, 1), 150.0)



cam_update_rate = 30.0                      
fov            = math.radians(60)           

cam_rel_pose = chrono.ChFrameD(
    chrono.ChVectorD(0.0, 0.0, 1.5),        
    chrono.QUNIT                )           

camera = sens.ChCameraSensor(
    vehicle.GetChassisBody(),               
    cam_update_rate,
    cam_rel_pose,
    cam_width,
    cam_height,
    fov
)
camera.SetName("DriverCam")


camera.PushFilter(sens.ChFilterRGBA8Access())


camera.PushFilter(sens.ChFilterVisualize(cam_width, cam_height, "Camera"))

smanager.AddSensor(camera)




print("VEHICLE MASS:", vehicle.GetVehicle().GetMass())

render_steps   = math.ceil(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()

step_number  = 0
render_frame = 0

while vis.Run():
    time = system.GetChTime()

    
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

    
    smanager.Update()

    
    realtime_timer.Spin(step_size)
    step_number += 1