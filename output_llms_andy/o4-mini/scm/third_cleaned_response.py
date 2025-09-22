import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math
import random




chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")





initLoc = chrono.ChVector3d(-8, 0, 0.6)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_NONE
tire_model = veh.TireModelType_RIGID


soil_Kphi = 2e6
soil_Kc   = 0.0
soil_n    = 1.1
soil_cohesion = 0.0
soil_friction = 30
soil_Janosi   = 0.01
soil_E       = 2e8
soil_damping = 3e4


step_size       = 1e-3
tire_step_size  = step_size
render_fps      = 50
render_step_size= 1.0/render_fps


contact_method = chrono.ChContactMethod_SMC




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
terrain.SetSoilParameters(soil_Kphi, soil_Kc, soil_n,
                          soil_cohesion, soil_friction,
                          soil_Janosi, soil_E, soil_damping)


terrain.AddMovingPatch(vehicle.GetChassisBody(),
                       chrono.ChVector3d(0,0,0),
                       chrono.ChVector3d(5,3,1))

terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)
terrain.Initialize(20, 20, 0.02)




vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo with Boxes & Camera Sensor')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(0,0,1.7), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())




system = vehicle.GetSystem()
num_boxes = 20
box_dims  = (1.0, 0.5, 0.25)   
for i in range(num_boxes):
    
    while True:
        x = random.uniform(-50,50)
        y = random.uniform(-50,50)
        if (x-initLoc.x)**2 + (y-initLoc.y)**2 > 25:
            break
    lx, ly, lz = box_dims
    box = chrono.ChBodyEasyBox(lx, ly, lz,       
                               1000,             
                               True, True)       
    box.SetPos(chrono.ChVectorD(x, y, lz/2))
    box.SetBodyFixed(False)
    system.Add(box)





sensor_manager = sens.ChSensorManager(system)




cam_offset = chrono.ChFrameD(
    chrono.ChVectorD(0.5, 0, 1.7),
    chrono.ChQuaternionD(1, 0, 0, 0)
)
camera = sens.ChCameraSensor(
    vehicle.GetChassisBody(),     
    render_fps,                   
    cam_offset,                   
    640,                          
    480,                          
    chrono.CH_C_PI / 3            
)

camera.PushFilter(sens.ChFilterVisualizeRGBA())
sensor_manager.AddSensor(camera)






device = vis.GetDevice()
scene  = device.getSceneManager()
light_positions = [
    chrono.ChVectorD( 20,  20, 8),
    chrono.ChVectorD(-20, -20, 8),
    chrono.ChVectorD(  0,   0, 8),
]
for pos in light_positions:
    scene.addLightSceneNode(
        None,
        irr.core.vector3df(pos.x, pos.y, pos.z),
        irr.video.SColorf(1.0,1.0,1.0,1.0),
        50.0   
    )




driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time  = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()




print("VEHICLE MASS:", vehicle.GetVehicle().GetMass())

render_steps = math.ceil(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()
step_number    = 0

while vis.Run():
    time = system.GetChTime()

    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    
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

    step_number += 1
    realtime_timer.Spin(step_size)