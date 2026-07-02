import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import numpy as np
import math


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(-15, 0, 1.2)
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


render_step_size = 1.0 / 20  


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


obstacles = []
for _ in range(5):
    obstacle = chrono.ChBodyEasyBox(2.0, 2.0, 2.0, 1000, True, True)
    obstacle.SetPos(chrono.ChVector3d(
        np.random.uniform(-40, 40),
        np.random.uniform(-40, 40),
        1.0
    ))
    obstacle.SetFixed(True)
    obstacle.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
    vehicle.GetSystem().AddBody(obstacle)
    obstacles.append(obstacle)


terrain = veh.SCMTerrain(vehicle.GetSystem())
terrain.SetSoilParameters(2e6, 0, 1.1, 0, 30, 0.01, 2e8, 3e4)
terrain.AddMovingPatch(vehicle.GetChassisBody(), chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(5, 3, 1))
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)
terrain.Initialize(veh.GetDataFile("terrain/height_maps/bump64.bmp"), 40, 40, -1, 1, 0.02)
terrain.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)


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


manager = sens.ChSensorManager(vehicle.GetSystem())
manager.scene.AddPointLight(chrono.ChVector3f(20, 20, 100), chrono.ChColor(1, 1, 1), 500)

lidar = sens.ChLidarSensor(
    vehicle.GetChassisBody(),  
    5,                          
    chrono.ChFrame<>(chrono.ChVector3d(0, 0, 2), chrono.Q_from_AngAxis(0, chrono.ChVector3d(0, 1, 0))),  
    360,                        
    1,                          
    2 * math.pi,                
    0.01,                       
    100.0                       
)
lidar.SetName("LIDAR Sensor")
lidar.SetLag(0.0)
lidar.SetCollectionWindow(0.0)
lidar.PushFilter(sens.ChFilterVisualize(360, 1, "LIDAR Visualization"))
manager.AddSensor(lidar)


print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())
render_steps = math.ceil(render_step_size / step_size)
step_number = 0
render_frame = 0
realtime_timer = chrono.ChRealtimeStepTimer()

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
    
    
    manager.Update()
    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)
    
    step_number += 1
    realtime_timer.Spin(step_size)