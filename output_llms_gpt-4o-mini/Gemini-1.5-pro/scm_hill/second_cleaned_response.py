import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math
import numpy as np


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


terrain.Initialize(veh.GetDataFile("terrain/height_maps/bump64.bmp"),40, 40, -1,1, 0.02)

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




obstacle_assets = []
for i in range(5):
    obstacle_asset = chrono.ChAsset()
    obstacle_asset.SetFileName(chrono.GetChronoDataFile("sensor/textures/cube.obj"))
    obstacle_assets.append(obstacle_asset)

obstacles = []
for i in range(5):
    pos = chrono.ChVector3d(np.random.uniform(low=-50, high=50), np.random.uniform(low=-10, high=10), 1)
    obstacle = chrono.ChBodyEasyBox(2, 2, 2, 1000, True, True)
    obstacle.SetPos(pos)
    obstacle.AddAsset(obstacle_assets[i])
    obstacle.GetCollisionModel().SetFamily(1)
    obstacle.GetCollisionModel().SetFamilyMaskNoCollisionWithFamily(1)
    vehicle.GetSystem().Add(obstacle)
    obstacles.append(obstacle)



manager = sens.ChSensorManager(vehicle.GetSystem())
manager.scene.AddPointLight(chrono.ChVectorF(100, 100, 100), chrono.ChColor(1, 1, 1), 500)
manager.scene.AddPointLight(chrono.ChVectorF(-100, 0, 100), chrono.ChColor(0.6, 0.6, 0.8), 500)
manager.SetAmbientLight(chrono.ChColor(0.4, 0.4, 0.4))
manager.scene.SetSceneRadiance(chrono.ChColor(0.8, 0.8, 0.8))


lidar = sens.ChLidarSensor()
lidar.SetName("Lidar Sensor")
lidar.SetLag(0.0)
lidar.SetUpdateRate(50)
lidar.SetPos(chrono.ChFrameD(chrono.ChVectorD(0, 0, 1.6)))
lidar.SetDirection(chrono.ChFrameD(chrono.ChQuaternionD(1, 0, 0, 0)))
lidar.SetBodyFixed(True)


lidar.SetVerticalSamples(64)
lidar.SetHorizontalSamples(1024)
lidar.SetRange(100.0)
lidar.SetVerticalAngleSpan(chrono.CH_C_PI / 4)
lidar.SetHorizontalAngleSpan(chrono.CH_C_2PI)
lidar.SetIntensity(1)
lidar.SetOffset(chrono.ChFrameD())


noise_model = sens.ChNoiseNormal(0.01, 0.005)
intensity_filter = sens.ChFilterIntensityConstant(1)
range_filter = sens.ChFilterRange(0.1, lidar.GetRange())
lidar.AddNoise(noise_model)
lidar.AddFilter(intensity_filter)
lidar.AddFilter(range_filter)

vis_mesh_color = chrono.ChColorAsset()
vis_mesh_color.SetColor(chrono.ChColor(1., 1., 1.))
lidar.AddAsset(vis_mesh_color)


manager.AddSensor(lidar)





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
    manager.Update()

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)
    manager.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)