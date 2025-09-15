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


def create_obstacles(system):
    num_obstacles = 5
    for i in range(num_obstacles):
        
        pos = chrono.ChVector3d(
            np.random.uniform(-50, 50),  
            np.random.uniform(-50, 50),  
            np.random.uniform(0.5, 2)    
        )
        size = chrono.ChVector3d(
            np.random.uniform(1, 3),    
            np.random.uniform(1, 3),    
            np.random.uniform(1, 3)     
        )
        
        
        box = chrono.ChBoxShape()
        box.SetDimensions(size)
        
        
        body = chrono.ChBody()
        body.SetPos(pos)
        body.SetMass(1000)  
        body.AddVisualShape(box)
        body.GetCollisionModel().AddBoxGeometry(size)
        
        
        system.Add(body)

create_obstacles(vehicle.GetSystem())


sensor_manager = sens.ChSensorManager()
sensor_manager.AttachTo(vehicle.GetChassisBody())


lidar_params = sens.ChLidarParameters()
lidar_params.horz_start_angle = -45.0
lidar_params.horz_end_angle = 45.0
lidar_params.vert_start_angle = 15.0
lidar_params.vert_end_angle = -15.0
lidar_params.range = 100.0
lidar_params.horz_res = 0.5
lidar_params.vert_res = 1.0
lidar_params.noise = 0.05

lidar = sens.ChLidar(sensor_manager, lidar_params)
lidar.SetName("lidar")
lidar.SetPos(chrono.ChVector3d(0, 0, 2))  


lidar.GetVisualization().SetMinHitDistance(0.1)
lidar.GetVisualization().SetMaxHitDistance(lidar_params.range)
lidar.GetVisualization().SetDrawPoints(True)
lidar.GetVisualization().SetDrawSegments(True)
lidar.GetVisualization().SetDrawCoordinateSystem(True)


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo with Lidar')
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


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    
    sensor_manager.Update()

    
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

    
    step_number += 1

    
    realtime_timer.Spin(step_size)