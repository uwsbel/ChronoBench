import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math
import numpy as np
import random


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


terrain.Initialize(veh.GetDataFile("terrain/height_maps/bump64.bmp"), 40, 40, -1, 1, 0.02)

terrain.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)


system = vehicle.GetSystem()
box_material = chrono.ChMaterialSurfaceSMC()


for i in range(5):
    
    x_pos = random.uniform(-10, 10)
    y_pos = random.uniform(-10, 10)
    z_pos = 0.5  
    
    
    box_size_x = random.uniform(0.5, 2.0)
    box_size_y = random.uniform(0.5, 2.0)
    box_size_z = random.uniform(0.5, 2.0)
    
    box = chrono.ChBodyEasyBox(box_size_x, box_size_y, box_size_z, 1000, True, True, box_material)
    box.SetPos(chrono.ChVector3d(x_pos, y_pos, z_pos))
    box.SetBodyFixed(True)
    box.SetCollide(True)
    
    
    box.AddVisualShape(chrono.ChVisualShapeBox(chrono.ChVector3d(box_size_x, box_size_y, box_size_z)))
    box.GetVisualShape(0).SetColor(chrono.ChColor(0.8, 0.2, 0.1))
    
    
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
manager.scene.AddPointLight(chrono.ChVector3d(0, 0, 100), chrono.ChColor(1, 1, 1), 500)


offset_pose = chrono.ChFrame3d(
    chrono.ChVector3d(0, 0, 2.0),  
    chrono.ChQuaterniond(1, 0, 0, 0)  
)

lidar = sens.ChLidarSensor(
    vehicle.GetChassisBody(),           
    10,                                 
    offset_pose,                        
    1000,                               
    50,                                 
    chrono.CH_C_2PI,                    
    chrono.CH_C_PI / 6.0,               
    100.0                               
)


lidar.SetLidarNoiseXYZ(0.01, 0.01, 0.01)


lidar_vis = sens.ChFilterVisualize3D(
    1280,               
    720,                
    "Lidar Display",    
    False               
)


lidar_data = sens.ChFilterLidarNoiseXYZ(0.01, 0.01, 0.01)


lidar.PushFilter(lidar_vis)
lidar.PushFilter(lidar_data)
manager.AddSensor(lidar)






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
    
    
    manager.Update()

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)