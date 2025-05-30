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






min_dist_from_vehicle = 3.0  

num_boxes = 10
box_size = chrono.ChVectorD(0.5, 0.5, 0.5)  
boxes = []

for _ in range(num_boxes):
    while True:
        
        x = random.uniform(-terrainLength/2, terrainLength/2)
        y = random.uniform(-terrainWidth/2, terrainWidth/2)
        z = terrainHeight + box_size.z()/2

        pos = chrono.ChVectorD(x, y, z)

        dist = (pos - initLoc).Length()
        
        if dist > min_dist_from_vehicle:
            break

    box = chrono.ChBodyEasyBox(box_size.x(), box_size.y(), box_size.z(), 1000, True, True)  
    box.SetPos(pos)
    box.SetBodyFixed(False)
    box.SetName(f"Box_{_}")
    vehicle.GetSystem().Add(box)
    boxes.append(box)






sensor_manager = sens.ChSensorManager(vehicle.GetSystem())


sensor_update_hz = 30.0
sensor_update_period = 1.0 / sensor_update_hz




light_positions = [
    chrono.ChVectorD(-10, -10, 10),
    chrono.ChVectorD(10, -10, 10),
    chrono.ChVectorD(-10, 10, 10),
    chrono.ChVectorD(10, 10, 10),
    chrono.ChVectorD(0, 0, 15),
]

for i, light_pos in enumerate(light_positions):
    point_light = sens.ChLightPointSensor(vehicle.GetSystem())
    point_light.SetName(f"PointLight_{i}")
    point_light.SetOffsetPose(chrono.ChCoordsysD(light_pos, chrono.ChQuaternionD(1, 0, 0, 0)))
    
    
    sensor_manager.AddSensor(point_light)


cam_offset_pose = chrono.ChCoordsysD(chrono.ChVectorD(0.8, 0, 1.2), chrono.ChQuaternionD(1, 0, 0, 0))  
camera_fov = 1.2  
camera_width = 640
camera_height = 480
camera_update_rate = sensor_update_hz

camera = sens.ChCameraSensor(
    vehicle.GetChassisBody(),           
    camera_update_rate,                  
    cam_offset_pose,                    
    camera_width,                       
    camera_height,                      
    camera_fov                         
)

camera.SetName("ChassisCamera")
camera.PushFilter(sens.ChFilterVisualizeRGBA8())  
sensor_manager.AddSensor(camera)






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
    sensor_manager.Synchronize(time)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)
    sensor_manager.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)