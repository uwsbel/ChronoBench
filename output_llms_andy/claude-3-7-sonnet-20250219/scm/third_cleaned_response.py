import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math
import numpy as np
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
vehicle.SetInitPosition(chrono.ChCoordsys(initLoc, initRot))  
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


terrain.AddMovingPatch(vehicle.GetChassisBody(), chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(10, 5, 1))  


terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)


terrain.Initialize(20, 20, 0.02)


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo with Sensors and Objects')
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




def is_close_to_vehicle(pos, vehicle_pos, min_distance=5.0):
    dx = pos.x - vehicle_pos.x
    dy = pos.y - vehicle_pos.y
    distance = math.sqrt(dx*dx + dy*dy)
    return distance < min_distance


vehicle_pos = vehicle.GetVehicle().GetPos()


system = vehicle.GetSystem()
num_boxes = 20

for i in range(num_boxes):
    
    while True:
        x = random.uniform(-30, 30)
        y = random.uniform(-30, 30)
        pos = chrono.ChVector3d(x, y, 0.5)  
        if not is_close_to_vehicle(pos, vehicle_pos):
            break
    
    
    box_size_x = random.uniform(0.5, 1.5)
    box_size_y = random.uniform(0.5, 1.5)
    box_size_z = random.uniform(0.5, 2.0)
    
    box = chrono.ChBodyEasyBox(box_size_x, box_size_y, box_size_z, 1000, True, True)
    box.SetPos(pos)
    box.SetRot(chrono.Q_from_AngZ(random.uniform(0, 2*math.pi)))
    box.SetBodyFixed(False)
    
    
    col = chrono.ChColor(random.random(), random.random(), random.random())
    box.GetVisualShape(0).SetColor(col)
    
    system.Add(box)





manager = sens.ChSensorManager(system)


light_positions = [
    chrono.ChVectorD(10, 10, 10),
    chrono.ChVectorD(-10, 10, 10),
    chrono.ChVectorD(10, -10, 10),
    chrono.ChVectorD(-10, -10, 10)
]

for pos in light_positions:
    
    manager.AddPointLight(pos, chrono.ChColor(1, 1, 1), 500.0)


camera_offset = chrono.ChVectorD(0, 0, 1.5)  
camera_direction = chrono.ChVectorD(1, 0, 0)  
camera_up = chrono.ChVectorD(0, 0, 1)         


camera = sens.ChCameraSensor(
    vehicle.GetChassisBody(),                  
    30.0,                                      
    chrono.ChFrameD(camera_offset, chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 0, 1))),  
    1280,                                      
    720,                                       
    1.0                                        
)


camera.SetName("Camera Sensor")
camera.SetLag(0.0)
camera.SetCollectionWindow(0.0)


camera.PushFilter(sens.ChFilterVisualize(1280, 720, "Camera View", 0.8))


camera.PushFilter(sens.ChFilterRGBNoise(0.1, 0.1, 0.1))


camera.PushFilter(sens.ChFilterSave("camera/"))


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
    
    
    manager.Update()

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)