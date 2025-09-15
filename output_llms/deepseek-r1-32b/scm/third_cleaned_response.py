import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math
import random
import numpy as np


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


def add_random_box(system, num_boxes=10):
    for _ in range(num_boxes):
        
        while True:
            x = random.uniform(-50, 50)
            y = random.uniform(-50, 50)
            z = random.uniform(0, 5)
            
            vehicle_pos = vehicle.GetChassisBody().GetPos()
            dist = math.sqrt((x - vehicle_pos.x)**2 + (y - vehicle_pos.y)**2)
            if dist > 5:  
                break
        
        
        box = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 1000)
        box.SetPos(chrono.ChVector3d(x, y, z))
        box.SetRot(chrono.ChQuaterniond(1, random.uniform(0,1), random.uniform(0,1), random.uniform(0,1)).Normalize())
        box.SetMaterialSurface(chrono.ChMaterialSurface())
        box.GetMaterialSurface().SetYoungModulus(2e8)
        box.GetMaterialSurface().SetFriction(0.5)
        
        
        box.SetColor(chrono.ChColor(random.uniform(0,1), random.uniform(0,1), random.uniform(0,1)))
        
        
        system.Add(box)

add_random_box(vehicle.GetSystem(), num_boxes=20)


sensor_manager = sens.ChSensorManager(vehicle.GetSystem())


light1 = irr.ChPointLight()
light1.SetPos(chrono.ChVector3d(10, 10, 10))
light1.SetIntensity(1)
sensor_manager.AddLight(light1)

light2 = irr.ChPointLight()
light2.SetPos(chrono.ChVector3d(-10, -10, 10))
light2.SetIntensity(1)
sensor_manager.AddLight(light2)


camera = sens.ChCameraSensor(sensor_manager)
camera.SetResolution(640, 480)
camera.SetFieldOfView(60)
camera.SetPosition(chrono.ChVector3d(0, 0, 1.71))  
camera.SetRotation(chrono.ChQuaterniond(1, 0, 0, 0))  
camera.AttachTo(vehicle.GetChassisBody())  


camera_filter = sens.ChFilterVisualize()
camera_filter.SetWindowTitle("Camera Feed")
camera_filter.SetFrequency(30)  
camera.AddFilter(camera_filter)


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo with Sensors')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())


vis.AttachSensorManager(sensor_manager)


driver = veh.ChInteractiveDriverIRR(vis)


steering_time = 1.0  
throttle_time = 1.0  
braking_time = 0.3   
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()






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
    sensor_manager.Synchronize(time)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)
    sensor_manager.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)