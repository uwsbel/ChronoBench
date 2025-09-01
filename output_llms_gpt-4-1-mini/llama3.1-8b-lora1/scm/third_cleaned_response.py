import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math





chrono.SetChronoDataPath(chrono.GetChronoDataPath() + 'vehicle/')


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


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())


sensor_manager = veh.ChVehicleSensorManagerIRR(vis)

camera = veh.ChCameraSensor(
    vehicle.GetChassisBody(),            
    chrono.ChVector3d(0.0, 0.0, 2.0),    
    chrono.ChVector3d(0.0, 0.0, 1.0).normalized(),  
    1280,  
    720,   
    3.14   
)
camera.SetName("Camera")
camera.SetLag(0)
camera.SetCollectionWindow(0, float('inf'))  

sensor_manager.AddCamera(camera)

sensor_manager.Initialize()


driver = veh.ChInteractiveDriverIRR(vis)


steering_time = 1.0  
throttle_time = 1.0  
braking_time = 0.3   
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()






for i in range(10):
    box = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 1000,  
                               True,            
                               False)           
    box.SetPos(chrono.ChVector3d(5 + (i - 5) * 2, 0, 1))
    box.SetFixed(True)
    vehicle.GetSystem().Add(box)






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
    vis.Synchronize(time, driver_inputs)

    
    driver.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)