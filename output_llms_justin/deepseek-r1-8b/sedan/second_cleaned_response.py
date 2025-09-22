import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math
import os


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_TMEASY


terrainHeight = 0      
terrainLength = 100.0  
terrainWidth = 100.0   


trackPoint = chrono.ChVector3d(-5.0, 0.0, 1.8)


contact_method = chrono.ChContactMethod_NSC
contact_vis = False


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 50  






vehicle = veh.BMW_E90()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()


truck = veh.Truck()
truck.SetContactMethod(contact_method)
truck.SetChassisCollisionType(chassis_collision_type)
truck.SetChassisFixed(False)
truck.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 1.5), chrono.QUNIT))
truck.SetTireType(tire_model)
truck.SetTireStepSize(tire_step_size)
truck.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Sedan & Truck')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())
vis.AttachVehicle(truck.GetVehicle())


driver = veh.ChInteractiveDriverIRR(vis)
truck_driver = veh.ChInteractiveDriverIRR(vis)


steering_time = 1.0  
throttle_time = 1.0  
braking_time = 0.3   


def get_steering_input(time):
    return 1.0 * math.sin(time / 10)  


driver.SetSteeringDelta(render_step_size / steering_time)
truck_driver.SetSteeringDelta(render_step_size / steering_time)

driver.SetThrottleDelta(render_step_size / throttle_time)
truck_driver.SetThrottleDelta(render_step_size / throttle_time)

driver.SetBrakingDelta(render_step_size / braking_time)
truck_driver.SetBrakingDelta(render_step_size / braking_time)


driver.Initialize()
truck_driver.Initialize()






print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())
print("TRUCK MASS: ", truck.GetVehicle().GetMass())


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

    
    vehicle.Synchronize(time)
    truck.Synchronize(time)
    driver.Synchronize(time)
    truck_driver.Synchronize(time)
    vis.Synchronize(time, driver_inputs)

    
    vehicle.Advance(step_size)
    truck.Advance(step_size)
    driver.Advance(step_size)
    truck_driver.Advance(step_size)

    
    steering_angle = get_steering_input(time)
    if steering_angle > 1.0:
        steering_angle = 1.0
    elif steering_angle < -1.0:
        steering_angle = -1.0
    driver.SetSteeringInput(steering_angle)
    truck_driver.SetSteeringInput(steering_angle)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)