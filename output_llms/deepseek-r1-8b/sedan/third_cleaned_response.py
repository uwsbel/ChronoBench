import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_TMEASY


terrainHeight = 0.0  
terrainLength = 100.0   
terrainWidth = 100.0   


trackPoint = chrono.ChVector3d(-5.0, 0.0, 1.8)


contact_method = chrono.ChContactMethod_NSC
contact_vis = False


step_size = 0.0001  
tire_step_size = step_size
render_step_size = step_size  


vehicle = veh.BMW_E90()
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


patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)


patches = []
for i in range(int(terrainLength / 2) + 1):
    for j in range(int(terrainWidth / 2) + 1):
        patch = terrain.AddPatch(patch_mat,
            chrono.ChCoordsysd(chrono.ChVector3d(i, j, 0), chrono.QUNIT),
            5, 5)  
        patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
        patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
        patches.append(patch)

terrain.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Sedan')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())


driver = veh.ChInteractiveDriverIRR(vis)


kp = 0.1
ki = 0.01
kd = 0.1
ref_speed = 25.0  


steering_time = 5.0  
throttle_time = 0.5  
braking_time = 0.3


driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()


print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())


render_steps = math.ceil(render_step_size / step_size)
render_frame = 0
step_number = 0
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

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    speed = vehicle.GetVehicle().GetSpeed()
    speed_error = ref_speed - speed

    
    if abs(speed_error) > 0.01:
        throttle = (kp * speed_error) + (ki * sum(speed_errors) * step_size) + (kd * (speed_error - speed_errors[-1]))
        if throttle > 1.0:
            throttle = 1.0
        elif throttle < 0.0:
            throttle = 0.0
        vehicle.SetThrottle(throttle)
    
    
    speed_errors.append(speed_error)

    step_number += 1
    realtime_timer.Spin(step_size)