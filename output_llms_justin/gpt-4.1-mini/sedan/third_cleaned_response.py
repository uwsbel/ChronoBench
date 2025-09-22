import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')






initLoc = chrono.ChVector3d(5, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_TMEASY


terrainHeight = 0  
terrainMeshFile = veh.GetDataFile('terrain/mesh/highway/scene.obj')  


trackPoint = chrono.ChVector3d(-5.0, 0.0, 1.8)


contact_method = chrono.ChContactMethod_NSC
contact_vis = False


step_size = 5e-4  
tire_step_size = step_size


render_step_size = 1.0 / 100  


reference_speed = 15.0  



pid_Kp = 0.5
pid_Ki = 0.1
pid_Kd = 0.0


integral_error = 0.0
prev_error = 0.0






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


terrain = veh.RigidTerrain(vehicle.GetSystem())


patch = terrain.AddPatch(
    chrono.ChContactMaterialNSC(),
    chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)),
    terrainMeshFile
)

patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
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



steering_time = 5.0  
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()






print("VEHICLE MASS:", vehicle.GetVehicle().GetMass())


render_steps = math.ceil(render_step_size / step_size)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0


def pid_control(speed_measured, dt):
    global integral_error, prev_error
    error = reference_speed - speed_measured
    integral_error += error * dt
    derivative = 0.0
    if dt > 0:
        derivative = (error - prev_error) / dt
    prev_error = error
    
    output = pid_Kp * error + pid_Ki * integral_error + pid_Kd * derivative
    
    if output > 1.0:
        output = 1.0
    if output < 0.0:
        output = 0.0
    return output

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    
    if (step_number % render_steps) == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    driver_inputs = driver.GetInputs()

    

    
    chassis_vel = vehicle.GetVehicle().GetPointVelocityLocal(chrono.ChVectorD(0, 0, 0))  
    
    spd = chassis_vel.x

    
    pid_throttle = pid_control(spd, step_size)

    
    
    
    new_driver_inputs = veh.DriverInputs()
    new_driver_inputs.m_steering = driver_inputs.m_steering
    new_driver_inputs.m_throttle = pid_throttle
    new_driver_inputs.m_braking = driver_inputs.m_braking

    
    driver.Synchronize(time)
    
    
    
    
    
    terrain.Synchronize(time)
    vehicle.Synchronize(time, new_driver_inputs, terrain)
    vis.Synchronize(time, new_driver_inputs)

    
    driver.Advance(step_size)  
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)