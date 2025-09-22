import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_PRIMITIVES


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_TMEASY


terrainHeight = 0      
terrainLength = 200.0  
terrainWidth = 200.0   


trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)


contact_method = chrono.ChContactMethod_NSC
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


patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Path Following Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())






circle_radius = 30.0
circle_center = chrono.ChVector3d(0, 0, 0)


Kp = 0.5  
Ki = 0.01  
Kd = 0.1  


throttle_value = 0.3


path_marker1 = chrono.ChBody()
path_marker1.SetPos(chrono.ChVector3d(circle_radius, 0, 0.5))
path_marker1.SetCollide(False)
path_marker1.SetBodyFixed(True)
path_marker1.GetVisualModel().AddSphere(1.0)
path_marker1.GetVisualModel().SetColor(chrono.ChColor(1, 0, 0))  
vehicle.GetSystem().AddBody(path_marker1)

path_marker2 = chrono.ChBody()
path_marker2.SetPos(chrono.ChVector3d(0, circle_radius, 0.5))
path_marker2.SetCollide(False)
path_marker2.SetBodyFixed(True)
path_marker2.GetVisualModel().AddSphere(1.0)
path_marker2.GetVisualModel().SetColor(chrono.ChColor(0, 1, 0))  
vehicle.GetSystem().AddBody(path_marker2)


sentinel_marker = chrono.ChBody()
sentinel_marker.SetCollide(False)
sentinel_marker.GetVisualModel().AddSphere(0.5)
sentinel_marker.GetVisualModel().SetColor(chrono.ChColor(0, 0, 1))  
vehicle.GetSystem().AddBody(sentinel_marker)

target_marker = chrono.ChBody()
target_marker.SetCollide(False)
target_marker.GetVisualModel().AddSphere(0.5)
target_marker.GetVisualModel().SetColor(chrono.ChColor(1, 1, 0))  
vehicle.GetSystem().AddBody(target_marker)


integral = 0
prev_error = 0






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

    
    vehicle_pos = vehicle.GetVehicle().GetPos()
    vehicle_rot = vehicle.GetVehicle().GetRot()

    
    look_ahead_dist = 5.0  
    forward_dir = vehicle_rot.Rotate(chrono.ChVector3d(0, 1, 0))  
    sentinel_pos = vehicle_pos + forward_dir * look_ahead_dist

    
    
    vec_to_sentinel = sentinel_pos - circle_center
    current_radius = vec_to_sentinel.Length()
    target_pos = circle_center + vec_to_sentinel.GetNormalized() * circle_radius

    
    sentinel_marker.SetPos(sentinel_pos)
    target_marker.SetPos(target_pos)

    
    error = (target_pos - sentinel_pos).Length()

    
    lateral_dev = (target_pos - vehicle_pos).Cross(forward_dir).z

    
    integral += error * step_size
    derivative = (error - prev_error) / step_size
    steering_command = Kp * lateral_dev + Ki * integral + Kd * derivative
    prev_error = error

    
    steering_command = max(-1.0, min(1.0, steering_command))

    
    driver_inputs = veh.ChDriverInputs()
    driver_inputs.m_throttle = throttle_value
    driver_inputs.m_steering = steering_command
    driver_inputs.m_braking = 0

    
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)


print("VEHICLE MASS:", vehicle.GetVehicle().GetMass())