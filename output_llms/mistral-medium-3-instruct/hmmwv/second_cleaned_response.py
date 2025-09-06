import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math
import numpy as np

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






path_radius = 25.0  
path_center = chrono.ChVector3d(0, 0, 0)
throttle = 0.3


Kp = 0.5  
Ki = 0.01  
Kd = 0.1  


path_vis_radius = 0.5
path_vis_mat = chrono.ChContactMaterialNSC()
path_vis_mat.SetFriction(0.9)
path_vis_mat.SetRestitution(0.01)


ball1 = chrono.ChBody()
ball1.SetPos(chrono.ChVector3d(path_radius, 0, 0.5))
ball1.SetRot(chrono.QUNIT)
ball1.SetMass(1)
ball1.SetCollide(False)
ball1.SetBodyFixed(True)
ball1.GetVisualModel().AddSphere(path_vis_radius)
ball1.GetVisualModel().SetColor(chrono.ChColor(1, 0, 0))
vehicle.GetSystem().Add(ball1)

ball2 = chrono.ChBody()
ball2.SetPos(chrono.ChVector3d(-path_radius, 0, 0.5))
ball2.SetRot(chrono.QUNIT)
ball2.SetMass(1)
ball2.SetCollide(False)
ball2.SetBodyFixed(True)
ball2.GetVisualModel().AddSphere(path_vis_radius)
ball2.GetVisualModel().SetColor(chrono.ChColor(0, 1, 0))
vehicle.GetSystem().Add(ball2)


sentinel_ball = chrono.ChBody()
sentinel_ball.SetPos(chrono.ChVector3d(0, 0, 0.5))
sentinel_ball.SetRot(chrono.QUNIT)
sentinel_ball.SetMass(1)
sentinel_ball.SetCollide(False)
sentinel_ball.SetBodyFixed(True)
sentinel_ball.GetVisualModel().AddSphere(0.3)
sentinel_ball.GetVisualModel().SetColor(chrono.ChColor(0, 0, 1))  
vehicle.GetSystem().Add(sentinel_ball)

target_ball = chrono.ChBody()
target_ball.SetPos(chrono.ChVector3d(0, 0, 0.5))
target_ball.SetRot(chrono.QUNIT)
target_ball.SetMass(1)
target_ball.SetCollide(False)
target_ball.SetBodyFixed(True)
target_ball.GetVisualModel().AddSphere(0.3)
target_ball.GetVisualModel().SetColor(chrono.ChColor(1, 1, 0))  
vehicle.GetSystem().Add(target_ball)


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

    
    heading = vehicle_rot.Rotate(chrono.ChVector3d(1, 0, 0))

    
    to_center = path_center - vehicle_pos
    to_center.z = 0  

    
    distance = to_center.Length()

    
    if distance > 0.1:  
        desired_heading = to_center.Cross(chrono.ChVector3d(0, 0, 1)).GetNormalized()
    else:
        desired_heading = chrono.ChVector3d(1, 0, 0)  

    
    
    current_dir = chrono.ChVector3d(heading.x, heading.y, 0).GetNormalized()
    desired_dir = chrono.ChVector3d(desired_heading.x, desired_heading.y, 0).GetNormalized()

    
    dot = current_dir.Dot(desired_dir)
    cross = current_dir.Cross(desired_dir).z
    error = math.atan2(cross, dot)

    
    integral += error * step_size
    derivative = (error - prev_error) / step_size
    steering = Kp * error + Ki * integral + Kd * derivative
    prev_error = error

    
    steering = max(-1.0, min(1.0, steering))

    
    driver_inputs = veh.ChDriverInputs()
    driver_inputs.m_throttle = throttle
    driver_inputs.m_steering = steering
    driver_inputs.m_braking = 0

    
    
    sentinel_pos = vehicle_pos + heading * 2.0
    sentinel_ball.SetPos(chrono.ChVector3d(sentinel_pos.x, sentinel_pos.y, 0.5))

    
    target_pos = vehicle_pos + desired_heading * 5.0
    target_ball.SetPos(chrono.ChVector3d(target_pos.x, target_pos.y, 0.5))

    
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)


print("VEHICLE MASS:", vehicle.GetVehicle().GetMass())