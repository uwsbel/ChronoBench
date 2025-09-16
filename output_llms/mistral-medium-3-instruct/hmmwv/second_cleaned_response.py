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


path_radius = 20.0  
path_center = chrono.ChVector3d(0, 0, 0)
constant_throttle = 0.3


Kp = 0.5  
Ki = 0.0  
Kd = 0.1  


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


path_vis = irr.ChIrrTools(vis.GetSceneManager(), vis.GetVideoDriver())
path_vis.addSphere(chrono.ChVector3d(path_center.x + path_radius, path_center.y, path_center.z), 0.5, chrono.ChColor(1, 0, 0))
path_vis.addSphere(chrono.ChVector3d(path_center.x - path_radius, path_center.y, path_center.z), 0.5, chrono.ChColor(0, 1, 0))


integral_error = 0
prev_error = 0


print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())


render_steps = math.ceil(render_step_size / step_size)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    
    vehicle_pos = vehicle.GetVehicle().GetPos()
    vehicle_rot = vehicle.GetVehicle().GetRot()

    
    forward_dir = vehicle_rot.Rotate(chrono.ChVector3d(1, 0, 0))
    forward_dir.z = 0  
    forward_dir.Normalize()

    
    to_center = path_center - vehicle_pos
    to_center.z = 0  

    
    lateral_error = to_center.Cross(forward_dir).z

    
    desired_heading = to_center.Normalize()
    heading_error = math.atan2(desired_heading.Cross(forward_dir).z, desired_heading.Dot(forward_dir))

    
    integral_error += lateral_error * step_size
    derivative_error = (lateral_error - prev_error) / step_size

    steering_angle = Kp * lateral_error + Ki * integral_error + Kd * derivative_error

    
    steering_angle = max(-0.5, min(0.5, steering_angle))

    prev_error = lateral_error

    
    driver_inputs = veh.ChDriverInputs()
    driver_inputs.m_throttle = constant_throttle
    driver_inputs.m_steering = steering_angle
    driver_inputs.m_braking = 0

    
    if step_number % render_steps == 0:
        
        sentinel_pos = path_center + desired_heading * path_radius
        sentinel_vis = path_vis.addSphere(chrono.ChVector3d(sentinel_pos.x, sentinel_pos.y, sentinel_pos.z + 0.5),
                                         0.3, chrono.ChColor(0, 0, 1))

        
        target_pos = path_center + desired_heading * (path_radius + 5)
        target_vis = path_vis.addSphere(chrono.ChVector3d(target_pos.x, target_pos.y, target_pos.z + 0.5),
                                       0.3, chrono.ChColor(1, 1, 0))

    
    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)