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
terrainWidth = 100.0   


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
vis.SetWindowTitle('HMMWV Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())


path_radius = 10.0  
target_point = chrono.ChVector3d(path_radius, 0, 0)  
sentinel_point = chrono.ChVector3d(-path_radius, 0, 0)  


sphere_target = chrono.ChVisualShapeSphere(0.5)  
sphere_target.SetColor(chrono.ChColor(1, 0, 0))  
sphere_target.SetPos(target_point)
vehicle.GetSystem().AddVisualShape(sphere_target)

sphere_sentinel = chrono.ChVisualShapeSphere(0.5)  
sphere_sentinel.SetColor(chrono.ChColor(0, 1, 0))  
sphere_sentinel.SetPos(sentinel_point)
vehicle.GetSystem().AddVisualShape(sphere_sentinel)


Kp = 1.0  
Ki = 0.1  
Kd = 0.5  
previous_error = 0
integral = 0


throttle_value = 0.3


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

    
    angle_to_target = math.atan2(target_point.y - vehicle_pos.y, target_point.x - vehicle_pos.x)
    vehicle_angle = vehicle.GetVehicle().GetRot().GetEuler()  

    
    error = angle_to_target - vehicle_angle.z
    integral += error * step_size
    derivative = (error - previous_error) / step_size
    steering = Kp * error + Ki * integral + Kd * derivative
    previous_error = error

    
    driver_inputs = veh.ChDriverInputs()
    driver_inputs.m_throttle = throttle_value
    driver_inputs.m_steering = steering

    
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)