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


path_radius = 10.0
path_angle_increment = math.radians(5)
path_target_index = 0
path_target_list = []


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
patch = terrain.AddPatch(patch_mat,chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),terrainLength, terrainWidth)
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


Kp = 0.2
Ki = 0.0
Kd = 0.0
steering_error_sum = 0.0
prev_steering_error = 0.0
steering_command = 0.0

def PID_controller(setpoint, current_value):
    global steering_error_sum, prev_steering_error, steering_command, Kp, Ki, Kd
    error = setpoint - current_value
    steering_error_sum += error
    derivative = error - prev_steering_error
    prev_steering_error = error
    steering_command = Kp * error + Ki * steering_error_sum + Kd * derivative
    return steering_command


for angle in range(0, 360, int(path_angle_increment)):
    x = path_radius * math.cos(math.radians(angle))
    y = path_radius * math.sin(math.radians(angle))
    path_target_list.append(chrono.ChVector2d(x, y))


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

        
        if path_target_index < len(path_target_list):
            target_pos = chrono.ChVector3d(path_target_list[path_target_index].x(), path_target_list[path_target_index].y(), 0.1)
            vis.AddSphere(target_pos, 0.3, chrono.ChColor(0, 1, 0))

            sentinel_pos = vehicle.GetVehicle().GetChassisWorldTransform().TransformPointLocal(chrono.ChVector3d(0, 0, 0))
            vis.AddSphere(sentinel_pos, 0.3, chrono.ChColor(1, 0, 0))

    
    steering_command = PID_controller(path_target_list[path_target_index].x(), vehicle.GetVehicle().GetChassisWorldTransform().TransformPointLocal(chrono.ChVector3d(0, 0, 0)).x())
    driver_inputs = veh.ChDriverInputs()
    driver_inputs.steer = steering_command
    driver_inputs.throttle = 0.3
    driver_inputs.brake = 0.0

    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    if abs(vehicle.GetVehicle().GetChassisWorldTransform().TransformPointLocal(chrono.ChVector3d(0, 0, 0)).x() - path_target_list[path_target_index].x()) < 0.1:
        path_target_index += 1

    
    step_number += 1

    
    realtime_timer.Spin(step_size)