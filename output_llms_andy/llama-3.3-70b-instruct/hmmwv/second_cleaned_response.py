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


path_radius = 20.0
path_center = chrono.ChVector3d(0, 0, 0)
path = []
for i in range(100):
    angle = 2 * math.pi * i / 100
    point = path_center + chrono.ChVector3d(path_radius * math.cos(angle), path_radius * math.sin(angle), 0)
    path.append(point)


ball1 = chrono.ChBodyEasySphere(vehicle.GetSystem(), 0.5, 1000, True)
ball1.SetPos(chrono.ChVector3d(path[0].x(), path[0].y(), 1))
ball1.SetBodyFixed(True)
vehicle.GetSystem().Add(ball1)

ball2 = chrono.ChBodyEasySphere(vehicle.GetSystem(), 0.5, 1000, True)
ball2.SetPos(chrono.ChVector3d(path[-1].x(), path[-1].y(), 1))
ball2.SetBodyFixed(True)
vehicle.GetSystem().Add(ball2)


steering_Kp = 10.0
steering_Ki = 0.1
steering_Kd = 0.1
steering_pid = chrono.ChPIDController(steering_Kp, steering_Ki, steering_Kd)


throttle = 0.3
target_point = path[0]
sentinel_point = path[0]


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run() :
    time = vehicle.GetSystem().GetChTime()

    
    if (step_number % int(render_step_size / step_size) == 0) :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    vehicle_pos = vehicle.GetChassisBody().GetPos()
    min_distance = float('inf')
    for point in path:
        distance = (point - vehicle_pos).Length()
        if distance < min_distance:
            min_distance = distance
            target_point = point
    sentinel_point = path[(path.index(target_point) + 10) % len(path)]

    
    target_ball = chrono.ChBodyEasySphere(vehicle.GetSystem(), 0.5, 1000, True)
    target_ball.SetPos(chrono.ChVector3d(target_point.x(), target_point.y(), 1))
    target_ball.SetBodyFixed(True)
    vehicle.GetSystem().Add(target_ball)

    sentinel_ball = chrono.ChBodyEasySphere(vehicle.GetSystem(), 0.5, 1000, True)
    sentinel_ball.SetPos(chrono.ChVector3d(sentinel_point.x(), sentinel_point.y(), 1))
    sentinel_ball.SetBodyFixed(True)
    vehicle.GetSystem().Add(sentinel_ball)

    
    error = (target_point - vehicle_pos).Normalize().Dot(chrono.ChVector3d(1, 0, 0))
    steering_input = steering_pid.Update(error, step_size)

    
    vehicle.SetThrottle(throttle)
    vehicle.SetSteering(steering_input)

    
    vehicle.Advance(step_size)
    terrain.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)