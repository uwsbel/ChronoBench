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


vehicle = veh.HMMWV_Full() 
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(1e-3)
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
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())


path_radius = 50.0
path_center = chrono.ChVector3d(0, 0, 0)


ball1 = chrono.ChBodyEasySphere(vehicle.GetSystem(), 1.0, 1000, chrono.ChVector3d(path_center.x + path_radius, path_center.y, path_center.z + 1.0), chrono.ChColor(1, 0, 0))
ball2 = chrono.ChBodyEasySphere(vehicle.GetSystem(), 1.0, 1000, chrono.ChVector3d(path_center.x - path_radius, path_center.y, path_center.z + 1.0), chrono.ChColor(0, 1, 0))
vehicle.GetSystem().Add(ball1)
vehicle.GetSystem().Add(ball2)


Kp = 10.0
Ki = 0.1
Kd = 0.1


target_point = chrono.ChVector3d(path_center.x + path_radius, path_center.y, path_center.z)


sentinel_point = chrono.ChVector3d(path_center.x + path_radius, path_center.y, path_center.z)


target_sphere = chrono.ChBodyEasySphere(vehicle.GetSystem(), 1.0, 1000, target_point, chrono.ChColor(0, 0, 1))
sentinel_sphere = chrono.ChBodyEasySphere(vehicle.GetSystem(), 1.0, 1000, sentinel_point, chrono.ChColor(1, 1, 0))
vehicle.GetSystem().Add(target_sphere)
vehicle.GetSystem().Add(sentinel_sphere)


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 50  


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0


throttle = 0.3

while vis.Run() :
    time = vehicle.GetSystem().GetChTime()

    
    if (step_number % math.ceil(render_step_size / step_size) == 0) :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    target_point = chrono.ChVector3d(path_center.x + path_radius * math.cos(time), path_center.y + path_radius * math.sin(time), path_center.z)
    target_sphere.SetPos(target_point)

    
    sentinel_point = chrono.ChVector3d(path_center.x + path_radius * math.cos(time + math.pi / 2), path_center.y + path_radius * math.sin(time + math.pi / 2), path_center.z)
    sentinel_sphere.SetPos(sentinel_point)

    
    error = (target_point - vehicle.GetChassisBody().GetPos()).Length()
    steering_angle = Kp * error + Ki * error * step_size + Kd * (error - (target_point - vehicle.GetChassisBody().GetPos()).Length()) / step_size

    
    vehicle.GetSteering().SetSteering(steering_angle)
    vehicle.GetTransmission().SetThrottle(throttle)

    
    vehicle.Advance(step_size)
    terrain.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)