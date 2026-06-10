import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(50, 0, 0.5)  
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


path_radius = 50.0
path_center = chrono.ChVector3d(50, 0, 0.5)  


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


num_path_points = 100
path = chrono.ChLinePath()
for i in range(num_path_points + 1):
    angle = 2 * math.pi * i / num_path_points
    x = path_center.x() + path_radius * math.cos(angle)
    y = path_center.y() + path_radius * math.sin(angle)
    z = path_center.z()
    path.AddPoint(chrono.ChVector3d(x, y, z))


ball1_radius = 0.5
ball1_visual = chrono.ChVisualShapeSphere(ball1_radius)
ball1_visual.SetColor(chrono.ChColor(1, 0, 0))  
ball1_pos = chrono.ChVector3d(path_center.x() + path_radius, path_center.y(), path_center.z())
ball1_body = chrono.ChBody()
ball1_body.SetPos(ball1_pos)
ball1_body.AddVisualShape(ball1_visual)
vehicle.GetSystem().AddBody(ball1_body)

ball2_radius = 0.5
ball2_visual = chrono.ChVisualShapeSphere(ball2_radius)
ball2_visual.SetColor(chrono.ChColor(0, 0, 1))  
ball2_pos = chrono.ChVector3d(path_center.x() - path_radius, path_center.y(), path_center.z())
ball2_body = chrono.ChBody()
ball2_body.SetPos(ball2_pos)
ball2_body.AddVisualShape(ball2_visual)
vehicle.GetSystem().AddBody(ball2_body)


sentinel_radius = 0.2
sentinel_visual = chrono.ChVisualShapeSphere(sentinel_radius)
sentinel_visual.SetColor(chrono.ChColor(1, 1, 0))  
sentinel_body = chrono.ChBody()
sentinel_body.SetPos(initLoc)
sentinel_body.AddVisualShape(sentinel_visual)
vehicle.GetSystem().AddBody(sentinel_body)

target_radius = 0.2
target_visual = chrono.ChVisualShapeSphere(target_radius)
target_visual.SetColor(chrono.ChColor(0, 1, 1))  
target_body = chrono.ChBody()
target_body.SetPos(initLoc)
target_body.AddVisualShape(target_visual)
vehicle.GetSystem().AddBody(target_body)


vis = veh.ChVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo - Path Following')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())


driver = veh.ChPathFollowerDriver(vehicle.GetVehicle(), path, vehicle.GetSystem())



steering_Kp = 0.5  
steering_Ki = 0.0  
steering_Kd = 0.2  
driver.SetSteeringPID(steering_Kp, steering_Ki, steering_Kd)


driver.SetThrottleDelta(0)  
driver.SetThrottle(0.3)  


braking_time = 0.3
driver.SetBrakingDelta(render_step_size / braking_time)


driver.Initialize()


print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())


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

    
    sentinel_body.SetPos(driver.GetSentinelPos())
    target_body.SetPos(driver.GetTargetPos())

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)