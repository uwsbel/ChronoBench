import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


path_radius = 20.0


initLoc = chrono.ChVector3d(path_radius, 0, 0.5)
initRot = chrono.QuatFromAngleAxis(math.pi / 2, chrono.ChVector3d(0, 0, 1))


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
patch = terrain.AddPatch(patch_mat,
                         chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                         terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()




num_points = 100
path_points = chrono.vector_ChVector3d()
for i in range(num_points):
    angle = 2.0 * math.pi * i / num_points
    x = path_radius * math.cos(angle)
    y = path_radius * math.sin(angle)
    path_points.push_back(chrono.ChVector3d(x, y, 0.5))


path = chrono.ChBezierCurve(path_points, True)  




vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Circular Path Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())





ball1 = chrono.ChBody()
ball1.SetPos(chrono.ChVector3d(path_radius, 0, 0.5))
ball1.SetFixed(True)
ball1_shape = chrono.ChVisualShapeSphere(1.0)
ball1_shape.SetColor(chrono.ChColor(1, 0.4, 0))
ball1.AddVisualShape(ball1_shape)
vehicle.GetSystem().Add(ball1)

ball2 = chrono.ChBody()
ball2.SetPos(chrono.ChVector3d(-path_radius, 0, 0.5))
ball2.SetFixed(True)
ball2_shape = chrono.ChVisualShapeSphere(1.0)
ball2_shape.SetColor(chrono.ChColor(1, 0.4, 0))
ball2.AddVisualShape(ball2_shape)
vehicle.GetSystem().Add(ball2)




target_speed = 8.0  

driver = veh.ChPathFollowerDriver(vehicle.GetVehicle(), path, "circular_path", target_speed)


steering_Kp = 0.4
steering_Ki = 0.0
steering_Kd = 0.0
driver.GetSteeringController().SetLookAheadDistance(5.0)
driver.GetSteeringController().SetGains(steering_Kp, steering_Ki, steering_Kd)


driver.GetSpeedController().SetGains(0.4, 0.0, 0.0)

driver.Initialize()


throttle_value = 0.3




sentinel_body = chrono.ChBody()
sentinel_body.SetFixed(True)
sentinel_shape = chrono.ChVisualShapeSphere(0.4)
sentinel_shape.SetColor(chrono.ChColor(1, 0, 0))   
sentinel_body.AddVisualShape(sentinel_shape)
vehicle.GetSystem().Add(sentinel_body)

target_body = chrono.ChBody()
target_body.SetFixed(True)
target_shape = chrono.ChVisualShapeSphere(0.4)
target_shape.SetColor(chrono.ChColor(0, 1, 0))     
target_body.AddVisualShape(target_shape)
vehicle.GetSystem().Add(target_body)


print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())


render_steps = math.ceil(render_step_size / step_size)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    
    if (step_number % render_steps == 0):
        
        sentinel_body.SetPos(driver.GetSteeringController().GetSentinelLocation())
        target_body.SetPos(driver.GetSteeringController().GetTargetLocation())

        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    driver_inputs = driver.GetInputs()

    
    driver_inputs.m_throttle = throttle_value
    driver_inputs.m_braking = 0.0

    
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