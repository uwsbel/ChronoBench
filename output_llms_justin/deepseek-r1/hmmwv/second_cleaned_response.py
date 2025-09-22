import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


terrainHeight = 0      
terrainLength = 200.0  
terrainWidth = 100.0   


path_radius = 20.0
path_center = chrono.ChVector3d(0, 0, 0.5)
initLoc = chrono.ChVector3d(path_radius, 0, 0.5)  
initRot = chrono.Q_from_AngZ(math.pi/2)  


vis_type = veh.VisualizationType_PRIMITIVES
chassis_collision_type = veh.CollisionType_NONE
tire_model = veh.TireModelType_TMEASY


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
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                         terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Circular Path Following')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())


npoints = 50
points = []
for i in range(npoints):
    angle = i * 2 * math.pi / (npoints - 1)
    x = path_center.x + path_radius * math.cos(angle)
    y = path_center.y + path_radius * math.sin(angle)
    points.append(chrono.ChVector3d(x, y, path_center.z))
path = chrono.ChBezierCurve(points)


driver = veh.ChPathFollowerDriver(vehicle.GetVehicle(), path, "circular_path", 0.0, 10.0)
driver.GetSteeringController().SetGains(0.5, 0.0, 0.0)  
driver.Initialize()


ball_radius = 0.5
ball1 = chrono.ChBodyEasySphere(ball_radius, 1000, True, True)
ball1.SetPos(chrono.ChVector3d(path_radius, 0, path_center.z))
ball1.SetFixed(True)
ball1.GetVisualShape(0).SetColor(chrono.ChColor(1, 0, 0))  
vehicle.GetSystem().Add(ball1)

ball2 = chrono.ChBodyEasySphere(ball_radius, 1000, True, True)
ball2.SetPos(chrono.ChVector3d(0, path_radius, path_center.z))
ball2.SetFixed(True)
ball2.GetVisualShape(0).SetColor(chrono.ChColor(0, 1, 0))  
vehicle.GetSystem().Add(ball2)


sentinel_ball = chrono.ChBodyEasySphere(0.3, 1000, True, True)
sentinel_ball.SetPos(path_center)
sentinel_ball.SetFixed(True)
sentinel_ball.GetVisualShape(0).SetColor(chrono.ChColor(0, 0, 1))  
vehicle.GetSystem().Add(sentinel_ball)

target_ball = chrono.ChBodyEasySphere(0.3, 1000, True, True)
target_ball.SetPos(path_center)
target_ball.SetFixed(True)
target_ball.GetVisualShape(0).SetColor(chrono.ChColor(1, 1, 0))  
vehicle.GetSystem().Add(target_ball)


print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())


render_steps = math.ceil(render_step_size / step_size)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    driver.Synchronize(time)
    driver_inputs = driver.GetInputs()
    driver_inputs.m_throttle = 0.3  
    
    
    sentinel = driver.GetSteeringController().GetSentinelLocation()
    target = driver.GetSteeringController().GetTargetLocation()
    sentinel_ball.SetPos(sentinel)
    target_ball.SetPos(target)

    
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)