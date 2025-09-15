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


patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(chrono.ChSystem())
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


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


radius = 10.0
center = chrono.ChVector3d(0, 0, 0.5)
num_points = 100
points = []
for i in range(num_points):
    theta = 2 * math.pi * i / (num_points - 1)
    x = center.x + radius * math.cos(theta)
    y = center.y + radius * math.sin(theta)
    points.append(chrono.ChVector3d(x, y, center.z))
path = chrono.ChBezierCurve(points)


path_center = chrono.ChBodyEasySphere(0.5, 1000, True, True)
path_center.SetPos(center)
path_center.AddVisualShape(chrono.ChVisualShapeSphere(0.5))
path_center.GetVisualShape(0).SetMaterial(0, chrono.ChVisualMaterial().SetDiffuseColor(chrono.ChColor(1, 0, 0)))
vehicle.GetSystem().Add(path_center)

path_edge = chrono.ChBodyEasySphere(0.5, 1000, True, True)
path_edge.SetPos(chrono.ChVector3d(center.x + radius, center.y, center.z))
path_edge.AddVisualShape(chrono.ChVisualShapeSphere(0.5))
path_edge.GetVisualShape(0).SetMaterial(0, chrono.ChVisualMaterial().SetDiffuseColor(chrono.ChColor(0, 1, 0)))
vehicle.GetSystem().Add(path_edge)


steering_controller = veh.ChPIDController()
steering_controller.SetGains(0.5, 0.0, 0.1)


driver = veh.ChPathFollowerDriver(vehicle.GetVehicle(), path, "circular_path", 0.0, True)
driver.SetLateralController(steering_controller)
driver.Initialize()


sentinel_ball = chrono.ChBodyEasySphere(0.2, 1000, True, True)
sentinel_ball.AddVisualShape(chrono.ChVisualShapeSphere(0.2))
sentinel_ball.GetVisualShape(0).SetMaterial(0, chrono.ChVisualMaterial().SetDiffuseColor(chrono.ChColor(1, 0, 0)))
vehicle.GetSystem().Add(sentinel_ball)

target_ball = chrono.ChBodyEasySphere(0.2, 1000, True, True)
target_ball.AddVisualShape(chrono.ChVisualShapeSphere(0.2))
target_ball.GetVisualShape(0).SetMaterial(0, chrono.ChVisualMaterial().SetDiffuseColor(chrono.ChColor(0, 0, 1)))
vehicle.GetSystem().Add(target_ball)


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())


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

    
    driver_inputs = driver.GetInputs()
    driver_inputs.m_throttle = 0.3  

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    sentinel_pos = driver.GetLateralController().GetSentinelPosition()
    target_pos = driver.GetLateralController().GetTargetPosition()
    sentinel_ball.SetPos(sentinel_pos)
    target_ball.SetPos(target_pos)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)