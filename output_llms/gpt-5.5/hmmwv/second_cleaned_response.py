import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')






terrainHeight = 0
terrainLength = 200.0   
terrainWidth = 100.0


path_radius = 35.0
path_center = chrono.ChVector3d(0.0, 0.0, terrainHeight + 0.1)



initLoc = chrono.ChVector3d(path_radius, 0.0, 0.5)
initYaw = math.pi / 2.0
initRot = chrono.ChQuaterniond(
    math.cos(initYaw / 2.0),
    0.0,
    0.0,
    math.sin(initYaw / 2.0)
)


vis_type = veh.VisualizationType_PRIMITIVES


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_TMEASY


trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)


contact_method = chrono.ChContactMethod_NSC


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 50.0


constant_throttle = 0.3
target_speed = 12.0          
lookahead_distance = 5.0


steering_Kp = 0.8
steering_Ki = 0.0
steering_Kd = 0.15





def create_visual_sphere(system, pos, radius, color):
    body = chrono.ChBody()
    body.SetFixed(True)
    body.SetPos(pos)

    sphere = chrono.ChVisualShapeSphere(radius)
    sphere.SetColor(color)
    body.AddVisualShape(sphere)

    system.Add(body)
    return body






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

patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(
        chrono.ChVector3d(0.0, 0.0, terrainHeight),
        chrono.ChQuaterniond(1.0, 0.0, 0.0, 0.0)
    ),
    terrainLength,
    terrainWidth
)

patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

terrain.Initialize()






try:
    path = veh.CirclePath(path_center, path_radius, 200, True)
except TypeError:
    path = veh.CirclePath(path_center, path_radius, 200)


path_ball_1 = create_visual_sphere(
    vehicle.GetSystem(),
    chrono.ChVector3d(path_radius, 0.0, terrainHeight + 0.35),
    0.35,
    chrono.ChColor(0.0, 0.0, 1.0)
)

path_ball_2 = create_visual_sphere(
    vehicle.GetSystem(),
    chrono.ChVector3d(-path_radius, 0.0, terrainHeight + 0.35),
    0.35,
    chrono.ChColor(0.0, 0.0, 1.0)
)


sentinel_sphere = create_visual_sphere(
    vehicle.GetSystem(),
    initLoc,
    0.30,
    chrono.ChColor(1.0, 0.0, 0.0)
)

target_sphere = create_visual_sphere(
    vehicle.GetSystem(),
    initLoc,
    0.30,
    chrono.ChColor(0.0, 1.0, 0.0)
)





driver = veh.ChPathFollowerDriver(
    vehicle.GetVehicle(),
    path,
    "circular_path",
    target_speed
)

driver.GetSteeringController().SetLookAheadDistance(lookahead_distance)
driver.GetSteeringController().SetGains(steering_Kp, steering_Ki, steering_Kd)



driver.GetSpeedController().SetGains(0.4, 0.0, 0.0)

driver.Initialize()





vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Circular Path Following')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())





print("VEHICLE MASS:", vehicle.GetVehicle().GetMass())


render_steps = math.ceil(render_step_size / step_size)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0





while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    
    driver.Synchronize(time)

    
    controller_inputs = driver.GetInputs()

    driver_inputs = veh.DriverInputs()
    driver_inputs.m_steering = controller_inputs.m_steering
    driver_inputs.m_throttle = constant_throttle
    driver_inputs.m_braking = 0.0

    
    sentinel_sphere.SetPos(driver.GetSteeringController().GetSentinelLocation())
    target_sphere.SetPos(driver.GetSteeringController().GetTargetLocation())

    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)