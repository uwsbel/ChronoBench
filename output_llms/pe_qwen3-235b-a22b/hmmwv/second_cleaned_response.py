import pychrono as chrono
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


radius = 20.0
path = chrono.ChPath()
num_points = 100
for i in range(num_points + 1):
    angle = 2 * math.pi * i / num_points
    x = radius * math.cos(angle)
    y = radius * math.sin(angle)
    path.AddPoint(chrono.ChVector3d(x, y, 0.5))


sys = vehicle.GetSystem()


ball1 = chrono.ChBody()
ball1.SetPos(chrono.ChVector3d(radius, 0, 0.5))
ball1.SetFixed(True)
ball1.AddVisualShape(chrono.ChVisualShapeSphere(0.5, chrono.ChColor(1, 0, 0)))
sys.Add(ball1)

ball2 = chrono.ChBody()
ball2.SetPos(chrono.ChVector3d(0, radius, 0.5))
ball2.SetFixed(True)
ball2.AddVisualShape(chrono.ChVisualShapeSphere(0.5, chrono.ChColor(0, 0, 1)))
sys.Add(ball2)


sentinel_sphere = chrono.ChBody()
sentinel_sphere.SetFixed(True)
sentinel_sphere.AddVisualShape(chrono.ChVisualShapeSphere(0.3, chrono.ChColor(0, 1, 0)))
sys.Add(sentinel_sphere)

target_sphere = chrono.ChBody()
target_sphere.SetFixed(True)
target_sphere.AddVisualShape(chrono.ChVisualShapeSphere(0.3, chrono.ChColor(0, 0, 1)))
sys.Add(target_sphere)


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


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Path Following Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())


driver = veh.ChPathFollowerDriver(vehicle, path)
steering_controller = driver.GetSteeringController()
steering_controller.SetLookAheadDistance(5.0)  
steering_controller.SetGains(0.8, 0.0, 0.01)   
driver.SetDesiredSpeed(10.0)                   


driver.Initialize()


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
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver.GetInputs(), terrain)
    vis.Synchronize(time, driver.GetInputs())

    
    driver_inputs = driver.GetInputs()
    driver_inputs.m_throttle = 0.3  

    
    if hasattr(steering_controller, 'GetSentinelPoint') and hasattr(steering_controller, 'GetTargetPoint'):
        sentinel_sphere.SetPos(steering_controller.GetSentinelPoint())
        target_sphere.SetPos(steering_controller.GetTargetPoint())

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)