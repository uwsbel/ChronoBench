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


radius = 30.0  
path = veh.ChBezierCurve()
center = chrono.ChVector3d(0, 0, 0.1)  


num_points = 36
for i in range(num_points + 1):
    angle = i * 2 * math.pi / num_points
    x = center.x + radius * math.cos(angle)
    y = center.y + radius * math.sin(angle)
    z = center.z
    path.AddPoint(chrono.ChVector3d(x, y, z))


driver = veh.ChPathFollowerDriver(vehicle.GetVehicle(), path, "my_path", 10.0)  
driver.GetSteeringController().SetLookAheadDistance(5.0)
driver.GetSteeringController().SetGains(0.5, 0.0, 0.0)  
driver.GetSpeedController().SetGains(0.4, 0.0, 0.0)  
driver.Initialize()


constant_throttle = 0.3
driver.SetThrottle(constant_throttle)



sentinel_body = chrono.ChBodyEasySphere(0.5, 1000, vis_type != veh.VisualizationType_NONE)
sentinel_body.SetBodyFixed(True)
sentinel_body.SetCollide(False)
sentinel_body.AddVisualShapeSphere(0.5, chrono.ChVector3d(0, 0, 0), chrono.ChQuaternion(1, 0, 0, 0), chrono.ChColor(1, 0, 0))
vehicle.GetSystem().Add(sentinel_body)

target_body = chrono.ChBodyEasySphere(0.5, 1000, vis_type != veh.VisualizationType_NONE)
target_body.SetBodyFixed(True)
target_body.SetCollide(False)
target_body.AddVisualShapeSphere(0.5, chrono.ChVector3d(0, 0, 0), chrono.ChQuaternion(1, 0, 0, 0), chrono.ChColor(0, 1, 0))
vehicle.GetSystem().Add(target_body)


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

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    sentinel_pos = driver.GetSteeringController().GetSentinelLocation()
    target_pos = driver.GetSteeringController().GetTargetLocation()
    sentinel_body.SetPos(sentinel_pos)
    target_body.SetPos(target_pos)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)