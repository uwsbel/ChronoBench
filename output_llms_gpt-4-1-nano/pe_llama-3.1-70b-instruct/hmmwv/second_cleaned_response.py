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
vis.AddLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())



driver = veh.ChPathFollowerDriver()


path = chrono.ChBezierCurve()
path.AddBezierPoint(chrono.ChVector3d(0, 0, 0))
path.AddBezierPoint(chrono.ChVector3d(10, 0, 0))
path.AddBezierPoint(chrono.ChVector3d(20, 0, 0))
path.AddBezierPoint(chrono.ChVector3d(30, 0, 0))
driver.SetPath(path)


driver.SetSpeed(5.0)


driver.SetPIDGains(1.0, 1.0, 1.0)


driver.Initialize()


print( "VEHICLE MASS: ",  vehicle.GetVehicle().GetMass())


render_steps = math.ceil(render_step_size / step_size)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0


path_vis = chrono.ChVisualShapeLine()
path_vis.SetColor(chrono.ChColor(1, 0, 0))
vis.AddVisualShape(path_vis)


sentinel_vis = chrono.ChVisualShapeSphere()
sentinel_vis.SetColor(chrono.ChColor(0, 1, 0))
sentinel_vis.SetRadius(0.5)
vis.AddVisualShape(sentinel_vis)

target_vis = chrono.ChVisualShapeSphere()
target_vis.SetColor(chrono.ChColor(0, 0, 1))
target_vis.SetRadius(0.5)
vis.AddVisualShape(target_vis)

while vis.Run() :
    time = vehicle.GetSystem().GetChTime()

    
    if (step_number % render_steps == 0) :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver.GetInputs(), terrain)
    vis.Synchronize(time, driver.GetInputs())

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    path_vis.Clear()
    for i in range(100):
        point = path.Evaluate(i/100.0)
        path_vis.AddLineSegment(point, chrono.ChVector3d(0, 0, 0), chrono.ChColor(1, 0, 0))

    
    sentinel_point = driver.GetSentinelPoint()
    sentinel_vis.SetPosition(sentinel_point)
    target_point = driver.GetTargetPoint()
    target_vis.SetPosition(target_point)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)