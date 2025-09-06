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


path_radius = 20.0
path_center = chrono.ChVector3d(0, 0, 0)
path_follower = veh.ChPathFollowerDriverSmooth(vehicle.GetVehicle(), path_radius, path_center)
path_follower.SetSteeringControllerGain(1.0)
path_follower.SetThrottleControllerGain(0.1)
path_follower.SetThrottleValue(0.3)
path_follower.Initialize()


path_ball1 = chrono.ChBodyEasySphere(0.5, 1000, True, True)
path_ball1.SetPos(chrono.ChVector3d(path_center.x() + path_radius, path_center.y(), path_center.z() + 0.5))
path_ball1.SetBodyFixed(True)
vehicle.GetSystem().Add(path_ball1)

path_ball2 = chrono.ChBodyEasySphere(0.5, 1000, True, True)
path_ball2.SetPos(chrono.ChVector3d(path_center.x() - path_radius, path_center.y(), path_center.z() + 0.5))
path_ball2.SetBodyFixed(True)
vehicle.GetSystem().Add(path_ball2)


sentinel_point = chrono.ChBodyEasySphere(0.2, 1000, True, True)
sentinel_point.SetPos(chrono.ChVector3d(path_center.x(), path_center.y() + path_radius, path_center.z() + 0.5))
sentinel_point.SetBodyFixed(True)
vehicle.GetSystem().Add(sentinel_point)

target_point = chrono.ChBodyEasySphere(0.2, 1000, True, True)
target_point.SetPos(chrono.ChVector3d(path_center.x(), path_center.y() - path_radius, path_center.z() + 0.5))
target_point.SetBodyFixed(True)
vehicle.GetSystem().Add(target_point)


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

    
    path_follower.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, path_follower.GetInputs(), terrain)
    vis.Synchronize(time, path_follower.GetInputs())

    
    path_follower.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)