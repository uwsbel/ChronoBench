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
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())


path = veh.WheeledVehiclePath()
path.AddSplinePoint(chrono.ChVector3d(0, 0, 0.5), chrono.ChVector3d(0, 0, 0))
path.AddSplinePoint(chrono.ChVector3d(50, 0, 0.5), chrono.ChVector3d(0, 0, 0))
path.AddSplinePoint(chrono.ChVector3d(100, 0, 0.5), chrono.ChVector3d(0, 0, 0))
path.AddSplinePoint(chrono.ChVector3d(150, 0, 0.5), chrono.ChVector3d(0, 0, 0))
path.AddSplinePoint(chrono.ChVector3d(200, 0, 0.5), chrono.ChVector3d(0, 0, 0))
path.AddSplinePoint(chrono.ChVector3d(200, 50, 0.5), chrono.ChVector3d(0, 0, 0))
path.AddSplinePoint(chrono.ChVector3d(200, 100, 0.5), chrono.ChVector3d(0, 0, 0))
path.AddSplinePoint(chrono.ChVector3d(150, 100, 0.5), chrono.ChVector3d(0, 0, 0))
path.AddSplinePoint(chrono.ChVector3d(100, 100, 0.5), chrono.ChVector3d(0, 0, 0))
path.AddSplinePoint(chrono.ChVector3d(50, 100, 0.5), chrono.ChVector3d(0, 0, 0))
path.AddSplinePoint(chrono.ChVector3d(0, 100, 0.5), chrono.ChVector3d(0, 0, 0))
path.AddSplinePoint(chrono.ChVector3d(0, 50, 0.5), chrono.ChVector3d(0, 0, 0))
path.AddSplinePoint(chrono.ChVector3d(0, 0, 0.5), chrono.ChVector3d(0, 0, 0))


steering_controller = veh.ChWheeledVehiclePIDController()
steering_controller.SetGains(chrono.ChVector3d(1.0, 0.0, 0.0), chrono.ChVector3d(0.0, 0.0, 0.0), chrono.ChVector3d(0.0, 0.0, 0.0))


path_follower = veh.ChWheeledVehiclePathFollower()
path_follower.SetPath(path)
path_follower.SetSteeringController(steering_controller)
path_follower.SetThrottle(0.3)
path_follower.SetBraking(0.0)


path_vis = veh.WheeledVehiclePathVisualizer()
path_vis.SetPath(path)
path_vis.SetColor(chrono.ChColor(1.0, 0.0, 0.0))
vis.AddVisualizer(path_vis)


sentinel_vis = veh.WheeledVehicleSentinelVisualizer()
sentinel_vis.SetPathFollower(path_follower)
sentinel_vis.SetColor(chrono.ChColor(0.0, 1.0, 0.0))
vis.AddVisualizer(sentinel_vis)

target_vis = veh.WheeledVehicleTargetVisualizer()
target_vis.SetPathFollower(path_follower)
target_vis.SetColor(chrono.ChColor(0.0, 0.0, 1.0))
vis.AddVisualizer(target_vis)


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