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


path = veh.WheeledVehiclePathFollowerDriver()
path.SetPathType(veh.PathType_CIRCULAR)
path.SetRadius(20.0)
path.SetCenter(chrono.ChVector3d(0, 0, 0))
path.SetLookAheadDistance(5.0)
path.SetGain(0.5)
path.SetMass(1500.0)
path.SetThrottle(0.3)
path.SetBrakingDistance(10.0)
path.SetTolerance(0.5)
path.SetMaxRecoveryTime(10.0)
path.SetSynchronizationMode(veh.PathFollowerSynchronizationMode_SLIP velocity)
path.SetWheelbase(vehicle.GetWheelbase())
path.SetTrackwidth(vehicle.GetTrackwidth())
path.SetCGtoWheelbase(vehicle.GetCGtoWheelbase())
path.SetSteeringHardpointsToWheelcenterDistance(vehicle.GetSteeringHardpointsToWheelcenterDistance())
path.SetWheelRadius(vehicle.GetWheelRadius())
path.SetMass(vehicle.GetVehicle().GetMass())
path.Initialize()


pid_controller = veh.ChSteeringPIDController()
pid_controller.SetGains(2.0, 0.5, 0.1)
pid_controller.SetLookAheadDistance(5.0)
pid_controller.SetMass(1500.0)
pid_controller.SetWheelbase(vehicle.GetWheelbase())
pid_controller.SetTrackwidth(vehicle.GetTrackwidth())
pid_controller.SetCGtoWheelbase(vehicle.GetCGtoWheelbase())
pid_controller.SetSteeringHardpointsToWheelcenterDistance(vehicle.GetSteeringHardpointsToWheelcenterDistance())
pid_controller.SetWheelRadius(vehicle.GetWheelRadius())
pid_controller.SetMass(vehicle.GetVehicle().GetMass())
pid_controller.Initialize()


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

    
    path.Synchronize(time)
    pid_controller.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, pid_controller.GetSteering(), terrain)
    vis.Synchronize(time, pid_controller.GetSteering())

    
    path.Advance(step_size)
    pid_controller.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    sentinel_point = path.GetSentinelPoint()
    target_point = path.GetTargetPoint()
    vis.AddSphereMarker(sentinel_point, 0.5, chrono.ChColor(1, 0, 0))
    vis.AddSphereMarker(target_point, 0.5, chrono.ChColor(0, 1, 0))

    
    step_number += 1

    
    realtime_timer.Spin(step_size)