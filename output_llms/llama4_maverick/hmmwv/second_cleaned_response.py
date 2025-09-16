import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import numpy as np


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


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
patch = terrain.AddPatch(patch_mat,chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrainHeight), chrono.QUNIT),terrainLength, terrainWidth)
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


path_radius = 30
path_center = chrono.ChVector3d(0, 0, 0)
num_path_points = 100
path = []
for i in range(num_path_points):
    angle = 2 * np.pi * i / num_path_points
    point = path_center + chrono.ChVector3d(path_radius * np.cos(angle), path_radius * np.sin(angle), 0)
    path.append(point)


ball1 = chrono.ChBodyEasySphere(1, 1000, True, False)
ball1.SetBodyFixed(True)
ball1.SetPos(path[0])
vis.GetSceneManager().getActiveCamera().addChild(ball1)

ball2 = chrono.ChBodyEasySphere(1, 1000, True, False)
ball2.SetBodyFixed(True)
ball2.SetPos(path[num_path_points//2])
vis.GetSceneManager().getActiveCamera().addChild(ball2)


Kp = 0.5
Ki = 0.0
Kd = 0.0

driver = veh.ChPathFollowerDriver(vehicle.GetVehicle(), path, 'my_path', 8.0, True, Kp, Ki, Kd)
driver.SetSteeringDelta(0.02)
driver.SetThrottle(0.3)


print( "VEHICLE MASS: ", vehicle.GetVehicle().GetMass())


render_steps = int(render_step_size / step_size)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run() :
    time = vehicle.GetSystem().GetChTime()

    
    if (step_number % render_steps == 0) :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    sentinel = driver.GetSentinelLocation()
    target = driver.GetTargetLocation()
    sentinel_ball = chrono.ChBodyEasySphere(0.5, 1000, True, False)
    sentinel_ball.SetBodyFixed(True)
    sentinel_ball.SetPos(sentinel)
    vis.GetSceneManager().getActiveCamera().addChild(sentinel_ball)
    target_ball = chrono.ChBodyEasySphere(0.5, 1000, True, False)
    target_ball.SetBodyFixed(True)
    target_ball.SetPos(target)
    vis.GetSceneManager().getActiveCamera().addChild(target_ball)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)