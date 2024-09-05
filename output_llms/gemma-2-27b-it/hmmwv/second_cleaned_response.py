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


path_radius = 20.0  
target_speed = 5.0  


Kp = 1.0  
Ki = 0.1  
Kd = 0.01  


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


sentinel_sphere = irr.ChIrrNodeSphere(0.5, vis.GetSceneManager())
sentinel_sphere.SetPos(chrono.ChVector3d(path_radius, 0, 0))
vis.AddNode(sentinel_sphere)

target_sphere = irr.ChIrrNodeSphere(0.5, vis.GetSceneManager())
target_sphere.SetPos(chrono.ChVector3d(path_radius, 0, 0))
vis.AddNode(target_sphere)


previous_error = 0.0
integral_error = 0.0


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    
    if (step_number % math.ceil(render_step_size / step_size) == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    
    angle = vehicle.GetVehicle().GetPos().x() / path_radius
    target_x = path_radius * math.cos(angle)
    target_y = path_radius * math.sin(angle)
    target_point = chrono.ChVector3d(target_x, target_y, 0)

    
    target_sphere.SetPos(target_point)

    
    error = (target_point - vehicle.GetVehicle().GetPos()).Length()

    
    integral_error += error * step_size
    derivative_error = (error - previous_error) / step_size
    steering_input = Kp * error + Ki * integral_error + Kd * derivative_error

    
    vehicle.GetVehicle().SetSteeringAngle(steering_input)

    
    vehicle.GetVehicle().SetThrottle(0.3)

    
    previous_error = error

    
    terrain.Synchronize(time)
    vehicle.Synchronize(time)
    vis.Synchronize(time)

    
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)