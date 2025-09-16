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


radius = 20.0  
path_points = []
for i in range(360):
    angle = i * math.pi / 180
    x = radius * math.cos(angle)
    y = radius * math.sin(angle)
    path_points.append(chrono.ChVector3d(x, y, 0))

sentinel_point = path_points[0]
target_point = path_points[1]


sentinel_sphere = irr.ChSphereShape()
sentinel_sphere.GetMaterial().SetDiffuseColor(chrono.ChColor(1, 0, 0))
sentinel_sphere.GetMaterial().SetSpecularColor(chrono.ChColor(1, 0, 0))
sentinel_sphere.SetRadius(0.2)
sentinel_vis = irr.ChBodyEasyBox(1, 1, 1, 1000)
sentinel_vis.AddShape(sentinel_sphere)
vis.GetSystem().Add(sentinel_vis)
sentinel_vis.SetPos(sentinel_point)

target_sphere = irr.ChSphereShape()
target_sphere.GetMaterial().SetDiffuseColor(chrono.ChColor(0, 0, 1))
target_sphere.GetMaterial().SetSpecularColor(chrono.ChColor(0, 0, 1))
target_sphere.SetRadius(0.2)
target_vis = irr.ChBodyEasyBox(1, 1, 1, 1000)
target_vis.AddShape(target_sphere)
vis.GetSystem().Add(target_vis)
target_vis.SetPos(target_point)


Kp = 1.0
Ki = 0.1
Kd = 0.01

integral = 0.0
previous_error = 0.0


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run() :
    time = vehicle.GetSystem().GetChTime()

    
    if (step_number % math.ceil(render_step_size / step_size) == 0) :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    current_pos = vehicle.GetVehicle().GetPos()
    error = (target_point - current_pos).Length()
    integral += error * step_size
    derivative = (error - previous_error) / step_size
    steering = Kp * error + Ki * integral + Kd * derivative
    vehicle.GetVehicle().SetSteeringAngle(steering)

    
    sentinel_point = target_point
    target_point = path_points[(path_points.index(target_point) + 1) % len(path_points)]
    sentinel_vis.SetPos(sentinel_point)
    target_vis.SetPos(target_point)

    
    terrain.Synchronize(time)
    vehicle.Synchronize(time, chrono.ChVector3d(0.3, 0, 0), terrain)
    vis.Synchronize(time)

    
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)