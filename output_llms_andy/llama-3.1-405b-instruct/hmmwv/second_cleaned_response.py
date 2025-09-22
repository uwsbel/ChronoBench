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



path_radius = 20.0
num_points = 100
path_points = []
for i in range(num_points):
    angle = 2 * math.pi * i / num_points
    x = path_radius * math.cos(angle)
    y = path_radius * math.sin(angle)
    path_points.append(chrono.ChVector3d(x, y, 0))


path_balls = []
for i in range(num_points):
    ball = chrono.ChSphereShape()
    ball.GetSphereGeometry().rad = 0.2
    ball.SetPos(path_points[i])
    vis.AddShape(ball)
    path_balls.append(ball)


throttle_value = 0.3
pid_gains = chrono.ChVector3d(0.1, 0.01, 0.001)  


while vis.Run() :
    time = vehicle.GetSystem().GetChTime()

    
    if (time % render_step_size < step_size) :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    
    vehicle_pos = vehicle.GetVehicle().GetPos()
    vehicle_vel = vehicle.GetVehicle().GetPos_dt()

    
    closest_point = None
    closest_distance = float('inf')
    for i in range(num_points):
        distance = (vehicle_pos - path_points[i]).Length()
        if distance < closest_distance:
            closest_distance = distance
            closest_point = path_points[i]

    
    steering_error = (closest_point - vehicle_pos).Length()

    
    pid_output = pid_gains.x * steering_error + pid_gains.y * steering_error / step_size + pid_gains.z * steering_error / step_size**2

    
    vehicle.GetVehicle().SetThrottle(throttle_value)
    vehicle.GetVehicle().SetSteering(pid_output)

    
    vehicle.Advance(step_size)
    terrain.Advance(step_size)
    vis.Advance(step_size)

    
    time += step_size

    
    chrono.ChRealtimeStepTimer().Spin(step_size)