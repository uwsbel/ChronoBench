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


patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()




circle_radius = 30.0  
circle_center = chrono.ChVector2d(0, 0)




path_points = []
N_points = 100
for i in range(N_points):
    angle = 2 * math.pi * i / N_points
    x = circle_radius * math.cos(angle)
    y = circle_radius * math.sin(angle)
    path_points.append(chrono.ChVector2d(x, y))


path = veh.ChBezierCurve()
for i in range(N_points):
    
    
    pass




path_line = veh.ChLinePath()

for pt in path_points:
    path_line.AddPoint(pt)

path_line.SetClosed(True)
path_line.SetLength(circle_radius * 2 * math.pi)  


driver = veh.ChPathFollowerDriver(vehicle, path_line, "my_path_follower")
driver.SetLookAheadDistance(5.0)       
driver.SetLookAheadSteps(15)            
driver.SetGains(0.8, 0, 0)              
driver.SetTargetSpeed(5.0)              
driver.Initialize()


constant_throttle = 0.3




vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo with Path Following')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())





sphere_radius = 0.3

sphere1_pos = chrono.ChVectorD(circle_radius, 0, 0.5)
sphere2_pos = chrono.ChVectorD(-circle_radius, 0, 0.5)

def create_visual_sphere(pos, radius, color):
    sphere = chrono.ChBodyEasySphere(radius, 1000, True, True)
    sphere.SetPos(pos)
    sphere.SetBodyFixed(True)
    sphere.GetVisualShape(0).SetColor(color)
    return sphere

sphere1 = create_visual_sphere(sphere1_pos, sphere_radius, chrono.ChColor(0, 1, 0))
sphere2 = create_visual_sphere(sphere2_pos, sphere_radius, chrono.ChColor(0, 1, 0))

vehicle.GetSystem().Add(sphere1)
vehicle.GetSystem().Add(sphere2)




target_sphere = create_visual_sphere(chrono.ChVectorD(0, 0, 10), sphere_radius, chrono.ChColor(1, 0, 0))  
vehicle.GetSystem().Add(target_sphere)

sentinel_sphere = create_visual_sphere(chrono.ChVectorD(0, 0, 10), sphere_radius, chrono.ChColor(0, 0, 1))  
vehicle.GetSystem().Add(sentinel_sphere)


print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())


render_steps = math.ceil(render_step_size / step_size)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():

    time = vehicle.GetSystem().GetChTime()

    
    
    

    driver.Synchronize(time)
    terrain.Synchronize(time)
    
    
    inputs = driver.GetInputs()
    inputs.m_throttle = constant_throttle
    inputs.m_braking = 0.0
    
    
    vehicle.Synchronize(time, inputs, terrain)
    vis.Synchronize(time, inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    

    
    target_point = driver.GetTargetPoint()
    sentinel_point = driver.GetSentinelPoint()

    
    if target_point is not None:
        target_sphere.SetPos(chrono.ChVectorD(target_point.x, target_point.y, 0.5))

    if sentinel_point is not None:
        sentinel_sphere.SetPos(chrono.ChVectorD(sentinel_point.x, sentinel_point.y, 0.5))

    
    if (step_number % render_steps) == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    step_number += 1

    
    realtime_timer.Spin(step_size)