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
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                         terrainLength, terrainWidth)
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


radius = 10.0
center = chrono.ChVector3d(50, 0, 0)
num_points = 36  
path_points = []
for i in range(num_points):
    angle = 2 * math.pi * i / num_points
    x = center.x + radius * math.cos(angle)
    y = center.y + radius * math.sin(angle)
    path_points.append(chrono.ChVector3d(x, y, terrainHeight))


Kp = 1.0
Ki = 0.0
Kd = 0.1


integral = 0.0
previous_error = 0.0


path_spheres = []
for point in path_points:
    sphere = chrono.ChBodyEasy()
    sphere.SetPos(point)
    sphere.SetMass(1.0)
    sphere.SetCollide(True)
    sphere.SetBodyFixed(False)
    sphere.GetCollisionModel().ClearModel()
    sphere.GetCollisionModel().AddSphere(0.5)
    sphere.SetVisualizationType(veh.VisualizationType_PRIMITIVES)
    vis.AddBody(sphere)
    path_spheres.append(sphere)


throttle = 0.3


print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())


render_steps = math.ceil(render_step_size / step_size)


s = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    
    current_path_index = step_number % num_points
    target_point = path_points[current_path_index]

    
    vehicle_pos = vehicle.GetVehicle().GetPos()
    error = target_point.x - vehicle_pos.x

    
    integral += error * step_size
    derivative = (error - previous_error) / step_size
    steering = Kp * error + Ki * integral + Kd * derivative
    previous_error = error

    
    steering = max(-0.5, min(steering, 0.5))

    
    driver_inputs = veh.ChDriverInputs()
    driver_inputs.throttle = throttle
    driver_inputs.steering = steering
    driver_inputs.braking = 0.0

    
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1

    
    s.Spin(step_size)