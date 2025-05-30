import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math


veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


R = 30.0  
initLoc = chrono.ChVector3d(R, 0, 0.5)  
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_PRIMITIVES


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_TMEASY


terrainHeight = 0
terrainLength = 200.0  
terrainWidth = 100.0


trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)


contact_method = chrono.ChContactMethod_NSC
step_size = 1e-3
tire_step_size = step_size
render_step_size = 1.0 / 50  


Kp = 0.5
Ki = 0.0
Kd = 0.1
integral = 0.0
prev_error = 0.0


vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)


vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

vehicle.Initialize()

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat, 
                        chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                        terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Circular Path Following')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())


ball_radius = 0.5
for angle in [0, math.pi]:
    ball = chrono.ChBodyEasySphere(ball_radius, 1000, True, True)
    x = R * math.cos(angle)
    y = R * math.sin(angle)
    ball.SetPos(chrono.ChVector3d(x, y, 0.5))
    ball.SetFixed(True)
    ball.GetVisualShape(0).SetColor(chrono.ChColor(1, 0, 0))
    vehicle.GetSystem().Add(ball)


sentinel = chrono.ChBodyEasySphere(0.3, 1000, True, True)
sentinel.SetFixed(True)
sentinel.GetVisualShape(0).SetColor(chrono.ChColor(0, 0, 1))
vehicle.GetSystem().Add(sentinel)

target = chrono.ChBodyEasySphere(0.3, 1000, True, True)
target.SetFixed(True)
target.GetVisualShape(0).SetColor(chrono.ChColor(1, 1, 0))
vehicle.GetSystem().Add(target)


render_steps = math.ceil(render_step_size / step_size)
step_number = 0
render_frame = 0
realtime_timer = chrono.ChRealtimeStepTimer()

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    pos = vehicle.GetVehicle().GetPos()
    current_radius = math.sqrt(pos.x**2 + pos.y**2)
    error = R - current_radius

    
    integral += error * step_size
    derivative = (error - prev_error) / step_size
    steering = Kp * error + Ki * integral + Kd * derivative
    steering = max(min(steering, 1.0), -1.0)
    prev_error = error

    
    sentinel.SetPos(pos)
    direction = chrono.ChVector3d(pos.x, pos.y, 0)
    if direction.Length() > 1e-6:
        direction.Normalize()
    target_pos = direction * R
    target_pos.z = 0.5
    target.SetPos(target_pos)

    
    driver_inputs = veh.DriverInputs()
    driver_inputs.throttle = 0.3  
    driver_inputs.steering = steering
    driver_inputs.braking = 0.0

    
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)