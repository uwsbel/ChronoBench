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


Kp = 0.5
Ki = 0.05
Kd = 0.1
integral = 0.0
prev_error = 0.0


radius = 20.0  
center = chrono.ChVector3d(50.0, 0.0, 0.0)  


sphere1 = irr.ChSphere()
sphere1.SetPos(center + chrono.ChVector3d(radius, 0, 0))
sphere1.SetRadius(0.5)
sphere1.SetColor(chrono.ChColor(1, 0, 0))
vis.GetSceneManager().AddLightSphere(sphere1)

sphere2 = irr.ChSphere()
sphere2.SetPos(center + chrono.ChVector3d(radius * math.cos(math.pi/2), radius * math.sin(math.pi/2), 0))
sphere2.SetRadius(0.5)
sphere2.SetColor(chrono.ChColor(0, 1, 0))
vis.GetSceneManager().AddLightSphere(sphere2)

def get_desired_position(time):
    
    angle = time * 0.1  
    x = center.x + radius * math.cos(angle)
    y = center.y + radius * math.sin(angle)
    return chrono.ChVector3d(x, y, 0)


throttle = 0.3


render_steps = math.ceil(render_step_size / step_size)
step_number = 0
render_frame = 0
realtime_timer = chrono.ChRealtimeStepTimer()

while vis.Run():
    time = vehicle.GetSystem().GetChTime()
    
    
    current_pos = vehicle.GetVehicle().GetChassis().GetPos()
    
    
    desired_pos = get_desired_position(time)
    
    
    error = math.sqrt((current_pos.x - desired_pos.x)**2 + (current_pos.y - desired_pos.y)**2)
    integral += error * step_size
    derivative = (error - prev_error) / step_size
    steering = Kp * error + Ki * integral + Kd * derivative
    steering = max(min(steering, 1.0), -1.0)  
    
    
    driver_inputs = veh.ChDriverInputs()
    driver_inputs.m_throttle = throttle
    driver_inputs.m_steering = steering
    
    
    angle = time * 0.1
    sphere1.SetPos(center + chrono.ChVector3d(radius * math.cos(angle), radius * math.sin(angle), 0))
    sphere2.SetPos(center + chrono.ChVector3d(radius * math.cos(angle + math.pi/2), radius * math.sin(angle + math.pi/2), 0))
    
    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1
    
    
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)
    
    
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)
    
    
    prev_error = error
    step_number += 1
    realtime_timer.Spin(step_size)