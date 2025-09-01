import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math


import pychrono.postprocess as post

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


path_radius = 50.0  
path_center = chrono.ChVector3d(0, 0, 0)  
num_points = 100  

def generate_circular_path(radius, center, num_points):
    path = []
    for i in range(num_points):
        angle = 2 * math.pi * i / num_points
        x = center.x + radius * math.cos(angle)
        y = center.y + radius * math.sin(angle)
        z = center.z
        path.append(chrono.ChVector3d(x, y, z))
    return path

path = generate_circular_path(path_radius, path_center, num_points)


pid_gains = {
    'Kp': 0.8,
    'Ki': 0.05,
    'Kd': 0.1
}


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
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Circular Path Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())


marker_size = 0.5
current_target = irr.ChMarker()
current_target.SetPos(path[0])
current_target.SetColor(chrono.ChColor(1, 0, 0))
current_target.SetScale(marker_size)
vis.GetSystem().Add(current_target)

next_target = irr.ChMarker()
next_target.SetPos(path[1])
next_target.SetColor(chrono.ChColor(0, 1, 0))
next_target.SetScale(marker_size)
vis.GetSystem().Add(next_target)


class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0
        self.previous_error = 0
        
    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.previous_error = error
        return output

steering_controller = PIDController(pid_gains['Kp'], pid_gains['Ki'], pid_gains['Kd'])


throttle = 0.3


step_number = 0
render_frame = 0
current_target_index = 0


realtime_timer = chrono.ChRealtimeStepTimer()

while vis.Run():
    time = vehicle.GetSystem().GetChTime()
    
    
    vehicle_pos = vehicle.GetVehicle().GetChassis().GetPos()
    
    
    target_pos = path[current_target_index]
    
    
    error = (target_pos.y - vehicle_pos.y)
    
    
    steering = steering_controller.update(error, step_size)
    
    
    max_steering = 0.6  
    if steering > max_steering:
        steering = max_steering
    elif steering < -max_steering:
        steering = -max_steering
    
    
    current_target.SetPos(path[current_target_index])
    next_target.SetPos(path[(current_target_index + 1) % len(path)])
    
    
    distance = (target_pos - vehicle_pos).Length()
    if distance < 1.0:  
        current_target_index = (current_target_index + 1) % len(path)
    
    
    driver_inputs = veh.DriverInputs()
    driver_inputs.m_throttle = throttle
    driver_inputs.m_steering = steering
    
    
    if step_number % math.ceil(render_step_size / step_size) == 0:
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
    
    
    step_number += 1
    
    
    realtime_timer.Spin(step_size)