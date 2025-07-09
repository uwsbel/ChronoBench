import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math
import numpy as np


chrono.SetChronoDataPath('/path/to/chrono/data')


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


path_radius = 10.0
path_center = chrono.ChVector3d(0, 0, 0)
num_points = 100
path_points = []
for i in range(num_points):
    angle = 2 * math.pi * i / num_points
    point = path_center + chrono.ChVector3d(path_radius * math.cos(angle), path_radius * math.sin(angle), 0)
    path_points.append(point)


ball1 = chrono.ChBodyEasySphere(0.5, 1000, True, True)
ball1.SetPos(path_points[0])
ball1.SetColor(chrono.ChColor(1, 0, 0))
vehicle.GetSystem().Add(ball1)

ball2 = chrono.ChBodyEasySphere(0.5, 1000, True, True)
ball2.SetPos(path_points[-1])
ball2.SetColor(chrono.ChColor(0, 1, 0))
vehicle.GetSystem().Add(ball2)


class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.error = 0
        self.prev_error = 0
        self.integral = 0

    def update(self, error):
        self.prev_error = self.error
        self.error = error
        self.integral += error * step_size
        return self.kp * error + self.ki * self.integral + self.kd * (error - self.prev_error) / step_size


pid_controller = PIDController(1.0, 0.1, 0.01)


target_point = path_points[0]


sentinel_point = path_points[0]


sentinel_ball = chrono.ChBodyEasySphere(0.2, 1000, True, True)
sentinel_ball.SetPos(sentinel_point)
sentinel_ball.SetColor(chrono.ChColor(0, 0, 1))
vehicle.GetSystem().Add(sentinel_ball)

target_ball = chrono.ChBodyEasySphere(0.2, 1000, True, True)
target_ball.SetPos(target_point)
target_ball.SetColor(chrono.ChColor(1, 1, 0))
vehicle.GetSystem().Add(target_ball)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0
while vis.Run() :
    time = vehicle.GetSystem().GetChTime()

    
    if (step_number % int(render_step_size / step_size) == 0) :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    current_point = vehicle.GetChassis().GetPos()
    distance_to_target = (current_point - target_point).Length()
    if distance_to_target < 1.0:
        current_index = path_points.index(target_point)
        if current_index < len(path_points) - 1:
            target_point = path_points[current_index + 1]
        else:
            target_point = path_points[0]
    target_ball.SetPos(target_point)

    
    sentinel_point = target_point + (target_point - current_point).GetNormalized() * 5.0
    sentinel_ball.SetPos(sentinel_point)

    
    error = (target_point - current_point).GetNormalized().dot(chrono.ChVector3d(1, 0, 0))
    steering_angle = pid_controller.update(error)

    
    vehicle.SetThrottle(0.3)
    vehicle.SetSteering(steering_angle)
    vehicle.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)