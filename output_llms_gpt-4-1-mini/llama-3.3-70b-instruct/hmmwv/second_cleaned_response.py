import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math
import numpy as np


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
path_center = chrono.ChVector3d(0, 0, 0)
num_points = 100
path_points = []
for i in range(num_points):
    angle = 2 * math.pi * i / num_points
    point = path_center + chrono.ChVector3d(path_radius * math.cos(angle), path_radius * math.sin(angle), 0)
    path_points.append(point)


ball1 = chrono.ChBodyEasySphere(0.5, 1000, True, True)
ball1.SetPos(path_points[0])
ball1.SetMaterial(chrono.ChMaterialSurfaceNSC())
ball1.GetMaterialSurfaceNSC().SetFriction(0.9)
ball1.GetMaterialSurfaceNSC().SetRestitution(0.01)
ball1.AddCollide()
vehicle.GetSystem().Add(ball1)

ball2 = chrono.ChBodyEasySphere(0.5, 1000, True, True)
ball2.SetPos(path_points[-1])
ball2.SetMaterial(chrono.ChMaterialSurfaceNSC())
ball2.GetMaterialSurfaceNSC().SetFriction(0.9)
ball2.GetMaterialSurfaceNSC().SetRestitution(0.01)
ball2.AddCollide()
vehicle.GetSystem().Add(ball2)


class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.error_prev = 0
        self.error_int = 0

    def update(self, error, dt):
        error_dot = (error - self.error_prev) / dt
        self.error_int += error * dt
        self.error_prev = error
        return self.kp * error + self.ki * self.error_int + self.kd * error_dot

pid_controller = PIDController(1.0, 0.1, 0.1)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0
target_point_index = 0
while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    
    if (step_number % int(render_step_size / step_size) == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    vehicle_pos = vehicle.GetChassisBody().GetPos()
    vehicle_rot = vehicle.GetChassisBody().GetRot()

    
    target_point = path_points[target_point_index]
    if np.linalg.norm(np.array([vehicle_pos.x(), vehicle_pos.y()]) - np.array([target_point.x(), target_point.y()])) < 1.0:
        target_point_index = (target_point_index + 1) % num_points

    
    error = np.arctan2(target_point.y() - vehicle_pos.y(), target_point.x() - vehicle_pos.x()) - vehicle_rot.EulerAngles().z()

    
    steering_input = pid_controller.update(error, step_size)

    
    throttle_input = 0.3
    braking_input = 0.0
    vehicle.GetPart(0).SetSteering(steering_input)
    vehicle.GetPart(0).SetThrottle(throttle_input)
    vehicle.GetPart(0).SetBraking(braking_input)

    
    vehicle.Advance(step_size)
    terrain.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)