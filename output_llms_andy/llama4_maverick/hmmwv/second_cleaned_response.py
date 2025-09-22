import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import numpy as np


veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_PRIMITIVES


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_TMEASY


terrainHeight = 0  
terrainLength = 200.0  
terrainWidth = 100.0  


contact_method = chrono.ChContactMethod_NSC


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
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())


path_radius = 30
path_center = chrono.ChVector3d(0, 0, 0)
num_path_points = 100
path_points = []
for i in range(num_path_points):
    angle = 2 * np.pi * i / num_path_points
    point = path_center + chrono.ChVector3d(path_radius * np.cos(angle), path_radius * np.sin(angle), 0)
    path_points.append(point)


ball1 = chrono.ChBodyEasySphere(1, 1000, True, True)
ball1.SetBodyFixed(True)
ball1.SetPos(path_points[0])
vis.GetSceneManager().getActiveScene().add(ball1)

ball2 = chrono.ChBodyEasySphere(1, 1000, True, True)
ball2.SetBodyFixed(True)
ball2.SetPos(path_points[num_path_points // 2])
vis.GetSceneManager().getActiveScene().add(ball2)


class PathFollowerDriver:
    def __init__(self, vehicle, path_points, throttle_value, kp, ki, kd):
        self.vehicle = vehicle
        self.path_points = path_points
        self.throttle_value = throttle_value
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.prev_error = 0
        self.target_point = path_points[0]
        self.sentinel_point = path_points[0]

        
        self.sentinel_sphere = chrono.ChBodyEasySphere(0.5, 1000, True, True)
        self.sentinel_sphere.SetBodyFixed(True)
        vis.GetSceneManager().getActiveScene().add(self.sentinel_sphere)

        self.target_sphere = chrono.ChBodyEasySphere(0.5, 1000, True, True)
        self.target_sphere.SetBodyFixed(True)
        vis.GetSceneManager().getActiveScene().add(self.target_sphere)

    def Update(self, time):
        
        current_pos = self.vehicle.GetPos()
        min_dist = float('inf')
        closest_point_index = 0
        for i, point in enumerate(self.path_points):
            dist = (current_pos - point).Length()
            if dist < min_dist:
                min_dist = dist
                closest_point_index = i
        self.target_point = self.path_points[(closest_point_index + 10) % len(self.path_points)]
        self.sentinel_point = self.path_points[closest_point_index]

        
        self.sentinel_sphere.SetPos(self.sentinel_point)
        self.target_sphere.SetPos(self.target_point)

        
        error = np.arctan2(self.target_point.y - current_pos.y, self.target_point.x - current_pos.x) - self.vehicle.GetRot().Q_to_Euler123().z
        self.integral += error * step_size
        derivative = (error - self.prev_error) / step_size
        steering_value = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error

        
        return veh.Inputs(self.throttle_value, steering_value, 0)

driver = PathFollowerDriver(vehicle.GetVehicle(), path_points, 0.3, 0.5, 0, 0)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0
render_steps = int(render_step_size / step_size)

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    driver_inputs = driver.Update(time)

    
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)