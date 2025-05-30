import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)


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
vehicle.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)

vehicle.Initialize()


vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)


vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystemType_BULLET)


patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QDU(1, 0, 0, 0)),
    terrainLength,
    terrainWidth
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Circular Path Following Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())


radius = 50  
center_point = chrono.ChVector3d(0, 0, 0.5)

num_points = 36
circle_points = [
    chrono.ChVector3d(
        center_point.x + radius * math.cos(2 * math.pi * i / num_points),
        center_point.y + radius * math.sin(2 * math.pi * i / num_points),
        center_point.z
    )
    for i in range(num_points)
]


sentinel_sphere = chrono.ChBodySimple()
sentinel_sphere.GetCollisionModel().ClearModel()
sentinel_sphere.SetMass(0)
sentinel_sphere.SetPos(chrono.ChVector3d(0, 0, 0))
sentinel_sphere.GetVisualShape().AddSphere(0.5, True, chrono.ChColor(1, 0, 0))
vehicle.GetSystem().Add(sentinel_sphere)

target_sphere = chrono.ChBodySimple()
target_sphere.GetCollisionModel().ClearModel()
target_sphere.SetMass(0)
target_sphere.SetPos(chrono.ChVector3d(0, 0, 0))
target_sphere.GetVisualShape().AddSphere(0.5, True, chrono.ChColor(0, 1, 0))
vehicle.GetSystem().Add(target_sphere)


target_index = 0


desired_throttle = 0.3  

Kp = 0.8
Ki = 0.0
Kd = 0.2


integral_error = 0.0
prev_error = 0.0








class PathFollower:
    def __init__(self, center, radius, points):
        self.center = center
        self.radius = radius
        self.points = points
        self.target_idx = 0
        self.kp = Kp
        self.ki = Ki
        self.kd = Kd
        self.integral_error = 0
        self.prev_error = 0

    def update_target(self):
        
        current_target = self.points[self.target_idx]
        vehicle_pos = vehicle.GetVehicle().GetPos()
        dist = (vehicle_pos - current_target).Length()
        if dist < 5.0:
            self.target_idx = (self.target_idx + 1) % len(self.points)

        self.current_target = self.points[self.target_idx]
        
        sentinel_sphere.SetPos(vehicle.GetVehicle().GetPos())
        target_sphere.SetPos(self.current_target)

    def compute_steering(self):
        vehicle_pos = vehicle.GetVehicle().GetPos()
        vehicle_dir = vehicle.GetVehicle().GetA().GetZaxis()  
        
        to_target = self.current_target - vehicle_pos
        
        to_target_norm = to_target
        to_target_norm.Normalize()

        
        heading = vehicle.GetVehicle().GetA().GetZaxis()  
        angle_error = math.atan2(
            to_target.y * heading.x - to_target.x * heading.y,
            to_target.x * heading.x + to_target.y * heading.y
        )

        
        error = angle_error
        self.integral_error += error * step_size
        derivative = (error - self.prev_error) / step_size
        steer_cmd = self.kp * error + self.ki * self.integral_error + self.kd * derivative
        self.prev_error = error

        
        max_steer = 1.0
        steer_cmd = max(min(steer_cmd, max_steer), -max_steer)

        return steer_cmd


path_follower = PathFollower(center_point, radius, circle_points)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    
    path_follower.update_target()

    
    steering_input = path_follower.compute_steering()

    
    
    driver_inputs = {'steering': steering_input, 'throttle': desired_throttle, 'braking': 0.0}

    
    if (step_number % math.ceil(render_step_size / step_size) == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)