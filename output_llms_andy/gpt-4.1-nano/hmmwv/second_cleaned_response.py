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


vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    terrainLength,
    terrainWidth
)
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




radius = 50  
center = chrono.ChVector3d(0, 0, 0)


sphere_radius = 0.5
sphere_color_sentinel = chrono.ChColor(1, 0, 0)  
sphere_color_target = chrono.ChColor(0, 1, 0)    


sentinel_sphere = chrono.ChSphereShape(sphere_radius)
target_sphere = chrono.ChSphereShape(sphere_radius)


sentinel_node = vis.GetSceneNode()
target_node = vis.GetSceneNode()


theta = 0
delta_theta = 2 * math.pi / 100  


sentinel_angle = 0
target_angle = delta_theta


sentinel_point = chrono.ChVector3d(center.x + radius * math.cos(sentinel_angle),
                                    center.y + radius * math.sin(sentinel_angle),
                                    0.5)
target_point = chrono.ChVector3d(center.x + radius * math.cos(target_angle),
                                   center.y + radius * math.sin(target_angle),
                                   0.5)


sentinel_vis_node = vis.GetSceneNode().AddChild(chrono.ChBodyEasyShape(chrono.ChShapeSphere(sphere_radius), 1, False))
sentinel_vis_node.GetVisualShape().SetColor(sphere_color_sentinel)
sentinel_vis_node.SetPos(sentinel_point)

target_vis_node = vis.GetSceneNode().AddChild(chrono.ChBodyEasyShape(chrono.ChShapeSphere(sphere_radius), 1, False))
target_vis_node.GetVisualShape().SetColor(sphere_color_target)
target_vis_node.SetPos(target_point)


Kp = 2.0
Ki = 0.1
Kd = 0.5


integral_error = 0
previous_error = 0


constant_throttle = 0.3


class PathFollower:
    def __init__(self, vehicle, center, radius):
        self.vehicle = vehicle
        self.center = center
        self.radius = radius
        self.sentinel_angle = 0
        self.target_angle = 0
        self.update_target_points()

        self.integral_error = 0
        self.previous_error = 0

    def update_target_points(self):
        self.sentinel_angle += delta_theta
        if self.sentinel_angle > 2 * math.pi:
            self.sentinel_angle -= 2 * math.pi

        self.target_angle = self.sentinel_angle + delta_theta
        if self.target_angle > 2 * math.pi:
            self.target_angle -= 2 * math.pi

        self.sentinel_point = chrono.ChVector3d(
            self.center.x + self.radius * math.cos(self.sentinel_angle),
            self.center.y + self.radius * math.sin(self.sentinel_angle),
            0.5
        )
        self.target_point = chrono.ChVector3d(
            self.center.x + self.radius * math.cos(self.target_angle),
            self.center.y + self.radius * math.sin(self.target_angle),
            0.5
        )

    def get_steering(self):
        
        veh_pos = self.vehicle.GetChassis().GetPos()
        veh_dir = self.vehicle.GetChassis().GetA().GetZ()  
        
        veh_heading = self.vehicle.GetChassis().GetA().GetZ()

        
        vec_to_target = self.target_point - veh_pos
        
        
        veh_forward = self.vehicle.GetChassis().GetA().GetZ()
        
        heading_x = math.cos(self.vehicle.GetChassis().GetA().GetYaw())
        heading_y = math.sin(self.vehicle.GetChassis().GetA().GetYaw())

        vec_x = self.target_point.x - veh_pos.x
        vec_y = self.target_point.y - veh_pos.y

        angle_to_target = math.atan2(vec_y, vec_x) - math.atan2(heading_y, heading_x)
        
        while angle_to_target > math.pi:
            angle_to_target -= 2 * math.pi
        while angle_to_target < -math.pi:
            angle_to_target += 2 * math.pi

        
        error = angle_to_target
        self.integral_error += error * render_step_size
        derivative = (error - self.previous_error) / render_step_size
        self.previous_error = error

        steering_cmd = Kp * error + Ki * self.integral_error + Kd * derivative
        
        steering_cmd = max(-1.0, min(1.0, steering_cmd))
        return steering_cmd

    def update_visualization(self):
        
        if hasattr(self, 'sentinel_vis_node'):
            self.sentinel_vis_node.SetPos(self.sentinel_point)
        if hasattr(self, 'target_vis_node'):
            self.target_vis_node.SetPos(self.target_point)

    def step(self):
        
        self.update_target_points()
        
        self.update_visualization()


path_follower = PathFollower(vehicle, center, radius)



realtime_timer = chrono.ChRealtimeStepTimer()


step_number = 0
render_frame = 0


while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    
    
    
    path_follower.step()

    
    if (step_number % math.ceil(render_step_size / step_size) == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    chassis = vehicle.GetChassis()
    
    chassis_A = chassis.GetA()
    yaw = math.atan2(2.0 * (chassis_A.w * chassis_A.z + chassis_A.x * chassis_A.y),
                     1 - 2.0 * (chassis_A.y * chassis_A.y + chassis_A.z * chassis_A.z))
    
    steering = path_follower.get_steering()

    
    driver_inputs = veh.ChDriver.Inputs()
    driver_inputs.m_steering = steering
    driver_inputs.m_throttle = constant_throttle
    driver_inputs.m_braking = 0

    
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