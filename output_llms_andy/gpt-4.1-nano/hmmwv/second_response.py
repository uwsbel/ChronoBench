import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Set data paths
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)

# Visualization type for vehicle parts
vis_type = veh.VisualizationType_PRIMITIVES

# Collision type for chassis
chassis_collision_type = veh.CollisionType_NONE

# Tire model type
tire_model = veh.TireModelType_TMEASY

# Terrain dimensions (increased length to 200.0)
terrainHeight = 0
terrainLength = 200.0  # increased from 100.0
terrainWidth = 100.0

# Track point for chase camera
trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Render step size
render_step_size = 1.0 / 50  # 50 FPS

# Create the vehicle
vehicle = veh.HMMWV_Full()  # Could be veh.HMMWV_Reduced()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)

vehicle.Initialize()

# Set visualization types
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# Set collision system type
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create terrain
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

# Visualization: create Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Circular Path Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Path and Controller Implementation

# Define a circular path
radius = 50  # reasonable radius
center = chrono.ChVector3d(0, 0, 0)

# Create visualization objects for path points
sphere_radius = 0.5
sphere_color_sentinel = chrono.ChColor(1, 0, 0)  # Red for sentinel
sphere_color_target = chrono.ChColor(0, 1, 0)    # Green for target

# Create two spheres for visualization
sentinel_sphere = chrono.ChSphereShape(sphere_radius)
target_sphere = chrono.ChSphereShape(sphere_radius)

# Add spheres to visualization
sentinel_node = vis.GetSceneNode()
target_node = vis.GetSceneNode()

# Initialize sentinel and target points along the circle
theta = 0
delta_theta = 2 * math.pi / 100  # divide circle into segments

# Initialize sentinel and target angles
sentinel_angle = 0
target_angle = delta_theta

# Calculate initial points
sentinel_point = chrono.ChVector3d(center.x + radius * math.cos(sentinel_angle),
                                    center.y + radius * math.sin(sentinel_angle),
                                    0.5)
target_point = chrono.ChVector3d(center.x + radius * math.cos(target_angle),
                                   center.y + radius * math.sin(target_angle),
                                   0.5)

# Create spheres in visualization
sentinel_vis_node = vis.GetSceneNode().AddChild(chrono.ChBodyEasyShape(chrono.ChShapeSphere(sphere_radius), 1, False))
sentinel_vis_node.GetVisualShape().SetColor(sphere_color_sentinel)
sentinel_vis_node.SetPos(sentinel_point)

target_vis_node = vis.GetSceneNode().AddChild(chrono.ChBodyEasyShape(chrono.ChShapeSphere(sphere_radius), 1, False))
target_vis_node.GetVisualShape().SetColor(sphere_color_target)
target_vis_node.SetPos(target_point)

# PID controller parameters for steering
Kp = 2.0
Ki = 0.1
Kd = 0.5

# Initialize PID variables
integral_error = 0
previous_error = 0

# Constant throttle
constant_throttle = 0.3

# Re-define driver as a custom control
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
        # Calculate the vector from vehicle to target point
        veh_pos = self.vehicle.GetChassis().GetPos()
        veh_dir = self.vehicle.GetChassis().GetA().GetZ()  # vehicle's forward direction (assuming Z axis is up)
        # Alternatively, get vehicle heading
        veh_heading = self.vehicle.GetChassis().GetA().GetZ()

        # Vector from vehicle to target
        vec_to_target = self.target_point - veh_pos
        # Calculate angle between vehicle heading and vector to target
        # Project vectors onto the XY plane
        veh_forward = self.vehicle.GetChassis().GetA().GetZ()
        # Vehicle heading as a 2D vector
        heading_x = math.cos(self.vehicle.GetChassis().GetA().GetYaw())
        heading_y = math.sin(self.vehicle.GetChassis().GetA().GetYaw())

        vec_x = self.target_point.x - veh_pos.x
        vec_y = self.target_point.y - veh_pos.y

        angle_to_target = math.atan2(vec_y, vec_x) - math.atan2(heading_y, heading_x)
        # Normalize angle to [-pi, pi]
        while angle_to_target > math.pi:
            angle_to_target -= 2 * math.pi
        while angle_to_target < -math.pi:
            angle_to_target += 2 * math.pi

        # PID control for steering
        error = angle_to_target
        self.integral_error += error * render_step_size
        derivative = (error - self.previous_error) / render_step_size
        self.previous_error = error

        steering_cmd = Kp * error + Ki * self.integral_error + Kd * derivative
        # Clamp steering to [-1, 1]
        steering_cmd = max(-1.0, min(1.0, steering_cmd))
        return steering_cmd

    def update_visualization(self):
        # Update spheres positions
        if hasattr(self, 'sentinel_vis_node'):
            self.sentinel_vis_node.SetPos(self.sentinel_point)
        if hasattr(self, 'target_vis_node'):
            self.target_vis_node.SetPos(self.target_point)

    def step(self):
        # Update target points
        self.update_target_points()
        # Update visualization
        self.update_visualization()

# Instantiate path follower
path_follower = PathFollower(vehicle, center, radius)

# Main simulation loop
# Initialize real-time timer
realtime_timer = chrono.ChRealtimeStepTimer()

# Simulation step counter
step_number = 0
render_frame = 0

# Loop
while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Visualization: draw the path points
    # (Optional: add visualization of path circle)
    # For now, update spheres positions
    path_follower.step()

    # Render scene
    if (step_number % math.ceil(render_step_size / step_size) == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Get current vehicle heading (yaw)
    chassis = vehicle.GetChassis()
    # Get yaw angle from chassis orientation
    chassis_A = chassis.GetA()
    yaw = math.atan2(2.0 * (chassis_A.w * chassis_A.z + chassis_A.x * chassis_A.y),
                     1 - 2.0 * (chassis_A.y * chassis_A.y + chassis_A.z * chassis_A.z))
    # Calculate steering command
    steering = path_follower.get_steering()

    # Set driver inputs
    driver_inputs = veh.ChDriver.Inputs()
    driver_inputs.m_steering = steering
    driver_inputs.m_throttle = constant_throttle
    driver_inputs.m_braking = 0

    # Synchronize modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Increment step count
    step_number += 1

    # Spin to catch up with real time
    realtime_timer.Spin(step_size)