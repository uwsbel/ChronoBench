import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Set data path
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)

# Visualization type for vehicle parts
vis_type = veh.VisualizationType_PRIMITIVES

# Collision type for chassis
chassis_collision_type = veh.CollisionType_NONE

# Tire model
tire_model = veh.TireModelType_TMEASY

# Terrain dimensions (increased from 100 to 200)
terrainHeight = 0
terrainLength = 200.0
terrainWidth = 100.0

# Track point for camera
trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Step sizes
step_size = 1e-3
tire_step_size = step_size

# Render step
render_step_size = 1.0 / 50  # 50 FPS

# Create vehicle
vehicle = veh.HMMWV_Full()
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

# Set collision system
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystemType_BULLET)

# Create terrain
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

# Visualization with Irrlicht
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Circular Path Following Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Path and controller parameters
radius = 50  # radius of the circle
center_point = chrono.ChVector3d(0, 0, 0.5)
# Generate points for circle (optional: can be used for visualization)
num_points = 36
circle_points = [
    chrono.ChVector3d(
        center_point.x + radius * math.cos(2 * math.pi * i / num_points),
        center_point.y + radius * math.sin(2 * math.pi * i / num_points),
        center_point.z
    )
    for i in range(num_points)
]

# Create visualization objects for path: two balls (spheres) for sentinel and target points
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

# Initialize target index
target_index = 0

# Control parameters
desired_throttle = 0.3  # constant throttle
# PID gains (tuned as needed)
Kp = 0.8
Ki = 0.0
Kd = 0.2

# Initialize PID controller variables
integral_error = 0.0
prev_error = 0.0

# Main simulation loop
# Replace the interactive driver with a custom path follower
# Disable the original driver
# driver = veh.ChInteractiveDriverIRR(vis)
# driver.Initialize()

# Create a class to handle path following
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
        # Advance target index if close enough
        current_target = self.points[self.target_idx]
        vehicle_pos = vehicle.GetVehicle().GetPos()
        dist = (vehicle_pos - current_target).Length()
        if dist < 5.0:
            self.target_idx = (self.target_idx + 1) % len(self.points)

        self.current_target = self.points[self.target_idx]
        # Visualize sentinel and target
        sentinel_sphere.SetPos(vehicle.GetVehicle().GetPos())
        target_sphere.SetPos(self.current_target)

    def compute_steering(self):
        vehicle_pos = vehicle.GetVehicle().GetPos()
        vehicle_dir = vehicle.GetVehicle().GetA().GetZaxis()  # facing direction
        # Vector from vehicle to current target
        to_target = self.current_target - vehicle_pos
        # Normalize
        to_target_norm = to_target
        to_target_norm.Normalize()

        # Calculate angle difference between vehicle heading and to_target vector
        heading = vehicle.GetVehicle().GetA().GetZaxis()  # Assuming Z axis points forward
        angle_error = math.atan2(
            to_target.y * heading.x - to_target.x * heading.y,
            to_target.x * heading.x + to_target.y * heading.y
        )

        # PID controller for steering
        error = angle_error
        self.integral_error += error * step_size
        derivative = (error - self.prev_error) / step_size
        steer_cmd = self.kp * error + self.ki * self.integral_error + self.kd * derivative
        self.prev_error = error

        # Clamp steering to [-1,1]
        max_steer = 1.0
        steer_cmd = max(min(steer_cmd, max_steer), -max_steer)

        return steer_cmd

# Instantiate path follower
path_follower = PathFollower(center_point, radius, circle_points)

# Simulation variables
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Update path target
    path_follower.update_target()

    # Compute steering from path follower
    steering_input = path_follower.compute_steering()

    # Send controls to vehicle
    # Instead of driver inputs, set throttle and steering directly
    driver_inputs = {'steering': steering_input, 'throttle': desired_throttle, 'braking': 0.0}

    # Visualization
    if (step_number % math.ceil(render_step_size / step_size) == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

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

    # Increment step
    step_number += 1

    # Maintain real-time
    realtime_timer.Spin(step_size)