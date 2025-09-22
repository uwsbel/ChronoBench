import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_PRIMITIVES

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain
terrainHeight = 0      # terrain height
terrainLength = 200.0  # size in X direction (increased from 100.0)
terrainWidth = 100.0   # size in Y direction

# Point chassis tracked by the camera (corrected comment)
trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Circular path parameters
path_radius = 30.0
path_center = chrono.ChVector3d(50, 0, 0)

# PID controller parameters for steering
Kp = 0.5
Ki = 0.02
Kd = 0.1

# Create the HMMWV vehicle, set parameters, and initialize
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

# Create the terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create path visualization spheres
def create_path_spheres():
    spheres = []
    num_spheres = 20
    for i in range(num_spheres):
        angle = 2 * math.pi * i / num_spheres
        sphere_pos = chrono.ChVector3d(
            path_center.x + path_radius * math.cos(angle),
            path_center.y + path_radius * math.sin(angle),
            0.2
        )
        
        sphere_body = chrono.ChBodyEasySphere(0.5, 1000, True, False)
        sphere_body.SetPos(sphere_pos)
        sphere_body.SetFixed(True)
        sphere_body.GetVisualShape(0).SetColor(chrono.ChColor(1, 0, 0))  # Red spheres
        vehicle.GetSystem().Add(sphere_body)
        spheres.append(sphere_body)
    return spheres

# Create sentinel and target point visualization
sentinel_sphere = chrono.ChBodyEasySphere(0.8, 1000, True, False)
sentinel_sphere.SetFixed(True)
sentinel_sphere.GetVisualShape(0).SetColor(chrono.ChColor(0, 1, 0))  # Green sphere
vehicle.GetSystem().Add(sentinel_sphere)

target_sphere = chrono.ChBodyEasySphere(0.8, 1000, True, False)
target_sphere.SetFixed(True)
target_sphere.GetVisualShape(0).SetColor(chrono.ChColor(0, 0, 1))  # Blue sphere
vehicle.GetSystem().Add(target_sphere)

# Create path visualization
path_spheres = create_path_spheres()

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Path Following Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Path following functions
def get_target_point(vehicle_pos, time):
    """Calculate target point on circular path"""
    # Use time-based progression along the path
    speed_factor = 0.1  # Controls how fast we move along the path
    angle = speed_factor * time
    
    target_x = path_center.x + path_radius * math.cos(angle)
    target_y = path_center.y + path_radius * math.sin(angle)
    
    return chrono.ChVector3d(target_x, target_y, 0)

def get_sentinel_point(vehicle_pos, target_pos):
    """Calculate sentinel point (lookhead point)"""
    lookhead_distance = 10.0
    direction = target_pos - vehicle_pos
    direction.Normalize()
    return vehicle_pos + direction * lookhead_distance

# PID controller class
class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0
    
    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

# Initialize PID controller
steering_pid = PIDController(Kp, Ki, Kd)

# output vehicle mass
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Get vehicle position and orientation
    vehicle_pos = vehicle.GetVehicle().GetPos()
    vehicle_rot = vehicle.GetVehicle().GetRot()
    vehicle_dir = vehicle_rot.GetZaxis()  # Vehicle forward direction

    # Calculate target and sentinel points
    target_pos = get_target_point(vehicle_pos, time)
    sentinel_pos = get_sentinel_point(vehicle_pos, target_pos)

    # Update visualization spheres
    target_sphere.SetPos(target_pos + chrono.ChVector3d(0, 0, 1))
    sentinel_sphere.SetPos(sentinel_pos + chrono.ChVector3d(0, 0, 1))

    # Calculate steering error (cross track error)
    to_target = target_pos - vehicle_pos
    to_target.z = 0  # Only consider horizontal plane
    to_target.Normalize()
    
    # Calculate angle between vehicle direction and target direction
    vehicle_dir_2d = chrono.ChVector3d(vehicle_dir.x, vehicle_dir.y, 0)
    vehicle_dir_2d.Normalize()
    
    # Cross product to determine steering direction
    cross_product = vehicle_dir_2d.Cross(to_target)
    steering_error = math.asin(max(-1, min(1, cross_product.z)))

    # PID control for steering
    steering_input = steering_pid.update(steering_error, step_size)
    steering_input = max(-1, min(1, steering_input))  # Clamp to [-1, 1]

    # Create driver inputs
    driver_inputs = veh.DriverInputs()
    driver_inputs.m_throttle = 0.3  # Constant throttle
    driver_inputs.m_steering = steering_input
    driver_inputs.m_braking = 0.0

    # Render scene
    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Update modules
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation for one timestep for all modules
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)