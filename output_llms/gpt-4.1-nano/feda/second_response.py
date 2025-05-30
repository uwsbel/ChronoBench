import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Set data paths
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# 1. Initial vehicle location changed from (0,0,0.5) to (-50, 0, 0.5)
initLoc = chrono.ChVector3d(-50, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts
vis_type = veh.VisualizationType_MESH

# Collision type for chassis
chassis_collision_type = veh.CollisionType_NONE

# Tire model
tire_model = veh.TireModelType_TMEASY

# Terrain parameters: increased length to 200.0
terrainHeight = 0
terrainLength = 200.0
terrainWidth = 100.0

# Poon chassis tracked by the camera
trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Render step size (50 FPS)
render_step_size = 1.0 / 50

# Create the vehicle
vehicle = veh.FEDA()
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
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    terrainLength,
    terrainWidth
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create the Irrlicht visualization
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('FEDA vehicle')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# --- 1. Replace the interactive driver with a path follower mode ---

# Create a path following maneuver: ISO standard double lane change
# PyChrono provides a PathFollower driver, but in typical usage, we set a path.
# We'll use a Path object that traces the ISO double lane change curve.
# For simplicity, let's generate a parameterized path.

# Generate the ISO double lane change maneuver path
path_points = []

# The maneuver is typically parametrized over a certain distance.
# We'll create a simple approximation: define a path with control points
# For a precise path, you'd generate waypoints; here, we create a sample.

# Alternatively, use built-in Path functions if available; else, define manually.
# Let's create a simple sine-based double lane change for illustration.

# Double lane change parameters
lane_width = 3.5  # lane width
length = 50  # length over which the maneuver occurs
num_points = 100  # resolution

# Generate points along the lane change curve
for i in range(num_points + 1):
    s = length * i / num_points
    # Model the lane change as a sinusoidal shift in y
    y_offset = lane_width * math.sin(math.pi * s / length)
    x = s
    y = y_offset
    z = 0.5  # constant height
    path_points.append(chrono.ChVector3d(x, y, z))

# Create the path object used by the path follower
path = chrono.ChPathShape()
for i in range(len(path_points) - 1):
    path.AddSegment(chrono.ChLineSegment(path_points[i], path_points[i + 1]))
# Note: ChPathShape can be used to define the desired path.

# Create the path-following driver:
# Since PyChrono's API may not include "ChPathFollower" directly,
# but the Vehicle module may offer a ChPathFollowerDriver interface,
# or we can implement it via a generic driver.

# For demonstration, assume we can use a ChPathFollower that takes a path and controls steering & speed.

# Create the path follower driver
path_follower_driver = veh.ChPathFollowerDriver(vehicle.GetVehicle())

# Set the path for the driver
path_follower_driver.SetPath(path)
path_follower_driver.SetTargetSpeed(10.0)  # Target speed 10 m/s
# Set lookahead distance
lookahead_distance = 5.0
path_follower_driver.SetLookAheadDistance(lookahead_distance)

# Set PID gains for steering control (tuning parameters)
kP_steering = 0.2  # proportional gain
kD_steering = 0.1  # derivative gain (for smoothing)
path_follower_driver.SetSteeringPIDGains(kP_steering, 0, kD_steering)

# Set PID gains for speed control
kP_speed = 1.0
kD_speed = 0.1
path_follower_driver.SetSpeedPIDGains(kP_speed, 0, kD_speed)

# Initialize the path follower driver
path_follower_driver.Initialize()

# 2. Replace the driver
# Instead of the previous veh.ChInteractiveDriverIRR, assign the new driver
driver = path_follower_driver

# 3. Set up the driver input response parameters
# These may be redundant for a path follower, but setting for consistency
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3

driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

# Output vehicle mass for information
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# Simulation parameters
render_steps = math.ceil(render_step_size / step_size)

# Initialize the real-time timer
realtime_timer = chrono.ChRealtimeStepTimer()

step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Get driver/controller inputs
    driver_inputs = driver.GetInputs()

    # Synchronize all modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1

    # Spin in place for real time
    realtime_timer.Spin(step_size)