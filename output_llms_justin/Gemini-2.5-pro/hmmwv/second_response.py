import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# --- PyChrono Data Paths ---
# Ensure CHRONO_DATA_DIR is set or Chrono data is in a default location.
# You can uncomment and set this manually if needed:
# chrono.SetChronoDataPath("your_path_to_chrono_data_directory")

CHRONO_DATA_DIR = chrono.GetChronoDataPath()
if not CHRONO_DATA_DIR or CHRONO_DATA_DIR == "../data/": # Default placeholder might be invalid
    # Attempt to find data directory relative to this script or common install spots
    # This is a fallback; setting CHRONO_DATA_DIR environment variable is preferred
    import os
    script_dir = os.path.dirname(os.path.abspath(__file__))
    potential_path = os.path.join(script_dir, "..", "data", "") # Common for source builds
    if os.path.exists(os.path.join(potential_path, "vehicle", "hmmwv", "HMMWV_Vehicle.json")):
         CHRONO_DATA_DIR = potential_path
    else: # Try typical install path if available (platform dependent)
        # This part is highly dependent on where Chrono installs its data
        # For simplicity, we'll rely on GetChronoDataPath() or user setting CHRONO_DATA_DIR
        print("Warning: Chrono data directory might not be correctly set.")
        print(f"CHRONO_DATA_DIR detected as: {CHRONO_DATA_DIR}")
        print("Please ensure CHRONO_DATA_DIR environment variable is set or pychrono.SetChronoDataPath() is called with the correct path.")


chrono.SetChronoDataPath(CHRONO_DATA_DIR)
veh.SetDataPath(CHRONO_DATA_DIR + 'vehicle/')

# --- Simulation Parameters ---

# Circular path parameters
path_radius = 40.0
path_center = chrono.ChVector3d(0, 0, 0.1)  # Path z-coordinate slightly above ground for clarity

# Initial vehicle location and orientation
# Vehicle starts at (path_radius, 0, 0.5) on the circle, facing +Y (tangent to circle)
initLoc = chrono.ChVector3d(path_center.x + path_radius, path_center.y, 0.5)
initRot = chrono.ChQuaterniond()
initRot.SetFromAngleZ(math.pi / 2.0)  # Rotate 90 degrees around Z to face +Y

# Visualization type for vehicle parts
vis_type = veh.VisualizationType_PRIMITIVES

# Collision type for chassis
chassis_collision_type = veh.CollisionType_NONE  # As per original, can be PRIMITIVES

# Tire model type
tire_model = veh.TireModelType_TMEASY

# Rigid Terrain parameters
terrain_height = 0.0  # Surface Z height of the terrain
terrainLength = 200.0  # MODIFICATION: Increased terrain length (X direction)
terrainWidth = 100.0   # Size in Y direction

# Camera tracking point (relative to chassis)
camera_trackPoint = chrono.ChVector3d(-4.0, 0.0, 1.7) # Adjusted for better view

# Contact method
contact_method = chrono.ChContactMethod_NSC

# Simulation step sizes
step_size = 2e-3  # Adjusted for potentially faster simulation, can be 1e-3
tire_step_size = step_size

# Rendering step size
render_step_size = 1.0 / 50  # Target 50 FPS

# --- Create Chrono System and HMMWV Vehicle ---
system = chrono.ChSystemNSC() # Create a system
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize(system) # Pass the system to the vehicle

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# --- Create Terrain ---
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(system) # Pass system to terrain object
# Create a flat patch of terrain centered at (0,0,terrain_height)
patch = terrain.AddPatch(patch_mat,
                         chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrain_height), chrono.QUNIT),
                         terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# --- Path and Controller Implementation ---

# 1. Create the circular path (ChBezierCurve)
path_points_list = []
num_circle_points = 100  # Number of points to define the circle geometry
for i in range(num_circle_points + 1):  # +1 to ensure the loop closes
    angle = (2 * math.pi * i) / num_circle_points
    x = path_center.x + path_radius * math.cos(angle)
    y = path_center.y + path_radius * math.sin(angle)
    z = path_center.z
    path_points_list.append(chrono.ChVector3d(x, y, z))

path_curve = chrono.ChBezierCurve(path_points_list)

# 2. Visualize the path using two balls (static markers)
path_marker_radius = 0.6
path_marker_color = chrono.ChColor(0.8, 0.2, 0.2) # Reddish

# Marker 1 at the start of the path
marker1_pos = path_points_list[0]
path_display_marker1 = chrono.ChBodyEasySphere(path_marker_radius, 1000, True, False) # density, visualize, no collide
path_display_marker1.SetPos(marker1_pos)
path_display_marker1.SetBodyFixed(True)
path_display_marker1.GetVisualShape(0).SetColor(path_marker_color)
system.Add(path_display_marker1)

# Marker 2 at a quarter point of the path
marker2_pos = path_points_list[int(num_circle_points / 4)]
path_display_marker2 = chrono.ChBodyEasySphere(path_marker_radius, 1000, True, False)
path_display_marker2.SetPos(marker2_pos)
path_display_marker2.SetBodyFixed(True)
path_display_marker2.GetVisualShape(0).SetColor(path_marker_color)
system.Add(path_display_marker2)

# 3. Setup Driver and Controllers
# ChPathFollowerDriver uses a ChPathSteeringController (Stanley-type) by default.
# We can tune its parameters.
target_speed_for_driver = 15.0  # m/s. Driver uses this for its internal speed controller.
driver = veh.ChPathFollowerDriver(vehicle.GetVehicle(), path_curve, "CircularPath", target_speed_for_driver, True) # True for closed path

# Access the default steering controller (Stanley) and set its parameters
steering_controller = driver.GetSteeringController()
steering_controller.SetLookAheadDistance(6.0)  # Look-ahead distance in meters
steering_controller.SetGains(Kp=0.7, Ki=0.0, Kd=0.0)  # Proportional gain for Stanley controller (Ki, Kd often unused or for different aspects)

# Constant throttle value
CONST_THROTTLE_VALUE = 0.3

driver.Initialize()

# --- Visualization Setup (Irrlicht) ---
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Path Following Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(camera_trackPoint, 6.0, 0.5)  # Point to track, distance, height
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle()) # Attaches vehicle and its system to Irrlicht

# --- Visualization for Controller Sentinel and Target Points ---
sentinel_sphere_radius = 0.35
sentinel_vis_sphere = chrono.ChBodyEasySphere(sentinel_sphere_radius, 1000, True, False)
sentinel_vis_sphere.GetVisualShape(0).SetColor(chrono.ChColor(1, 0, 0))  # Red
sentinel_vis_sphere.SetBodyFixed(True) # Position updated kinematically
system.Add(sentinel_vis_sphere)

target_sphere_radius = 0.35
target_vis_sphere = chrono.ChBodyEasySphere(target_sphere_radius, 1000, True, False)
target_vis_sphere.GetVisualShape(0).SetColor(chrono.ChColor(0, 0, 1))  # Blue
target_vis_sphere.SetBodyFixed(True) # Position updated kinematically
system.Add(target_vis_sphere)


# --- Simulation Loop ---
print(f"VEHICLE MASS: {vehicle.GetVehicle().GetMass()}")

realtime_timer = chrono.ChRealtimeStepTimer()
simulation_steps = 0
render_frames_interval = math.ceil(render_step_size / step_size)

while vis.Run():
    current_time = system.GetChTime()

    # Render scene
    if simulation_steps % render_frames_interval == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    # Get driver inputs from ChPathFollowerDriver
    driver.Synchronize(current_time)
    driver_inputs = driver.GetInputs()

    # Override throttle with constant value
    driver_inputs.m_throttle = CONST_THROTTLE_VALUE
    driver_inputs.m_braking = 0.0  # Ensure no braking from driver's speed controller

    # Update modules
    terrain.Synchronize(current_time)
    vehicle.Synchronize(current_time, driver_inputs, terrain)
    vis.Synchronize(current_time, driver_inputs)

    # Update sentinel and target sphere positions
    # (Accessing the controller used by the driver)
    active_steering_controller = driver.GetSteeringController()
    sentinel_pos_abs = active_steering_controller.GetSentinelPos()
    target_pos_abs = active_steering_controller.GetTargetPos()
    
    sentinel_vis_sphere.SetPos(sentinel_pos_abs)
    sentinel_vis_sphere.SetRot(chrono.QUNIT) # Keep sphere upright
    target_vis_sphere.SetPos(target_pos_abs)
    target_vis_sphere.SetRot(chrono.QUNIT)   # Keep sphere upright

    # Advance simulation
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size) # This advances the ChSystem as well
    vis.Advance(step_size)     # Advance Irrlicht visualization

    simulation_steps += 1
    realtime_timer.Spin(step_size)

del vis # Properly dispose of Irrlicht resources