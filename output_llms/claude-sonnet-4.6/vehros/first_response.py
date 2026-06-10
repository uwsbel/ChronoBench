# =============================================================================
# PyChrono HMMWV Vehicle Simulation with ROS Integration
# =============================================================================

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.ros as chros

# Try to import ROS-related packages
try:
    import rclpy
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    print("Warning: ROS2 (rclpy) not available. Running without ROS integration.")

# =============================================================================
# SIMULATION PARAMETERS
# =============================================================================

# Simulation timestep
step_size = 1e-3

# Simulation end time
t_end = 30.0

# Terrain parameters
terrain_length = 100.0  # meters
terrain_width = 100.0   # meters
terrain_friction = 0.9
terrain_restitution = 0.01

# Initial vehicle position and orientation
init_loc = chrono.ChVector3d(0, 0, 0.5)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)

# =============================================================================
# INITIALIZE CHRONO VEHICLE ENVIRONMENT
# =============================================================================

# Set the path to the Chrono data directory
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# =============================================================================
# CREATE THE HMMWV VEHICLE
# =============================================================================

print("Creating HMMWV vehicle...")

# Create the HMMWV vehicle object
hmmwv = veh.HMMWV_Full()

# Set contact method
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)

# Set chassis fixed (False = vehicle can move)
hmmwv.SetChassisFixed(False)

# Enable/disable chassis collision
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)

# Set initial position and orientation
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))

# Set powertrain/engine type
hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)

# Set transmission type
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)

# Set drive type (4WD)
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)

# Set steering type
hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)

# Set tire model
hmmwv.SetTireType(veh.TireModelType_TMEASY)

# Set tire step size
hmmwv.SetTireStepSize(step_size)

# Initialize the vehicle
hmmwv.Initialize()

# Set visualization types for vehicle components
hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

print("HMMWV vehicle created and initialized.")

# =============================================================================
# CREATE THE TERRAIN
# =============================================================================

print("Creating terrain...")

# Get the vehicle system
vehicle_system = hmmwv.GetSystem()

# Create rigid terrain
terrain = veh.RigidTerrain(vehicle_system)

# Create a terrain patch
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(terrain_friction)
patch_mat.SetRestitution(terrain_restitution)

# Add flat terrain patch
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    terrain_length,
    terrain_width
)

# Set terrain visualization
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

# Initialize terrain
terrain.Initialize()

print("Terrain created and initialized.")

# =============================================================================
# CREATE THE DRIVER SYSTEM
# =============================================================================

print("Creating driver system...")

# Create an interactive driver system
driver = veh.ChDriver(hmmwv.GetVehicle())

# Create driver inputs structure
driver_inputs = veh.DriverInputs()
driver_inputs.m_steering = 0.0
driver_inputs.m_throttle = 0.0
driver_inputs.m_braking = 0.0

# Initialize driver
driver.Initialize()

print("Driver system created and initialized.")

# =============================================================================
# SET UP ROS INTEGRATION
# =============================================================================

if ROS_AVAILABLE:
    print("Setting up ROS integration...")
    
    # Initialize ROS manager
    ros_manager = chros.ChROSPythonManager()
    
    # Register clock handler for time synchronization
    ros_manager.RegisterHandler(
        chros.ChROSClockHandler()
    )
    
    # Register driver inputs handler
    # This handler subscribes to ROS topics for vehicle control
    ros_manager.RegisterHandler(
        chros.ChROSDriverInputsHandler(
            25,  # update rate (Hz)
            driver,
            "~/input/driver_inputs"  # ROS topic name
        )
    )
    
    # Register vehicle state handler
    # This handler publishes vehicle state to ROS topics
    ros_manager.RegisterHandler(
        chros.ChROSBodyHandler(
            25,  # update rate (Hz)
            hmmwv.GetChassisBody(),
            "~/output/vehicle/state"  # ROS topic name
        )
    )
    
    # Initialize the ROS manager
    ros_manager.Initialize()
    
    print("ROS integration initialized.")
else:
    ros_manager = None
    print("Skipping ROS integration (ROS not available).")

# =============================================================================
# SIMULATION LOOP
# =============================================================================

print("Starting simulation loop...")
print(f"Simulation will run for {t_end} seconds with timestep {step_size} s")

# Initialize simulation time
time = 0.0
step_number = 0

# Simple driver input profile (for testing without ROS)
# Gradually increase throttle, then steer, then brake
def get_driver_inputs(t):
    """Generate simple driver inputs for testing."""
    inputs = veh.DriverInputs()
    
    if t < 2.0:
        # Initial phase: gradually apply throttle
        inputs.m_throttle = min(0.5, t * 0.25)
        inputs.m_steering = 0.0
        inputs.m_braking = 0.0
    elif t < 8.0:
        # Driving phase: maintain speed
        inputs.m_throttle = 0.5
        inputs.m_steering = 0.0
        inputs.m_braking = 0.0
    elif t < 12.0:
        # Steering phase: turn left
        inputs.m_throttle = 0.3
        inputs.m_steering = 0.3 * min(1.0, (t - 8.0) / 2.0)
        inputs.m_braking = 0.0
    elif t < 18.0:
        # Straight phase: straighten out
        inputs.m_throttle = 0.4
        inputs.m_steering = 0.0
        inputs.m_braking = 0.0
    else:
        # Braking phase: slow down
        inputs.m_throttle = 0.0
        inputs.m_steering = 0.0
        inputs.m_braking = min(1.0, (t - 18.0) * 0.2)
    
    return inputs

# Main simulation loop
try:
    while time < t_end:
        
        # Get current time
        time = vehicle_system.GetChTime()
        
        # =================================================================
        # SYNCHRONIZE SYSTEMS
        # =================================================================
        
        # Get driver inputs (from ROS or default profile)
        if ros_manager is not None:
            # ROS controls driver inputs via registered handler
            current_driver_inputs = driver.GetInputs()
        else:
            # Use predefined driver input profile
            current_driver_inputs = get_driver_inputs(time)
        
        # Synchronize terrain
        terrain.Synchronize(time)
        
        # Synchronize vehicle
        hmmwv.Synchronize(time, current_driver_inputs, terrain)
        
        # Synchronize driver
        driver.Synchronize(time)
        
        # =================================================================
        # ADVANCE SYSTEMS
        # =================================================================
        
        # Advance terrain
        terrain.Advance(step_size)
        
        # Advance vehicle
        hmmwv.Advance(step_size)
        
        # Advance driver
        driver.Advance(step_size)
        
        # =================================================================
        # UPDATE ROS MANAGER
        # =================================================================
        
        if ros_manager is not None:
            # Update ROS manager to publish/subscribe data
            if not ros_manager.Update(time, step_size):
                print("ROS manager update failed. Stopping simulation.")
                break
        
        # =================================================================
        # PRINT STATUS
        # =================================================================
        
        # Print status every 1 second
        if step_number % int(1.0 / step_size) == 0:
            # Get vehicle position
            pos = hmmwv.GetVehicle().GetPos()
            speed = hmmwv.GetVehicle().GetSpeed()
            
            print(f"Time: {time:.2f}s | "
                  f"Position: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}) | "
                  f"Speed: {speed:.2f} m/s | "
                  f"Throttle: {current_driver_inputs.m_throttle:.2f} | "
                  f"Steering: {current_driver_inputs.m_steering:.2f} | "
                  f"Braking: {current_driver_inputs.m_braking:.2f}")
        
        # Increment step counter
        step_number += 1

except KeyboardInterrupt:
    print("\nSimulation interrupted by user.")

# =============================================================================
# CLEANUP
# =============================================================================

print("\nSimulation completed.")
print(f"Total simulation time: {time:.3f} seconds")
print(f"Total steps executed: {step_number}")

# Final vehicle state
final_pos = hmmwv.GetVehicle().GetPos()
final_speed = hmmwv.GetVehicle().GetSpeed()
print(f"Final position: ({final_pos.x:.3f}, {final_pos.y:.3f}, {final_pos.z:.3f})")
print(f"Final speed: {final_speed:.3f} m/s")