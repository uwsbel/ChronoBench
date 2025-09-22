import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Path settings (assuming CHRONO_DATA_DIR is set appropriately)
# Ensure CHRONO_DATA_DIR environment variable points to the Chrono data directory
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# --- Custom Driver Class Implementation ---
class MyDriver(veh.ChDriver):
    def __init__(self, vehicle_object, delay):
        """
        Custom driver class.
        :param vehicle_object: The ChWheeledVehicle this driver controls.
        :param delay: Delay in seconds for driver inputs.
        """
        super().__init__(vehicle_object)
        self.delay = delay
        # Initialize driver inputs (these set m_throttle_input, m_steering_input, etc.)
        self.SetThrottle(0.0)
        self.SetSteering(0.0)
        self.SetBraking(0.0)

    def Synchronize(self, time):
        """
        Overrides ChDriver.Synchronize.
        Calculates and sets throttle, steering, and braking based on simulation time
        and the specified delay.
        """
        effective_time = time - self.delay

        current_throttle = 0.0
        current_steering = 0.0
        current_braking = 0.0  # Braking is always 0 as per implicit requirements

        if effective_time >= 0: # Only apply control logic if effective_time is non-negative
            # Throttle control:
            # Gradually increases to 0.7 after 0.2 seconds (effective_time).
            # Interpreted as: throttle is 0.7 if effective_time >= 0.2s.
            if effective_time >= 0.2:
                current_throttle = 0.7
            # else: current_throttle remains 0.0 (initialized above)

            # Steering control:
            # Sinusoidal pattern starting at 2 seconds (effective_time).
            # Amplitude 0.5, frequency 1 Hz.
            if effective_time >= 2.0:
                # steering = A * sin(2 * pi * f * (effective_time - start_time_steering))
                current_steering = 0.5 * math.sin(2 * math.pi * 1.0 * (effective_time - 2.0))
            # else: current_steering remains 0.0 (initialized above)
        
        # Update the driver inputs through the ChDriver's interface.
        # These values (m_throttle_input, etc.) will be used by ChDriver::Advance()
        # to set the actual m_throttle, m_steering, m_braking.
        self.SetThrottle(current_throttle)
        self.SetSteering(current_steering)
        self.SetBraking(current_braking)

# --- Simulation Parameters ---

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0) # Scalar component first

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_PRIMITIVES

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE # No chassis collision shapes

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain parameters
terrainHeight = 0.0    # Height of the terrain surface (Z coordinate)
terrainLength = 100.0  # Size in X direction
terrainWidth = 100.0   # Size in Y direction

# Point on chassis for camera tracking
trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)

# Contact method
contact_method = chrono.ChContactMethod_NSC

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size # Tire step size often same as simulation step size

# Time interval between two render frames
render_step_size = 1.0 / 50  # Target 50 FPS

# Simulation end time condition
simulation_end_time = 4.0

# --- Create and Configure Chrono Objects ---

# Create the HMMWV vehicle, set parameters, and initialize
vehicle = veh.HMMWV_Full() # Could also be veh.HMMWV_Reduced()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize() # This also sets up the collision system based on contact_method

# Set visualization types for vehicle components
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# Create the terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
# Add a terrain patch (flat box)
patch = terrain.AddPatch(patch_mat, 
                         chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrainHeight), chrono.QUNIT), 
                         terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create the Irrlicht visualization system
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Custom Driver Demo') # Updated window title
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5) # Camera parameters: target point, chase distance, height
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())


# --- Driver System Replacement ---
# Replace ChInteractiveDriverIRR with MyDriver
driver_delay = 0.5  # Delay parameter for the custom driver
driver = MyDriver(vehicle.GetVehicle(), driver_delay)
# MyDriver does not require an explicit Initialize() call like ChInteractiveDriverIRR
# driver.Initialize() # This line is removed as it's not needed for MyDriver

# Output vehicle mass
print(f"VEHICLE MASS: {vehicle.GetVehicle().GetMass()}")

# --- Simulation Loop ---

# Number of simulation steps between render frames
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation counters
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

# Main simulation loop
while vis.Run(): # Loop continues as long as Irrlicht window is open
    time = vehicle.GetSystem().GetChTime()

    # Simulation End Condition: Stop if simulation time reaches 4 seconds
    if time >= simulation_end_time:
        print(f"Simulation time {time:.2f}s has reached the limit of {simulation_end_time:.2f}s. Ending simulation.")
        break

    # Render scene at specified intervals
    if (step_number % render_steps == 0):
        vis.BeginScene(True, True, chrono.ChColor(0.3,0.3,0.3)) # Clear background and Z-buffer
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Synchronize modules: update data at current time
    driver.Synchronize(time)    # MyDriver calculates its throttle/steering/braking inputs
    terrain.Synchronize(time)
    # Vehicle pulls inputs from the driver object
    vehicle.Synchronize(time, driver, terrain) 
    # Visualization system updates based on driver inputs (for dashboard, etc.)
    vis.Synchronize(time, driver.GetInputs())

    # Advance simulation state for one timestep
    driver.Advance(step_size)   # Custom driver advances (base class applies inputs)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)      # Advance visualization animation time

    # Increment step number
    step_number += 1

    # Spin in place to maintain real-time progression (if simulation is faster than wall clock)
    realtime_timer.Spin(step_size)

# Simulation ended. No explicit cleanup usually required for vis in Python.