import pychrono as chrono
import pychrono.vehicle as chronoveh
import pychrono.irrlicht as chrono_irr
import os
import math

# -------------------------------------------------------------------------------
# Set path to Chrono data files
# -------------------------------------------------------------------------------
# Attempt to set CHRONO_DATA_DIR if not already set
if 'CHRONO_DATA_DIR' not in os.environ:
    # Try to guess the path relative to this script
    # This might need adjustment based on your directory structure
    script_dir = os.path.dirname(os.path.abspath(__file__))
    chrono_data_path = os.path.join(script_dir, "..", "..", "..", "chrono_data", "")
    if os.path.exists(os.path.join(chrono_data_path, "vehicle")):
        os.environ['CHRONO_DATA_DIR'] = chrono_data_path
    else:
        print("Error: CHRONO_DATA_DIR environment variable not set.")
        print("Please set CHRONO_DATA_DIR to the location of your Chrono data files.")
        exit(1)

chrono.SetChronoDataPath(os.environ['CHRONO_DATA_DIR'])
chronoveh.SetDataPath(os.path.join(os.environ['CHRONO_DATA_DIR'], 'vehicle', ''))

# -------------------------------------------------------------------------------
# Simulation parameters
# -------------------------------------------------------------------------------
# Simulation step size
step_size = 0.005  # seconds (for 200 Hz physics)

# Simulation end time
time_end = 100  # seconds

# Target rendering frames per second
render_fps = 50.0
render_step_size = 1.0 / render_fps # seconds per frame (e.g., 0.02s for 50 FPS)

# Initial vehicle location and orientation
initLoc = chrono.ChVectorD(0, 0.7, 0) # x, y, z position
initRot = chrono.ChQuaternionD(1, 0, 0, 0) # Identity quaternion (no rotation)

# Contact method
contact_method = chrono.ChContactMethod_SMC # or ChContactMethod_NSC

# Visualization type for vehicle parts
chassis_vis_type = chronoveh.VisualizationType_MESH
suspension_vis_type = chronoveh.VisualizationType_PRIMITIVES
steering_vis_type = chronoveh.VisualizationType_PRIMITIVES
wheel_vis_type = chronoveh.VisualizationType_MESH
tire_vis_type = chronoveh.VisualizationType_MESH

# Terrain dimensions
terrain_height = 0.0 # y level of the terrain
terrain_size_x = 200 # meters
terrain_size_z = 200 # meters

# -------------------------------------------------------------------------------
# Create the Chrono system and ARTcar vehicle
# -------------------------------------------------------------------------------
print("Creating Chrono system...")
if contact_method == chrono.ChContactMethod_SMC:
    sys = chrono.ChSystemSMC()
else:
    sys = chrono.ChSystemNSC()

sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
sys.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN) # Recommended for SMC
sys.SetSolverMaxIterations(150)
sys.SetMaxPenetrationRecoverySpeed(4.0)


print("Creating ARTcar vehicle...")
vehicle = chronoveh.ARTcar()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chronoveh.CollisionType_NONE) # No chassis collision for simplicity

# Initialize vehicle at specified position and orientation
vehicle.Initialize(chrono.ChCoordsysD(initLoc, initRot))

# Set visualization types
vehicle.SetChassisVisualizationType(chassis_vis_type)
vehicle.SetSuspensionVisualizationType(suspension_vis_type)
vehicle.SetSteeringVisualizationType(steering_vis_type)
vehicle.SetWheelVisualizationType(wheel_vis_type)
# Note: ARTcar uses a simple PAC02 tire model which might not have specific tire mesh.
# Tire visualization is often part of the wheel visualization for simpler models.
# If using more advanced tires like TMeasy or Pacejka, they have their own vis.
# For ARTcar, its tires are Pac02, which are analytical, so tire_vis_type might not apply directly.
# Let's assume wheel_vis_type covers the visual representation of the tire too.

print(f"Vehicle initialized. Mass: {vehicle.GetVehicleMass()} kg")

# -------------------------------------------------------------------------------
# Create the rigid terrain
# -------------------------------------------------------------------------------
print("Creating rigid terrain...")
terrain = chronoveh.RigidTerrain(sys)

# Define material properties for the terrain patch
if contact_method == chrono.ChContactMethod_SMC:
    patch_material = chrono.ChMaterialSurfaceSMC()
    patch_material.SetFriction(0.9)
    patch_material.SetRestitution(0.01)
    patch_material.SetYoungModulus(2e7)
    patch_material.SetPoissonRatio(0.3)
else: # NSC
    patch_material = chrono.ChMaterialSurfaceNSC()
    patch_material.SetFriction(0.9)
    patch_material.SetRestitution(0.01)

# Add a terrain patch
patch = terrain.AddPatch(patch_material,
                         chrono.CSYSNORM, # Centered at origin, Y up
                         terrain_size_x, terrain_size_z,
                         terrain_height) # Thickness, but for display it's effectively a plane

# Set texture for the terrain patch
texture_file = chronoveh.GetDataFile("terrain/textures/tile4.jpg")
patch.SetTexture(texture_file, 200, 200) # Texture file, repeats every 200x200 m

# Set a fallback color (if texture loading fails)
patch.SetColor(chrono.ChColor(0.5, 0.8, 0.5))

terrain.Initialize()
print("Terrain initialized.")

# -------------------------------------------------------------------------------
# Create the Irrlicht application and interactive driver
# -------------------------------------------------------------------------------
print("Creating Irrlicht application and driver...")
app = chrono_irr.ChIrrApp(sys, "ARTcar on Rigid Terrain", chrono_irr.dimension2du(1280, 720))
app.SetHUDDisplay(True) # Display a simple HUD

# Create the interactive driver system
driver = chronoveh.ChIrrGuiDriver(app)

# Set the time response for steering, throttle and braking inputs
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.04) # Increased for ARTcar responsiveness
driver.SetBrakingDelta(0.10) # Increased for ARTcar responsiveness

# Attach the driver to the vehicle
driver.Initialize()

# Configure the chase camera (managed by ChIrrGuiDriver)
# The ChIrrGuiDriver creates its own camera. We can configure it.
# trackPoint is relative to the chassis reference frame
trackPoint = chrono.ChVectorD(0.0, 0.0, 0.0) # Look at chassis center
# Use vehicle's suggested chase camera parameters if available
chase_dist = vehicle.GetOptimalChaseDistance() if hasattr(vehicle, 'GetOptimalChaseDistance') else 8.0
chase_height = vehicle.GetOptimalChaseHeight() if hasattr(vehicle, 'GetOptimalChaseHeight') else 2.0

driver.SetChaseCamera(trackPoint, chase_dist, chase_height)
driver.SetChaseCameraState(chronoveh.ChChaseCamera.Track) # Ensure it's in tracking mode
driver.SetChaseCameraAngle(-math.pi / 8.0) # Slight downward angle

# Ensure all assets are bound and updated in Irrlicht
# ChIrrGuiDriver's Initialize() method typically calls app.AssetBindAll() and app.AssetUpdateAll()
# So, these might be redundant but are harmless.
app.AssetBindAll()
app.AssetUpdateAll()

# -------------------------------------------------------------------------------
# Simulation loop
# -------------------------------------------------------------------------------
print(f"Starting simulation. Target FPS: {render_fps}. Physics step: {step_size}s.")
print("Controls: ")
print(" W/S: Throttle/Brake")
print(" A/D: Steering Left/Right")
print(" Mouse or Arrow Keys: Camera control (depending on camera mode)")
print(" C: Cycle camera modes")

realtime_timer = chrono.ChRealtimeStepTimer()
simulation_time = 0.0

while app.GetDevice().run():
    current_sim_time = sys.GetChTime()

    # --- Rendering ---
    # This happens once per render_step_size wall-clock time
    app.BeginScene(True, True, chrono.ChColor(0.1, 0.2, 0.3)) # Clear color
    # Get driver inputs for HUD. Inputs are polled by Irrlicht continuously.
    # For HUD display, use the latest inputs.
    driver_inputs_for_hud = driver.GetInputs()
    app.Synchronize(driver.GetInputModeAsString(), driver_inputs_for_hud)
    app.DrawAll()
    app.EndScene()

    # --- Physics Simulation ---
    # Advance simulation to catch up to the time for the next rendered frame.
    # The simulation time should advance by render_step_size for each rendered frame.
    target_sim_time_for_next_render = simulation_time + render_step_size

    while sys.GetChTime() < target_sim_time_for_next_render:
        # Determine the actual physics step to take, ensuring it doesn't overshoot
        # the target time for this sub-step within the render frame.
        time_now = sys.GetChTime()
        actual_physics_step = min(step_size, target_sim_time_for_next_render - time_now)
        
        if actual_physics_step < 1e-7: # Avoid infinitesimally small steps
            break

        # Get driver inputs (these are applied at physics rate)
        driver_inputs = driver.GetInputs()

        # Synchronize modules (process inputs from other modules)
        driver.Synchronize(time_now) # Synchronize driver (e.g. update internal states based on time)
        vehicle.Synchronize(time_now, driver_inputs, terrain) # Update vehicle with inputs
        terrain.Synchronize(time_now) # Update terrain (e.g. if deformable)
        # app.Synchronize is done once per render frame for HUD.

        # Advance simulation by one physics step.
        # ChIrrGuiDriver.Advance() handles advancing the driver logic,
        # the application's state (like camera), and the physical system itself.
        driver.Advance(actual_physics_step)
    
    simulation_time += render_step_size

    # Spin the timer to try to match wall-clock rendering rate
    realtime_timer.Spin(render_step_size)

    if sys.GetChTime() > time_end:
        print("Simulation ended.")
        break

# Cleanup
print("Exiting simulation.")
# Irrlicht device is automatically released when 'app' goes out of scope.