import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import os
import math

# -----------------------------------------------------------------------------
# Main settings
# -----------------------------------------------------------------------------

# The path to the Chrono data directory containing models, textures, etc.
# Try to use the CHRONO_DATA_DIR environment variable if set.
# Otherwise, assume a common structure relative to this script.
chrono_data_dir = os.environ.get('CHRONO_DATA_DIR')
if chrono_data_dir is None:
    # Fallback to a common relative path if the environment variable is not set.
    # This assumes the script is run from a directory where '../../chrono_data' is valid.
    # You might need to adjust this path based on your project structure.
    script_dir = os.path.dirname(os.path.abspath(__file__))
    chrono_data_dir = os.path.join(script_dir, "..", "..", "chrono_data", "")
    if not os.path.isdir(os.path.join(chrono_data_dir, "vehicle")):
        print("Error: CHRONO_DATA_DIR not set and default path not found.")
        print("Please set CHRONO_DATA_DIR or adjust the path in the script.")
        exit(1)

chrono.SetChronoDataPath(chrono_data_dir)
veh.SetDataPath(os.path.join(chrono_data_dir, 'vehicle', ''))

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0.7, 0)  # Slightly above ground
initRot = chrono.ChQuaterniond(1, 0, 0, 0) # No rotation

# Simulation step size and simulation length
step_size = 0.005  # s (integration step)
render_fps = 50    # FPS for visualization and real-time control
sim_time = 120     # s (total simulation time, if not ended by user)

# Contact method
contact_method = chrono.ChContactMethod_NSC # Use NSC (Non-Smooth Contact)

# Tire model
tire_model = veh.TireModelType_TMEASY

# Visualization type for vehicle parts
vis_type_chassis = veh.VisualizationType_PRIMITIVES
vis_type_suspension = veh.VisualizationType_PRIMITIVES
vis_type_steering = veh.VisualizationType_PRIMITIVES
vis_type_wheel = veh.VisualizationType_PRIMITIVES
vis_type_tire = veh.VisualizationType_PRIMITIVES

# -----------------------------------------------------------------------------
# Create the PyChrono System
# -----------------------------------------------------------------------------
print("Creating Chrono system...")
system = chrono.ChSystemNSC() if contact_method == chrono.ChContactMethod_NSC else chrono.ChSystemSMC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# Set solver settings (can be adjusted for performance/accuracy)
system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
system.SetSolverMaxIterations(150)
system.SetMaxPenetrationRecoverySpeed(4.0)

# -----------------------------------------------------------------------------
# Create the HMMWV vehicle
# -----------------------------------------------------------------------------
print("Creating HMMWV vehicle...")
hmmwv = veh.hmmwv.HMMWV_Full()
hmmwv.SetContactMethod(contact_method)
hmmwv.SetChassisCollisionType(veh.CollisionType_PRIMITIVES) # Use primitives for chassis collision
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChFramed(initLoc, initRot))
hmmwv.SetPowertrainType(veh.PowertrainModelType_SIMPLE) # Simple powertrain model
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)             # All-wheel drive
hmmwv.SetTireType(tire_model)
hmmwv.Initialize()

# Set visualization types for vehicle components
hmmwv.SetChassisVisualizationType(vis_type_chassis)
hmmwv.SetSuspensionVisualizationType(vis_type_suspension)
hmmwv.SetSteeringVisualizationType(vis_type_steering)
hmmwv.SetWheelVisualizationType(vis_type_wheel)
hmmwv.SetTireVisualizationType(vis_type_tire)

# Get the vehicle (ChWheeledVehicle object)
vehicle = hmmwv.GetVehicle()
# Associate the ChSystem with the vehicle
vehicle.SetSystem(system)


# -----------------------------------------------------------------------------
# Create the Rigid Terrain
# -----------------------------------------------------------------------------
print("Creating rigid terrain...")
terrain = veh.RigidTerrain(system)

# Define terrain properties
terrain_height = 0.0   # Height of the flat terrain
terrain_dim_x = 200.0  # Size in X direction
terrain_dim_y = 200.0  # Size in Y direction

# Create a contact material for the terrain
patch_mat = chrono.ChContactMaterialNSC() if contact_method == chrono.ChContactMethod_NSC else chrono.ChContactMaterialSMC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

# Add a terrain patch
patch = terrain.AddPatch(patch_mat,
                         chrono.ChVector3d(0, terrain_height, 0),  # Center of the patch
                         chrono.ChVector3d(0, 1, 0),               # Normal direction
                         terrain_dim_x, terrain_dim_y)

# Set texture for the terrain patch
texture_file = veh.GetDataFile("terrain/textures/tile4.jpg")
patch.SetTexture(texture_file, 200, 200) # Texture file, and UV scaling
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5)) # Fallback color if texture fails

terrain.Initialize()

# -----------------------------------------------------------------------------
# Create the Irrlicht visualization system
# -----------------------------------------------------------------------------
print("Creating Irrlicht visualization...")
# For ChWheeledVehicle, ChWheeledVehicleVisualSystemIrrlicht is preferred
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV on Rigid Terrain Demo')
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.75), 6.0, 0.5) # Point on chassis, dist, height
# vis.SetChaseCameraPosition(vehicle.GetPos() + chrono.ChVector3d(0,8,-15)) # Alternative static camera
# vis.SetChaseCameraState(veh.ChChaseCamera.Track ) # Track the vehicle
# vis.SetChaseCameraState(veh.ChChaseCamera.Free) # Free camera
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle)

# Make sure all visual shapes are bound to the Irrlicht scene
vis.BindAll()

# -----------------------------------------------------------------------------
# Create the interactive driver system
# -----------------------------------------------------------------------------
print("Creating interactive driver...")
driver = veh.ChInteractiveDriverIRR(vis)

# Set the time response for steering and throttle/braking inputs
# Smaller values => quicker response
driver.SetSteeringDelta(0.02)  # Adjust for sensitivity
driver.SetThrottleDelta(0.02)  # Adjust for sensitivity
driver.SetBrakingDelta(0.05)   # Adjust for sensitivity
driver.Initialize()

# -----------------------------------------------------------------------------
# Simulation loop
# -----------------------------------------------------------------------------
print("\nSimulation started. Control with:")
print("Steering: A/D keys")
print("Throttle: W key")
print("Braking: S key")
print("Camera: Use mouse and standard Irrlicht controls (e.g., WASD, QE for free camera)")
print("Press ESC to exit.\n")

# Real-time timer
realtime_timer = chrono.ChRealtimeStepTimer()
realtime_timer.SetStep(1.0 / render_fps) # Sync visualization/control to this rate

# Simulation loop
while vis.Run():
    time = system.GetChTime()

    # Update Irrlicht visualization
    vis.BeginScene()
    vis.Render()
    
    # Render GUI elements (like driver inputs)
    driver.DrawAllInfo() # If you want to see throttle/steering values on screen
    
    vis.EndScene()

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Update modules (process inputs from driver)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    hmmwv.Synchronize(time, driver_inputs, terrain) # Pass driver inputs and terrain

    # Advance simulation for one step
    driver.Advance(step_size)
    terrain.Advance(step_size)
    hmmwv.Advance(step_size)
    system.DoStepDynamics(step_size)

    # Spin in place to maintain real-time progression
    realtime_timer.Spin(step_size) # Spin for the duration of the integration step
    
    if time > sim_time : # Optional: end simulation after a certain time
        print(f"Simulation time limit ({sim_time}s) reached.")
        break

# Cleanup (PyChrono objects are usually garbage collected, but good practice for some cases)
# No explicit C++ 'delete' needed in Python bindings for most objects managed by shared_ptr
print("Simulation ended.")

# ...
        # Simulation step size and simulation length
        render_fps = 50    # FPS for visualization and real-time control
        step_size = 1.0 / render_fps # Physics step matches render/control step
        # ...
        # Real-time timer
        realtime_timer = chrono.ChRealtimeStepTimer() 
        # No SetStep here, Spin will use the actual time taken for step_size
        # ...
        # In the loop:
        # ...
        realtime_timer.Spin(step_size) # Spin to ensure each iteration takes at least 'step_size' in wall clock time

# ...
    # Simulation step size and simulation length
    target_fps = 50    # FPS for visualization, control, and physics updates
    step_size = 1.0 / target_fps 
    sim_time = 120     # s (total simulation time, if not ended by user)
    # ...
    # Real-time timer
    realtime_timer = chrono.ChRealtimeStepTimer()
    # realtime_timer.SetStep(step_size) # Spin will use this as target duration for each step
    # ...
    # In the loop:
    # ...
    realtime_timer.Spin(step_size) # Ensure current step took at least step_size wall-clock time