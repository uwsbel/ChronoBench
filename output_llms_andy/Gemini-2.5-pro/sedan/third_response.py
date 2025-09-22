import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math
import os # For potential CHRONO_DATA_DIR handling, though not strictly used in final path setting

# --- Path Setup ---
# This method relies on PyChrono's ability to find its data directory.
# If CHRONO_DATA_DIR environment variable is set, chrono.GetChronoDataPath() will use it.
# Otherwise, it might use a path compiled into Chrono or relative to the executable.
# The "!!!! Set this path..." comment in the original implies users might need to intervene.
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# --- Simulation Parameters ---

# Instruction: Initial vehicle location and orientation adjusted.
# Adjusted for starting on a typical highway mesh.
initLoc = chrono.ChVector3d(5, 1.75, 0.5)  # Start 5m along X, in a lane (Y=1.75m assumed), 0.5m height
initRot = chrono.ChQuaterniond(1, 0, 0, 0)  # No initial rotation

# Visualization type for vehicle parts (MESH is good for detail)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (NONE means no chassis collision geometry is created)
chassis_collision_type = veh.CollisionType_NONE # Kept as per original script

# Type of tire model (TMEASY is a common and computationally efficient choice)
tire_model = veh.TireModelType_TMEASY

# Instruction: Terrain initialized with a highway mesh.
# The specific mesh file; ensure this path is correct relative to CHRONO_DATA_DIR/vehicle/
terrain_mesh_file = veh.GetDataFile("terrain/meshes/StraightMeshed.obj")
# Note: If "StraightMeshed.obj" is not found, this will raise an error.

# Instruction: Point on chassis tracked by the camera (comment corrected)
# Point on chassis (relative to chassis reference frame) for the chase camera
trackPoint = chrono.ChVector3d(-5.0, 0.0, 1.8)

# Contact method for the simulation (NSC: Non-Smooth Contact method)
contact_method = chrono.ChContactMethod_NSC
# contact_vis = False # This variable was unused in the original script

# Instruction: Decreased simulation step size and render step size for finer control.
step_size = 0.0005  # Simulation step size in seconds (e.g., 0.5 ms)
tire_step_size = step_size  # Tire dynamics step size, usually same as simulation step_size

# Time interval between two render frames (determines FPS)
render_step_size = 1.0 / 100.0  # Target 100 FPS

# Instruction: Reference speed input added for controlling the vehicle's speed.
target_speed_mps = 15.0  # Target speed in meters/second (15 m/s = 54 km/h)

# Instruction: PID controller implemented for throttle control based on speed error.
# PID Controller parameters for throttle (these may require tuning)
Kp = 0.8  # Proportional gain
Ki = 0.3  # Integral gain (Increased a bit to help overcome stiction/reach target)
Kd = 0.1  # Derivative gain (Helps dampen oscillations)

# PID state variables
integral_error = 0.0
previous_error = 0.0

# --------------
# Create systems
# --------------

# Create the BMW E90 vehicle, set parameters, and initialize
vehicle = veh.BMW_E90()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)  # Vehicle is not fixed to the ground
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize() # Finalizes vehicle setup

# Set visualization type for various parts of the vehicle
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# Set the collision system type for the entire Chrono system
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create the terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)      # High friction typical for asphalt
patch_mat.SetRestitution(0.01)  # Low restitution

terrain = veh.RigidTerrain(vehicle.GetSystem())
# Add the highway mesh patch. The Z-offset (-0.1) for the patch_cs
# is often used if the mesh's origin is at Z=0 and vehicles spawn slightly above.
patch_cs = chrono.ChCoordsysd(chrono.ChVector3d(0, 0, -0.1), chrono.QUNIT)
patch = terrain.AddPatch(patch_mat, patch_cs, terrain_mesh_file)

# Set a fallback color for the mesh if it doesn't have its own materials.
# Textures are usually part of the mesh's .mtl file if it's an OBJ.
patch.SetColor(chrono.ChColor(0.5, 0.5, 0.5)) # A neutral grey

terrain.Initialize()

# Create the vehicle Irrlicht visualization system
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('BMW E90 on Highway - PID Speed Control') # Updated window title
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5) # Camera parameters: point, distance, height
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png')) # Add PyChrono logo
vis.AddLightDirectional() # Add a default light source
vis.AddSkyBox() # Add a skybox for a more immersive background
vis.AttachVehicle(vehicle.GetVehicle()) # Attach the vehicle to the visualization

# Create the driver system (for interactive steering and braking)
driver = veh.ChInteractiveDriverIRR(vis)

# Instruction: Increased steering response time to 5 seconds.
# Set the time response for steering, throttle, and braking keyboard inputs.
steering_time = 5.0  # Time (sec) to go from 0 to +1 (or 0 to -1) steering
throttle_time = 1.0  # Original throttle time (PID will override throttle input)
braking_time = 0.3   # Time (sec) to go from 0 to +1 braking

# Delta specifies how much input changes per render frame when key is held
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time) # Will be overridden by PID
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()

# ---------------
# Simulation loop
# ---------------

print(f"VEHICLE MASS: {vehicle.GetVehicle().GetMass()} kg")
print(f"Target speed set to: {target_speed_mps:.2f} m/s ({target_speed_mps * 3.6:.2f} km/h)")
print(f"PID Parameters: Kp={Kp}, Ki={Ki}, Kd={Kd}")
print("Controls: Steering (A/D keys), Braking (S key). Throttle is PID controlled.")
print("Close the Irrlicht window or press 'Q' (if bound by Irrlicht) to exit.")


# Number of simulation steps between render frames
render_steps = math.ceil(render_step_size / step_size)

realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Get driver inputs (steering and braking from keyboard)
    driver_inputs = driver.GetInputs()

    # --- PID Controller Logic for Throttle ---
    current_speed = vehicle.GetVehicle().GetSpeed() # Get current vehicle speed
    error = target_speed_mps - current_speed        # Calculate speed error

    # Integral term accumulation
    integral_error += error * step_size
    # Anti-windup for integral term (basic clamping)
    # Limits the maximum contribution of the integral term to prevent excessive overshoot
    # or slow recovery if the system is saturated for a long time.
    max_integral_contribution = 0.5 # Example: integral term can contribute up to 0.5 to throttle
    if Ki != 0: # Avoid division by zero if Ki is intentionally set to 0
        integral_limit = abs(max_integral_contribution / Ki)
        integral_error = max(-integral_limit, min(integral_error, integral_limit))
    
    # Derivative term calculation
    if step_size > 1e-9: # Avoid division by zero for very small step_size
        derivative_error = (error - previous_error) / step_size
    else:
        derivative_error = 0.0
    previous_error = error # Update previous_error for the next iteration

    # Calculate PID output signal for throttle
    throttle_signal = Kp * error + Ki * integral_error + Kd * derivative_error
    
    # Apply throttle:
    # If target speed is very low (e.g., near zero), command zero throttle.
    # Otherwise, clamp the PID output to the valid throttle range [0, 1].
    if target_speed_mps < 0.1:  # Threshold for considering target speed as effectively zero
        applied_throttle = 0.0
    else:
        applied_throttle = max(0.0, min(throttle_signal, 1.0))
    
    # Override the throttle from the interactive driver with the PID-calculated throttle
    driver_inputs.m_throttle = applied_throttle
    
    # --- Rendering ---
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        # Example of adding on-screen text (optional)
        # font = vis.GetGUIEnvironment().getBuiltInFont()
        # text_to_display = (f"Time: {time:.2f}s\n"
        #                    f"Speed: {current_speed:.2f} m/s (Target: {target_speed_mps:.2f} m/s)\n"
        #                    f"Throttle (PID): {applied_throttle:.2f}\n"
        #                    f"Steering: {driver_inputs.m_steering:.2f}\n"
        #                    f"Braking: {driver_inputs.m_braking:.2f}")
        # font.draw(text_to_display, chrono.ChRecti(10, 10, 400, 100), chrono.ChColor(0,0,0,200))
        vis.EndScene()
        
        # Optional: Print status to console periodically for debugging
        # print(f"T: {time:.2f} | Target: {target_speed_mps:.1f} | Actual: {current_speed:.1f} | Thr: {applied_throttle:.2f}")


    # Update modules (process inputs from other modules)
    driver.Synchronize(time) # Synchronize driver (e.g., Irrlicht event processing)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain) # Pass PID-modified driver inputs
    vis.Synchronize(time, driver_inputs) # Visualization sync

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)
    
    # Increment simulation step number
    step_number += 1

    # Spin in place for real time to catch up (attempts to run simulation in real-time)
    realtime_timer.Spin(step_size)

# Simulation ends when vis.Run() returns false (e.g., window closed)