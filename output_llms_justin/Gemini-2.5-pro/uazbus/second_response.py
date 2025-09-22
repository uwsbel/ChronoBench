import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math
import os # Though not strictly needed for this version, often useful

# Ensure CHRONO_DATA_DIR is set correctly in your environment
chrono.SetChronoDataPath(os.environ.get('CHRONO_DATA_DIR', '')) # Use environment variable
veh.SetDataPath(os.path.join(chrono.GetChronoDataPath(), 'vehicle/'))

# Initial vehicle location and orientation
# MODIFIED: Initial vehicle position changed
initLoc = chrono.ChVector3d(-40, 0, 0.5) 
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE # No chassis collision for this demo

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain
terrainHeight = 0      # terrain height
terrainLength = 200.0  # size in X direction (increased to accommodate maneuver)
terrainWidth = 100.0   # size in Y direction

# Point on chassis tracked by the camera
trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1) # Relative to vehicle CoG

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False # Not used in this script, but kept for consistency

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50


# Create the UAZBUS vehicle, set parameters, and initialize
vehicle = veh.UAZBUS() 
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

# Set the collision system type for the entire system
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create the terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrainHeight - 0.1), chrono.QUNIT), # Ensure patch is slightly below vehicle init
    terrainLength, terrainWidth)

# MODIFIED: Terrain texture changed
# CORRECTED: Texture path now uses chrono.GetChronoDataFile for correct path resolution
patch.SetTexture(chrono.GetChronoDataFile("terrain/textures/concrete.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('UAZBUS Double Lane Change Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# MODIFIED: Removed ChInteractiveDriverIRR for programmed maneuver
# Create a driver inputs structure
driver_inputs = veh.DriverInputs()

# output vehicle mass
print( "VEHICLE MASS: ",  vehicle.GetVehicle().GetMass())

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter 
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

# --- MODIFIED: Parameters for Double Lane Change Maneuver ---
throttle_value = 0.7    # Throttle during acceleration and maneuver

t_accelerate_end = 3.0  # Time to accelerate before starting maneuver

# First lane change (left)
steer_angle_1 = 0.4     # Normalized steering input for first turn
steer_1_start_time = t_accelerate_end
steer_1_peak_time = steer_1_start_time + 0.75
steer_1_end_time = steer_1_peak_time + 0.75 # Time to return steering to zero

# Straight section
straight_1_duration = 1.0
straight_1_end_time = steer_1_end_time + straight_1_duration

# Second lane change (right)
steer_angle_2 = -0.4    # Normalized steering input for second turn
steer_2_start_time = straight_1_end_time
steer_2_peak_time = steer_2_start_time + 0.75
steer_2_end_time = steer_2_peak_time + 0.75 # Time to return steering to zero

# Straight section after second lane change
straight_2_duration = 1.0
straight_2_end_time = steer_2_end_time + straight_2_duration

# Braking phase
t_brake_start = straight_2_end_time
braking_value = 0.8
t_brake_end = t_brake_start + 2.0

# End of simulation
t_simulation_end = t_brake_end + 1.0


# Simulation loop
while vis.Run() :
    time = vehicle.GetSystem().GetChTime()

    # --- MODIFIED: Programmed Double Lane Change Logic ---
    current_steering = 0.0
    current_throttle = 0.0
    current_braking = 0.0

    if time < t_accelerate_end: # Phase 1: Acceleration
        current_throttle = throttle_value
    elif time < steer_1_peak_time: # Phase 2a: Steer left (ramp up)
        current_throttle = throttle_value
        current_steering = steer_angle_1 * (time - steer_1_start_time) / (steer_1_peak_time - steer_1_start_time)
    elif time < steer_1_end_time: # Phase 2b: Steer left (ramp down to 0)
        current_throttle = throttle_value
        current_steering = steer_angle_1 * (steer_1_end_time - time) / (steer_1_end_time - steer_1_peak_time)
    elif time < straight_1_end_time: # Phase 3: Drive straight
        current_throttle = throttle_value
        current_steering = 0.0
    elif time < steer_2_peak_time: # Phase 4a: Steer right (ramp up)
        current_throttle = throttle_value
        current_steering = steer_angle_2 * (time - steer_2_start_time) / (steer_2_peak_time - steer_2_start_time)
    elif time < steer_2_end_time: # Phase 4b: Steer right (ramp down to 0)
        current_throttle = throttle_value
        current_steering = steer_angle_2 * (steer_2_end_time - time) / (steer_2_end_time - steer_2_peak_time)
    elif time < t_brake_start: # Phase 5: Drive straight
        current_throttle = throttle_value # Maintain speed or set to 0 to coast
        current_steering = 0.0
    elif time < t_brake_end: # Phase 6: Braking
        current_throttle = 0.0
        current_braking = braking_value
    else: # Phase 7: Idle or end
        current_throttle = 0.0
        current_braking = 0.1 # Light brake to hold position
        if time > t_simulation_end:
            vis.Quit() # End simulation

    # Update driver inputs
    driver_inputs.m_steering = chrono.ChClamp(current_steering, -1.0, 1.0)
    driver_inputs.m_throttle = chrono.ChClamp(current_throttle, 0.0, 1.0)
    driver_inputs.m_braking = chrono.ChClamp(current_braking, 0.0, 1.0)
    
    # Render scene and output POV-Ray data
    if (step_number % render_steps == 0) :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    # Update modules (process inputs from other modules)
    # No driver.Synchronize(time) as ChInteractiveDriverIRR is removed
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs) # vis can still show steering wheel based on driver_inputs

    # Advance simulation for one timestep for all modules
    # No driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Increment frame number
    step_number += 1
    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)