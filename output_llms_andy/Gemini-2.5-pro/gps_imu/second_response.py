import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math
import os # Often useful for path management, though not strictly needed for the SetChronoDataPath line as is.

# Set the Chrono data directory
# Ensure CHRONO_DATA_DIR is set in your environment, or modify this path:
# chrono.SetChronoDataPath(os.environ.get('CHRONO_DATA_DIR', 'fallback_path_to_chrono_data'))
chrono.SetChronoDataPath(chrono.GetChronoDataPath()) 
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain parameters
terrainHeight = 0      # Terrain height
terrainLength = 100.0  # Size in X direction
terrainWidth = 100.0   # Size in Y direction

# Point on the chassis tracked by the camera
trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False # Not actively used in this script for visualization

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# --- Modification: Added Logging Step Size ---
log_step_size = 0.1  # Log data every 0.1 seconds

# Create the HMMWV vehicle, set parameters, and initialize
vehicle = veh.HMMWV_Full() # veh.HMMWV_Reduced() could be another choice
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()

# Set visualization types for vehicle parts
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# Set collision system type (typically set by vehicle.Initialize based on contact method, but can be set explicitly)
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create the terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat,
                         chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrainHeight - 0.05), chrono.QUNIT), # Center patch slightly below terrainHeight if it's a box
                         terrainLength, terrainWidth, 0.1) # Added thickness for clarity, adjust as needed
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo with Programmatic Driver and GPS Logging')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# --- Modification: Modified Driver Inputs (Replaces ChInteractiveDriverIRR) ---
# The ChInteractiveDriverIRR is removed. We will create and manage DriverInputs directly.
# driver = veh.ChInteractiveDriverIRR(vis)
# # Set the time response for steering and throttle keyboard inputs
# steering_time = 1.0  # Time to go from 0 to +1 (or from 0 to -1)
# throttle_time = 1.0  # Time to go from 0 to +1
# braking_time = 0.3   # Time to go from 0 to +1
# driver.SetSteeringDelta(render_step_size / steering_time)
# driver.SetThrottleDelta(render_step_size / throttle_time)
# driver.SetBrakingDelta(render_step_size / braking_time)
# driver.Initialize()
driver_inputs = veh.DriverInputs() # Create a DriverInputs object

# Initialize sensor manager
manager = sens.ChSensorManager(vehicle.GetSystem())
# manager.SetVerbose(True) # Optional: for debugging sensor updates

# Create an Accelerometer sensor (referred to as IMU in original comments) and add it to the manager
# --- Error Correction: Changed chrono.ChFramed to chrono.ChFrameD ---
offset_pose = chrono.ChFrameD(chrono.ChVector3d(-8, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
imu = sens.ChAccelerometerSensor(vehicle.GetChassisBody(), # Body IMU is attached to
                                 100,        # Update rate in Hz (increased for smoother data if needed, e.g. 100Hz)
                                 offset_pose,          # Offset pose
                                 sens.ChNoiseNone())   # Noise model
imu.SetName("Accelerometer Sensor") # Changed name for clarity
imu.SetLag(0)
imu.SetCollectionWindow(0)
# Provides the host access to the IMU data
imu.PushFilter(sens.ChFilterAccelAccess()) # Correct filter for Accelerometer
# Add the IMU to the sensor manager
manager.AddSensor(imu)

# Create a GPS sensor and add it to the manager
gps_update_rate = 10 # Hz
gps = sens.ChGPSSensor(vehicle.GetChassisBody(),                     # Body GPS is attached to
                       gps_update_rate,        # Update rate in Hz
                       offset_pose,          # Offset pose (using the same offset as IMU for this example)
                       chrono.ChVector3d(-89.400, 43.070, 260.0),  # GPS reference point (Madison, WI)
                       sens.ChNoiseNone())   # Noise model
gps.SetName("GPS Sensor")
gps.SetLag(0)
gps.SetCollectionWindow(0) # Get the most recent reading
# Provides the host access to the GPS data
gps.PushFilter(sens.ChFilterGPSAccess()) # Correct filter for GPS
# Add the GPS to the sensor manager
manager.AddSensor(gps)

# ---------------
# Simulation loop
# ---------------

# Output vehicle mass
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)
# --- Modification: Added GPS Data Logging ---
log_steps = math.ceil(log_step_size / step_size)

# --- Modification: Initialized GPS Data List ---
gps_data = []

# Initialize simulation frame counter
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
# render_frame = 0 # Not strictly necessary for this version

# Simulation loop
while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render scene
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        # render_frame += 1 # Not strictly necessary

    # --- Modification: Modified Driver Inputs ---
    # Get driver inputs (programmatically set)
    if time < 6.0:
        driver_inputs.m_throttle = 0.5  # Apply 50% throttle
        driver_inputs.m_steering = 0.2  # Apply some steering (e.g., 0.2 for a gentle turn)
        driver_inputs.m_braking = 0.0   # No braking
    else:
        driver_inputs.m_throttle = 0.0   # Release throttle
        driver_inputs.m_steering = 0.0   # Straighten steering (or maintain last, depends on desired behavior)
        driver_inputs.m_braking = 1.0    # Apply full braking

    # Update modules (process inputs from other modules)
    # driver.Synchronize(time) # Removed as ChInteractiveDriverIRR is not used
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain) # Pass the programmatic driver_inputs
    vis.Synchronize(time, driver_inputs) # Pass driver_inputs for potential HUD updates

    # Advance simulation for one timestep for all modules
    # driver.Advance(step_size) # Removed
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size) # Must be advanced for Irrlicht window to respond

    # Update sensor manager in each step
    manager.Update()

    # --- Modification: Added GPS Data Logging ---
    if step_number % log_steps == 0:
        gps_buffer = gps.GetMostRecentGPSBuffer() # Get the buffer from the sensor
        if gps_buffer and gps_buffer.HasData(): # Check if buffer is valid and has data
            # GetGPSData() returns a list of ChVector3d, for collection_window=0, it's one point
            current_gps_coords_list = gps_buffer.GetGPSData()
            if current_gps_coords_list: # Ensure the list is not empty
                # Store the ChVector3d object (latitude, longitude, altitude)
                gps_data.append(current_gps_coords_list[0])
                # To print live data (optional):
                # print(f"Time: {time:.2f} GPS: {current_gps_coords_list[0].x}, {current_gps_coords_list[0].y}, {current_gps_coords_list[0].z}")


    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)

# --- Modification: Added GPS Data Output ---
print("\nSimulation Ended.")
print(f"Collected {len(gps_data)} GPS data points.")
# For cleaner printing of ChVector3d data, you might iterate and format
print("GPS Data (Lat, Long, Alt):")
for i, data_point in enumerate(gps_data):
    print(f"  Point {i}: Lat={data_point.x:.6f}, Long={data_point.y:.6f}, Alt={data_point.z:.2f}")
# Original requested print format:
# print("GPS Data: ", gps_data)

# Cleanup (optional, good practice if PyChrono or Irrlicht needs explicit shutdown)
# vis.EndScene() # If loop exited prematurely
# No explicit destructor calls usually needed for these Python objects due to garbage collection