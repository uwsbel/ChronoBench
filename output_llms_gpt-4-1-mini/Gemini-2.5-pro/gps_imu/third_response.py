import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math
import matplotlib.pyplot as plt # Added for plotting

# Set the Chrono data directory
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
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50
log_step_size = 1.0 / 20    # Frequency of data logging

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

# Set collision system type
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create the terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat,
                         chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrainHeight), chrono.QUNIT), # Corrected to use terrainHeight
                         terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo with Modified Sensors and Controls')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create the driver system
driver = veh.ChInteractiveDriverIRR(vis)

# Set the time response for steering and throttle keyboard inputs
# These are for interactive mode, but we'll override inputs in the loop
steering_time = 1.0  # Time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0  # Time to go from 0 to +1
braking_time = 0.3   # Time to go from 0 to +1
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# Initialize sensor manager
manager = sens.ChSensorManager(vehicle.GetSystem())
manager.SetVerbose(False) # Optional: reduce sensor manager console output

# --- Modified IMU Sensor Offset Pose ---
# Common sensor rotation (identity quaternion)
sensor_rotation = chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))

# Define the new offset pose for the IMU sensor
imu_offset_position = chrono.ChVector3d(0, 0, 1) # As per instruction
imu_sensor_offset_pose = chrono.ChFramed(imu_offset_position, sensor_rotation)

# Create an IMU sensor (Accelerometer) and add it to the manager
imu = sens.ChAccelerometerSensor(vehicle.GetChassisBody(), # Body IMU is attached to
                                 10,                     # Update rate in Hz
                                 imu_sensor_offset_pose, # Use modified offset pose
                                 sens.ChNoiseNone())     # Noise model
imu.SetName("IMU Sensor")
imu.SetLag(0)
imu.SetCollectionWindow(0)
imu.PushFilter(sens.ChFilterAccelAccess()) # Provides host access to accelerometer data
manager.AddSensor(imu)

# Define the offset pose for the GPS sensor (retaining original position component)
gps_offset_position = chrono.ChVector3d(-8, 0, 1) # Original position component
gps_sensor_offset_pose = chrono.ChFramed(gps_offset_position, sensor_rotation)

# Create a GPS sensor and add it to the manager
gps = sens.ChGPSSensor(vehicle.GetChassisBody(),                     # Body GPS is attached to
                       10,                                         # Update rate in Hz
                       gps_sensor_offset_pose,                     # Offset pose for GPS
                       chrono.ChVector3d(-89.400, 43.070, 260.0),  # GPS reference point
                       sens.ChNoiseNone())                         # Noise model
gps.SetName("GPS Sensor")
gps.SetLag(0)
gps.SetCollectionWindow(0)
gps.PushFilter(sens.ChFilterGPSAccess()) # Provides host access to GPS data
manager.AddSensor(gps)

# ---------------
# Simulation loop
# ---------------

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)
log_steps = math.ceil(log_step_size / step_size)
# Initialize simulation frame counter
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

gps_data_points = [] # Renamed to avoid conflict with gps sensor object

# Simulation loop
while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Render scene
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Log GPS data
    if step_number % log_steps == 0:
        gps_buffer = gps.GetMostRecentGPSBuffer()
        if gps_buffer and gps_buffer.HasData():
            # GetData() returns a list [latitude, longitude, altitude]
            current_gps_coords = gps_buffer.GetData()
            if current_gps_coords and len(current_gps_coords) == 3:
                gps_data_points.append([current_gps_coords[0], current_gps_coords[1], current_gps_coords[2]])
            # else:
                # print(f"Time: {time:.2f} - GPS data list empty or incomplete.") # For debugging
        # else:
            # print(f"Time: {time:.2f} - GPS buffer has no data.") # For debugging


    # --- Modified Driver Inputs ---
    # Simplified to maintain a constant steering of 0.6 and throttle of 0.5
    driver.SetSteering(0.6)
    driver.SetThrottle(0.5)
    driver.SetBraking(0.0)  # Ensure no braking unless intended

    driver_inputs = driver.GetInputs()

    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Update sensor manager in each step
    manager.Update()

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)

# --- Added Matplotlib Plot for GPS Data ---
print("\nSimulation finished.")
# print("Collected GPS Data: ", gps_data_points) # Optional: print all data

if gps_data_points:
    latitudes = [p[0] for p in gps_data_points]
    longitudes = [p[1] for p in gps_data_points]
    altitudes = [p[2] for p in gps_data_points] # Though not plotted here, it's available

    plt.figure(figsize=(10, 8))
    plt.plot(longitudes, latitudes, marker='.', linestyle='-', color='blue')
    if len(longitudes) > 0: # Plot start and end points
        plt.plot(longitudes[0], latitudes[0], 'go', markersize=10, label='Start')
        plt.plot(longitudes[-1], latitudes[-1], 'ro', markersize=10, label='End')
    plt.xlabel("Longitude (degrees)")
    plt.ylabel("Latitude (degrees)")
    plt.title("GPS Trajectory of HMMWV")
    plt.grid(True)
    plt.legend()
    plt.axis('equal') # Makes the plot scales equal for Lat/Long
    plt.show()
else:
    print("No GPS data was collected to plot.")