import pychrono as chrono
import pychrono.vehicle as vehicle
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr
import os
import math

# -----------------------------------------------------------------------------
# Set Chrono data paths
# -----------------------------------------------------------------------------
chrono.SetChronoDataPath(os.environ.get("CHRONO_DATA_DIR", "../../chrono_data/"))
vehicle.SetDataPath(os.path.join(chrono.GetChronoDataPath(), 'vehicle/'))
sens.SetSensorDataPath(os.path.join(chrono.GetChronoDataPath(), 'sensor/'))

# -----------------------------------------------------------------------------
# Simulation parameters
# -----------------------------------------------------------------------------
step_size = 2e-3  # Time step for dynamics and visualization
time_end = 100    # End time for simulation

# Initial vehicle position and orientation
init_loc = chrono.ChVectorD(0, 0, 0.5)
init_rot = chrono.ChQuaternionD(1, 0, 0, 0) # Identity quaternion

# GPS Reference Point (latitude, longitude, altitude)
# Example: Ann Arbor, MI
gps_reference = chrono.ChVectorD(42.2808, -83.7430, 250.0) # Lat, Long, Alt

# Sensor update rates
imu_update_rate = 100  # Hz
gps_update_rate = 10   # Hz

# -----------------------------------------------------------------------------
# 1. Initialize PyChrono environment and core components
# -----------------------------------------------------------------------------
print("Initializing Chrono system...")
# Create the Chrono physical system
# Using ChSystemNSC for non-smooth contacts (default for vehicle simulations)
my_system = chrono.ChSystemNSC()
my_system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81)) # Set gravity

# Set solver settings (optional, but good for stability)
my_system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
my_system.SetSolverMaxIterations(150)
my_system.SetMaxPenetrationRecoverySpeed(4.0)

# -----------------------------------------------------------------------------
# 2. Add required physical systems and objects
# -----------------------------------------------------------------------------

# --- Create the HMMWV vehicle ---
print("Creating HMMWV vehicle...")
my_hmmwv = vehicle.HMMWV_Full("myHMMWV")
my_hmmwv.SetContactMethod(chrono.ChContactMethod_NSC) # Or SMC if system is SMC
my_hmmwv.SetChassisFixed(False)
my_hmmwv.SetInitPosition(chrono.ChCoordsysD(init_loc, init_rot))
my_hmmwv.SetPowertrainType(vehicle.PowertrainModelType_SHAFTS) # More detailed powertrain
my_hmmwv.SetDriveType(vehicle.DrivelineTypeWV_AWD)
my_hmmwv.SetSteeringType(vehicle.SteeringTypeWV_PITMAN_ARM)
my_hmmwv.SetTireType(vehicle.TireModelType_TMEASY) # TMeasy tire model
# my_hmmwv.SetTireType(vehicle.TireModelType_RIGID) # Simpler rigid tire model if preferred
my_hmmwv.SetTireStepSize(step_size)
my_hmmwv.Initialize()

# Set visualization options for the vehicle
my_hmmwv.SetChassisVisualizationType(vehicle.VisualizationType_PRIMITIVES)
my_hmmwv.SetSuspensionVisualizationType(vehicle.VisualizationType_PRIMITIVES)
my_hmmwv.SetSteeringVisualizationType(vehicle.VisualizationType_PRIMITIVES)
my_hmmwv.SetWheelVisualizationType(vehicle.VisualizationType_MESH) # Use MESH for wheels
my_hmmwv.SetTireVisualizationType(vehicle.VisualizationType_MESH)   # Use MESH for tires

# Get the vehicle and chassis body (for sensor attachment)
veh = my_hmmwv.GetVehicle()
chassis_body = veh.GetChassisBody()
print(f"Initial Vehicle Mass: {veh.GetMass()} kg")

# --- Create the Terrain ---
print("Creating terrain...")
terrain = vehicle.RigidTerrain(my_system)
patch_mat = chrono.ChMaterialSurfaceNSC() # Ensure it matches system's contact method
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
# Add a large flat patch of ground
patch = terrain.AddPatch(patch_mat,
                         chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 1), # CSYS, normal
                         200, 200) # Size (length, width)
patch.SetTexture(vehicle.GetDataFile("terrain/textures/tile4.jpg"), 200, 200) # Texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# --- Create the Irrlicht Visualization Application ---
print("Creating Irrlicht visualization...")
# ChWheeledVehicleIrrApp is a helper class for common vehicle visualization
# It internally creates an Irrlicht application, camera, lights, etc.
# and handles basic vehicle controls if a ChIrrGuiDriver is used.
app = vehicle.ChWheeledVehicleIrrApp(veh, "HMMWV Sensor Demo")
app.SetSkyBox()
app.AddTypicalLights()
app.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 1.75), 6.0, 0.5) # Point, dist, height
# Set the time step for the Irrlicht application
app.SetTimestep(step_size)

# Bind all assets for Irrlicht (meshes, textures)
app.AssetBindAll()
app.AssetUpdateAll()

# --- Create the Driver ---
print("Creating driver system...")
# Use ChIrrGuiDriver for interactive control via Irrlicht GUI
driver = vehicle.ChIrrGuiDriver(app)
# Set the path to steering controller JSON files (normalize if needed)
driver.SetSteeringControllerFile(vehicle.GetDataFile("hmmwv/SteeringController.json"))
driver.SetSpeedControllerFile(vehicle.GetDataFile("hmmwv/SpeedController.json"))
driver.Initialize()
app.SetDriver(driver) # Link driver to the app for control processing

# --- Create Sensor Manager and Sensors ---
print("Creating sensor manager and sensors...")
sensor_manager = sens.ChSensorManager(my_system)
# Set sensor manager to update at a higher frequency than simulation step for sub-sampling if needed
# or equal to simulation step_size for lock-step update.
# For simplicity, let's update sensors after each dynamics step.
# If sensor_manager.SetMaxFrameRate(1/step_size) or similar is used,
# Update() will handle if it's time to acquire data.
# The individual sensor update rates will determine how often they *actually* produce new data.

# Define an offset pose for sensors relative to the chassis
# For example, place them slightly forward and up on the chassis
sensor_offset_pose = chrono.ChFrameD(chrono.ChVectorD(0.5, 0, 0.3), chrono.Q_from_AngAxis(0, chrono.VECT_Y))

# --- IMU Sensor ---
print(f"  Adding IMU sensor with update rate: {imu_update_rate} Hz")
imu_sensor = sens.ChIMUSensor(
    chassis_body,          # Parent body
    imu_update_rate,       # Update rate in Hz
    sensor_offset_pose,    # Offset pose relative to parent
    sens.ChNoiseNone()     # Noise model (can be ChNoiseNormal, etc.)
)
imu_sensor.SetName("IMU_Sensor")
imu_sensor.SetLag(0.0) # No lag
imu_sensor.SetCollectionWindow(0.0) # Collect data instantaneously
sensor_manager.AddSensor(imu_sensor)

# --- GPS Sensor ---
print(f"  Adding GPS sensor with update rate: {gps_update_rate} Hz")
gps_sensor = sens.ChGPSSensor(
    chassis_body,          # Parent body
    gps_update_rate,       # Update rate in Hz
    sensor_offset_pose,    # Offset pose relative to parent
    gps_reference,         # GPS reference point (lat, long, alt)
    sens.ChNoiseNone()     # Noise model
)
gps_sensor.SetName("GPS_Sensor")
gps_sensor.SetLag(0.0)
gps_sensor.SetCollectionWindow(0.0)
sensor_manager.AddSensor(gps_sensor)

# If rendering sensors, you'd set the scene for the manager:
# sensor_manager.SetScene(my_system) # Not strictly needed for IMU/GPS unless visualizing their frames

# -----------------------------------------------------------------------------
# 3. Simulation Loop
# -----------------------------------------------------------------------------
print("\nStarting simulation loop...")
realtime_timer = chrono.ChRealtimeStepTimer()
current_time = 0

frame_counter = 0
output_frequency = int(1.0 / step_size / 2) # Output data approx every 0.5 seconds

while app.GetDevice().run():
    current_time = my_system.GetChTime()
    if current_time >= time_end:
        break

    # Get driver inputs (steering, throttle, braking)
    # For ChIrrGuiDriver, inputs are processed by app.BeginScene() and driver.Synchronize()
    driver_inputs = driver.GetInputs() # This is a vehicle.DriverInputs object

    # Synchronize modules
    # Driver system: Process GUI inputs
    driver.Synchronize(current_time)
    # Terrain system: Potentially update based on vehicle (e.g., deformable terrain)
    terrain.Synchronize(current_time)
    # Vehicle system: Apply driver inputs, update forces based on terrain interaction
    my_hmmwv.Synchronize(current_time, driver_inputs, terrain)
    # Irrlicht application: Synchronize with vehicle state for camera, etc.
    # Also handles driver input mode string for display.
    app.Synchronize(driver.GetInputModeAsString(), driver_inputs.m_steering, driver_inputs.m_throttle, driver_inputs.m_braking)

    # Advance the state of modules (internal dynamics, before global physics step)
    driver.Advance(step_size)
    terrain.Advance(step_size)
    my_hmmwv.Advance(step_size) # This applies forces based on current inputs & state
    app.Advance(step_size)      # Advance Irrlicht app's internal time

    # Advance the physical system (perform integration step)
    my_system.DoStepDynamics(step_size)

    # Update sensor manager (this will trigger data acquisition for sensors whose update time has come)
    sensor_manager.Update()

    # Process and output sensor data
    if frame_counter % output_frequency == 0:
        print(f"\n--- Time: {current_time:.3f} s ---")
        print(f"Vehicle Position: {veh.GetPos().x:.2f}, {veh.GetPos().y:.2f}, {veh.GetPos().z:.2f}")
        print(f"Vehicle Speed: {veh.GetSpeed():.2f} m/s")
        print(f"Vehicle Mass: {veh.GetMass()} kg") # As requested

        # IMU Data
        imu_buffer = imu_sensor.GetMostRecentBufferIMU()
        if imu_buffer.HasData():
            imu_data = imu_buffer.GetLastData() # Get the most recent ChIMUData
            print(f"  IMU Accel: ({imu_data.Accel[0]:.2f}, {imu_data.Accel[1]:.2f}, {imu_data.Accel[2]:.2f}) m/s^2")
            print(f"  IMU Gyro:  ({imu_data.Gyro[0]:.2f}, {imu_data.Gyro[1]:.2f}, {imu_data.Gyro[2]:.2f}) rad/s")
            # Quaternion data is also available: imu_data.Quaternion
        else:
            print("  IMU: No new data")

        # GPS Data
        gps_buffer = gps_sensor.GetMostRecentBufferGPS()
        if gps_buffer.HasData():
            gps_data = gps_buffer.GetLastData() # Get the most recent ChGPSData
            print(f"  GPS Coords (Lat,Lon,Alt): ({gps_data.Latitude:.6f}, {gps_data.Longitude:.6f}, {gps_data.Altitude:.2f})")
            print(f"  GPS Time: {gps_data.Time}")
        else:
            print("  GPS: No new data")

    # Render scene
    app.BeginScene(True, True, chronoirr.SColor(255, 140, 160, 190)) # Clear back buffer, z buffer, set color
    app.DrawAll()
    app.EndScene()

    # Try to maintain real-time progression (optional)
    # realtime_timer.Spin(step_size)

    frame_counter += 1

# -----------------------------------------------------------------------------
# Simulation cleanup
# -----------------------------------------------------------------------------
print("\nSimulation ended.")
# No explicit cleanup needed for most PyChrono objects, Python's GC handles it.
# However, Irrlicht device might need explicit drop if not managed by app's destructor.
# ChWheeledVehicleIrrApp should handle this.