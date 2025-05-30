import pychrono.core as chrono
import pychrono.vehicle as vvehicle
import pychrono.sensor as vsensor

# Initialize Chrono physical system
sys = chrono.ChSystemNSC()

# Enable contact contact method and gravity
sys.SetGravity(chrono.ChVectorD(0, -9.81, 0))

# Create terrain
terrain = vvehicle.RigidTerrain(sys)
terrain.Initialize()

# Add a flat ground surface
ground = terrain.GetGround()
# Set some visual/physical properties as needed
# Using default properties here

# Create HMMWV Vehicle
# The full vehicle has detailed setup; using provided example
# For simplicity, we will use the HMMWV model with full setup
hmmwv = vvehicle.HMMWV_Full(sys)

# Set the initial position of the vehicle
initial_pos = chrono.ChVectorD(0, 0.5, 0)  # slightly above ground to avoid initial penetration
hmmwv.GetChassisBody().SetPos(initial_pos)

# Initialize the vehicle
hmmwv.Initialize(initial_pos)

# Setup the driver (simulate user input)
driver = vvehicle.ChDriver(hmmwv.GetVehicle(), 0)
driver.SetInputData(0, 0, 0, 0)  # steer, throttle, brake, clutch
driver.Initialize()

# Setup Irrlicht visualization
irrlicht = vvehicle.ChIrrApp(hmmwv.GetVehicle(), 'PyChrono HMMWV Simulation', chrono.dimension2du(1024,768))
irrlicht.AddLight(chrono.ChVectorD(4, 20, 0),  # Light position
                   chrono.ChVectorD(0.2, -1, -0.3),  # Light direction
                   1.0, 1.0, 1.0)  # RGB intensity
irrlicht.AssetBindAll()
irrlicht.AssetUpdateAll()

# Set up sensor manager
sensor_manager = vsensor.ChSensorManager()
sensor_manager.scene = irrlicht.GetScene()

# Create IMU sensor attached to the chassis
imu = vsensor.ChSensor(
    hmmwv.GetChassisBody(),
    vsensor.E_SENSORTYPE_INERTIAL,
    1.0  # update rate in Hz
)
imu.SetName("IMU_Sensor")
sensor_manager.AddSensor(imu)

# Create GPS sensor attached to the chassis
gps = vsensor.ChSensor(
    hmmwv.GetChassisBody(),
    vsensor.E_SENSORTYPE_GPS,
    1.0  # update rate in Hz
)
gps.SetName("GPS_Sensor")
sensor_manager.AddSensor(gps)

# Variables for sensor data processing
imu_data = None
gps_data = None

# Simulation settings
run_time = 20  # seconds
step_size = 1e-3  # 1 ms
steps = int(run_time / step_size)

# Main simulation loop
for step in range(steps):
    time = step * step_size

    # Update driver controls
    driver.ComputeControls()

    # Synchronize and advance the vehicle
    hmmwv.GetVehicle().SyncCollisionModels()
    hmmwv.GetVehicle().Advance(step_size)

    # Update terrain
    terrain.Advance(step_size)

    # Process sensors
    sensor_manager.Update()

    # Retrieve IMU data
    imu_data = imu.GetMostRecentRawData()
    if imu_data:
        # Process IMU data if needed
        pass

    # Retrieve GPS data
    gps_data = gps.GetMostRecentRawData()
    if gps_data:
        # Process GPS data if needed
        pass

    # Update visualization
    irrlicht.BeginScene()
    irrlicht.DrawAll()
    irrlicht.EndScene()

    # Output vehicle mass at each step
    vehicle_mass = hmmwv.GetVehicle().GetMass()
    print(f"Time: {time:.2f}s, Vehicle Mass: {vehicle_mass} kg")

# Finalize visualization
irrlicht.GetDevice().closeDevice()