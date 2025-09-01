import pychrono as chrono
import pychrono_vehicle as veh
import pychrono_sensor

# 1. Initialize the PyChrono environment and core components.

# Set Chrono to realtime speed and verbose output off
chrono.SetChronoDataPath(chrono.GetChronoDataPath())  # Ensure data path is set

# Create the physical system (with default solver, system parameters)
system = chrono.ChSystemNSC()

# Create the HMMWV vehicle with full system setup (powertrain, suspension, tires, driver inputs)
# Use default locations for model files and parameters
vehicle = veh.HMMWV_Full(system, chrono.vehicle.CollideType_NONE)

# Initialize the vehicle at origin, zero orientation
init_loc = chrono.ChVectorD(0, 0, 1.0)  # Start a bit above ground level
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)
vehicle.Initialize(chrono.ChCoordsysD(init_loc, init_rot))

# Set vehicle parameters, e.g. no step size here yet:
vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)

# Create the terrain using rigid terrain or vehicle terrain; use default patch.
terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)),
                         150.0, 150.0)
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 16, 16)
patch.SetContactFriction(0.9f)
patch.SetContactRestitution(0.01f)
patch.SetContactMaterialProperties(2e7, 0.3)
terrain.Initialize()

# Create and initialize the driver system (lookup driver inputs)
time_step = 1e-3
driver = veh.HMMWV_Driver(vehicle, veh.GetDataFile("vehicle/hmmwv/driver/HMMWV_acc_very_smooth.json"))
driver.Initialize()

# 2. Add IMU and GPS sensors attached to vehicle chassis

# Sensor manager (attach to the system)
sensor_manager = pychrono_sensor.SensorManager(system)
sensor_manager.SetVerbose(False)

# Add IMU sensor (accelerometer + gyro)
imu_offset_pose = chrono.ChFrameD()  # Attach at chassis center frame
imu_sensor = pychrono_sensor.ChSensor(
    vehicle.GetChassisBody(), 
    chrono.ChFrameD(imu_offset_pose.GetPos(), imu_offset_pose.GetRot()),
    1.0 / 60.0,  # update rate 60 Hz
    True,        # enabled sensor
)
imu_sensor.PushFilter(pychrono_sensor.ChFilterAccelNoise(0.05))  # Add some noise filter
imu_sensor.PushFilter(pychrono_sensor.ChFilterGyroNoise(0.01))
sensor_manager.AddSensor(imu_sensor)

# Add GPS sensor (position only)
gps_offset_pose = chrono.ChFrameD(chrono.ChVectorD(0, 0, 1))  # 1m above chassis origin
gps_sensor = pychrono_sensor.ChSensor(
    vehicle.GetChassisBody(),
    gps_offset_pose,
    10.0,  # gps update rate 10 Hz
    True,
)
gps_sensor.PushFilter(pychrono_sensor.ChFilterGPSNoise(1.5))  # 1.5 m noise typical GPS
sensor_manager.AddSensor(gps_sensor)

# 3. Setup Irrlicht visualization for vehicle

# Create the Irrlicht application with vehicle and scene
app = veh.ChIrrApp(vehicle.GetVehicle(), "HMMWV Simulation with Sensors", chrono.dimension2du(1280, 720))

app.AddTypicalLights()
app.AddTypicalCamera(chrono.ChVectorD(-8, -11, 4), chrono.ChVectorD(0, 0, 1))
app.SetTimestep(time_step)
app.AssetBindAll()
app.AssetUpdateAll()

# Set simulation end time
end_time = 20.0

# Main simulation loop

# Set initial step counters for sensors (sensor manager handles update at sensor update rates)
while app.GetDevice().run() and system.GetChTime() < end_time:
    # Collect driver inputs and update
    driver_inputs = driver.GetInputs()

    # Synchronize modules with current time and inputs
    time = system.GetChTime()
    driver.Synchronize(time)
    vehicle.Synchronize(time, driver.GetInputs(), terrain)
    terrain.Synchronize(time)
    sensor_manager.Update()  # update sensors (calls sensor updates as needed)
    app.BeginScene()
    app.DrawAll()

    # Output current vehicle mass and some info
    print(f"Time: {time:.3f}, Vehicle Mass: {vehicle.GetVehicle().GetMass():.1f} kg")

    # Advance simulation by fixed time step
    driver.Advance(time_step)
    vehicle.Advance(time_step)
    terrain.Advance(time_step)
    system.DoStepDynamics(time_step)
    app.EndScene()