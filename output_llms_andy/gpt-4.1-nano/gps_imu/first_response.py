import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

# Initialize the Chrono system
sys = chrono.ChSystemNSC()

# Create the terrain
terrain = veh.RigidTerrain(sys)
terrain.Initialize()

# Add a flat terrain (e.g., a patch of ground)
ground_material = terrain.GetMaterialSurface()
ground_material.SetFriction(0.9)
terrain.AddPatch(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0)), 100, 100, 0.1)

# Create the HMMWV vehicle
vehicle = veh.HMMWV_Full(sys)

# Set initial position
initial_pos = chrono.ChVectorD(0, 0.5, 0)
vehicle.GetChassisBody().SetPos(initial_pos)

# Initialize the vehicle (build the system)
vehicle.Initialize()

# Set visualization using Irrlicht
app = irr.ChIrrApp(vehicle.GetSystem(), 'HMMWV Simulation', irr.dimension2du(1024, 768))
app.AddTypicalSky()
app.AddTypicalCamera(irr.vector3df(0, 3, -8))
app.AddTypicalLights()
app.AssetBindAll()
app.AssetUpdateAll()

# Create driver input
driver = veh.ChIrrGuiDriver(app)

# Add IMU sensor (attached to the chassis)
imu_sensor = veh.ChIMUSensor(vehicle.GetChassisBody(),  # attached to chassis
                              offset=chrono.ChFrameD(chrono.ChVectorD(0, 1.0, 0)),  # position offset
                              sys=sys)
imu_sensor.Initialize()

# Add GPS sensor (attached to the chassis)
gps_sensor = veh.ChGPSSensor(vehicle.GetChassisBody(), sys=sys)
gps_sensor.Initialize()

# Variables to store sensor data
imu_data = {}
gps_data = {}

# Simulation parameters
time_step = 0.005
total_time = 20  # seconds
current_time = 0

# Simulation loop
while app.GetDevice().run():
    # Begin scene rendering
    app.BeginScene()
    app.DrawAll()

    # Driver inputs (e.g., throttle, steering)
    driver.BeginFrame()
    driver.Synchronize(current_time)
    driver.Advance(time_step)
    driver.EndFrame()

    # Get driver inputs
    throttle_input = driver.GetThrottle()
    steering_input = driver.GetSteering()

    # Apply driver inputs to vehicle
    vehicle.GetDriveTrain()->SetThrottle(throttle_input)
    vehicle.GetSteering()->SetSteering(steering_input)

    # Synchronize and advance the vehicle
    vehicle.Synchronize(current_time, chrono.VNULL, chrono.VNULL)
    vehicle.Advance(time_step)

    # Update terrain
    terrain.Synchronize(current_time)
    terrain.Advance(time_step)

    # Update sensors
    imu_sensor.Synchronize(current_time)
    imu_sensor.GetSensorData(imu_data)

    gps_sensor.Synchronize(current_time)
    gps_sensor.GetSensorData(gps_data)

    # Output vehicle mass
    print(f"Vehicle mass: {vehicle.GetVehicleMass()} kg")

    # Optionally, print sensor data
    # print(f"IMU data: {imu_data}")
    # print(f"GPS data: {gps_data}")

    # Finish rendering
    app.EndScene()

    # Increment time
    current_time += time_step

    # Check for user exit
    if app.GetDevice().isWindowClosed():
        break

# Cleanup
app.GetDevice().closeDevice()